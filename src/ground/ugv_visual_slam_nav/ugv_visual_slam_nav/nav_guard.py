"""导航守卫：路径重规划节流、卡住恢复、两段式到点确认。

离线策略（`policies.SlamNavPolicy`）和 ROS 导航节点（`nav_node.NavNode`）
共用这一份状态机。早先两边各写了一套——节点侧只有 goal → A* → pure_pursuit →
avoid_from_depth，没有卡住恢复也没有到点复核，于是「离线跑出的成功率」和
「`ros2 launch` 实际跑的节点」行为并不一致。抽出来以后两边是同一条控制律。

调用方需要提供：
- `self.path` / `self.last_plan` 等状态（由 `_reset_planner()` 初始化）
- `_cell_blocked(x, y)`：世界坐标点是否被占据（恢复动作后退前的安全检查）
"""

import math

import numpy as np

from . import params as P
from .controllers import goal_reached
from .planner import astar, inflate, simplify

INFLATE_CELLS = 5        # 0.5 m ≈ 车半径 + 纯追踪横向误差余量
REPLAN_PERIOD = 2.0      # 最小重规划间隔（秒）
STUCK_STEPS = 15         # 连续这么多步几乎没位移就判为卡住
STOP_TOL = P.GOAL_TOL * 0.55   # 停车判据比评测判据更严，给估计误差留余量
SETTLE_STEPS = 50              # 到点确认：原地扫两圈（约 5 s）再最终停车


class NavGuard:
    """维护全局路径、重规划节流、卡住恢复与到点确认。"""

    def _reset_planner(self):
        self.path = []
        self.last_plan = -1e9
        self._prev_pose = None
        self._stuck = 0
        self._recover = 0
        self._last_v = 0.0
        self._settle = -1          # -1 = 未触发；>0 = 剩余确认步数；0 = 已确认

    def _settle_goal(self, pose, goal, entry_tol=None):
        """两段式到点确认。

        SLAM 在终点角落可能带着 0.7 m 的漂移"自信停车"（估计已到点，
        真实还差 0.5 m）。对策：第一次满足停车判据时不立即停，先原地转
        一圈让相机从新角度扫到路标，EKF 吃到这批观测后漂移会明显收敛，
        再按放宽到 1.6 倍的判据做最终确认。真实机器人也确实会做这种
        "到点复核"动作。

        `entry_tol` 留给调用方指定"进入确认"的半径：离线策略用默认的
        `STOP_TOL`（更严，先把车压到 0.19 m 再复核）；ROS 节点沿用原来的
        0.35 m 到点判据 `GOAL_TOL`，避免换实现后反而停不下来。
        """
        enter = STOP_TOL if entry_tol is None else float(entry_tol)
        if self._settle < 0:
            if goal_reached(pose, goal, tol=enter):
                self._settle = SETTLE_STEPS
                self.path = []
            return False
        if self._settle > 0:
            self._settle -= 1
            return 'spin'
        # 复核判据不低于进入判据，否则自转期间的位姿漂移会让车再也停不下来
        return goal_reached(pose, goal, tol=max(STOP_TOL * 1.6, enter))

    def _plan(self, blocked, pose, goal, cost=None):
        path = astar(blocked, (pose[0], pose[1]), (goal[0], goal[1]), cost=cost)
        if path:
            # 视线简化用更保守的掩码：否则拉直后的线段又会贴回障碍边缘
            path = simplify(inflate(blocked, 2), path)
        return path

    def _path_deviation(self, pose):
        pts = np.asarray(self.path, dtype=np.float64)
        return float(np.min(np.hypot(pts[:, 0] - pose[0], pts[:, 1] - pose[1])))

    def _should_replan(self, pose, goal, now, always=False):
        """always=True 表示地图在变，过一段时间就无条件重规划。"""
        if not self.path:
            return True
        if now - self.last_plan < REPLAN_PERIOD:
            return False
        if always:
            return True
        # 路径终点已经不再是目标（例如被 frontier 目标替换过）
        if math.hypot(self.path[-1][0] - goal[0], self.path[-1][1] - goal[1]) > 1.0:
            return True
        return self._path_deviation(pose) > P.REPLAN_DIST

    def _cell_blocked(self, x, y):
        """子类覆盖：判断世界坐标点是否在障碍内（用于恢复动作的安全性检查）。"""
        return False

    def _recovery_cmd(self, pose):
        """恢复动作：先原地转向，实在出不去再低速后退。

        原地转向不改变车心位置，在当前尚未碰撞的前提下**不可能**撞上；但
        如果车头正对着墙、前方深度一直起不来，只转向也出不去，这时需要后退。
        后退要把车尾送进未知区域，所以先用地图查一下车尾那一格——只有确认
        是空的才退。
        """
        if self._recover > 5:
            return 0.0, 1.0
        bx = pose[0] - 0.42 * math.cos(pose[2])
        by = pose[1] - 0.42 * math.sin(pose[2])
        if self._cell_blocked(bx, by):
            return 0.0, 1.0
        return -0.18, 0.55

    def _check_stuck(self, pose):
        """几乎不动就进入恢复模式：原地转向。

        这里只转向、不后退。碰撞判据是"以车心为圆心的车体圆是否与障碍相交"，
        原地转向不改变圆心位置，因此在当前尚未碰撞的前提下转向**不可能**撞上；
        而倒车有可能把车尾送进墙里 —— 早先版本在 cluttered/seed 2 上就是
        这样把自己顶到西墙上撞掉的。
        """
        if self._recover > 0:
            self._recover -= 1
            self.path = []
            return True
        # 末端收敛段的速度只有 3 cm/s，一步走不到 2 mm。这种"慢"是故意的，
        # 不能算卡死 —— 否则车会在终点前反复进入恢复转向，永远停不下来。
        if self._last_v < 0.05:
            self._stuck = 0
            self._prev_pose = np.asarray(pose, dtype=np.float64).copy()
            return False
        if self._prev_pose is None:
            self._prev_pose = np.asarray(pose, dtype=np.float64).copy()
            return False
        moved = math.hypot(pose[0] - self._prev_pose[0], pose[1] - self._prev_pose[1])
        self._stuck = self._stuck + 1 if moved < 0.02 else 0
        self._prev_pose = np.asarray(pose, dtype=np.float64).copy()
        if self._stuck > STUCK_STEPS:
            self._stuck = 0
            self._recover = 12
            self.path = []
            return True
        return False
