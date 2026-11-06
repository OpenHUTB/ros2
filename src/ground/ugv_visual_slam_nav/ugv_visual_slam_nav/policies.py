"""三种策略，共用同一个 act(obs) 接口：

- OraclePolicy  ：真值位姿 + 真值地图 + A* + 纯追踪，作为性能上界
- SlamNavPolicy ：EKF-SLAM 位姿 + 视觉栅格 + A* + 纯追踪，完整流水线
- E2EPolicy     ：CNN 从图像直接回归 (v, w)，不起地图、不做规划
"""

import math

import numpy as np

from . import params as P
from .camera import resize_gray, to_gray
from .controllers import avoid_from_depth, goal_reached, pure_pursuit
from .gridmap import GridMap, truth_grid
from .kinematics import wrap
from .planner import astar, inflate, proximity_cost, simplify
from .slam import EKFSLAM

INFLATE_CELLS = 5        # 0.5 m ≈ 车半径 + 纯追踪横向误差余量
SPIN_TIME = 1.6          # 起步原地扫描建图的时间（秒）
REPLAN_PERIOD = 2.0      # 最小重规划间隔（秒）
STUCK_STEPS = 15         # 连续这么多步几乎没位移就判为卡住
STOP_TOL = P.GOAL_TOL * 0.55   # 停车判据比评测判据更严，给估计误差留余量
SETTLE_STEPS = 50              # 到点确认：原地扫两圈（约 5 s）再最终停车


def odom_delta(prev, cur):
    """两个里程计位姿之间的车体系增量。"""
    dx = cur[0] - prev[0]
    dy = cur[1] - prev[1]
    c, s = math.cos(prev[2]), math.sin(prev[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(cur[2] - prev[2]))


def mark_circle(mask, cx, cy, radius):
    """把世界坐标里的一个圆写进阻塞栅格。"""
    r0 = max(0, int((cy - radius) / P.GRID_RES))
    r1 = min(P.GH - 1, int((cy + radius) / P.GRID_RES))
    c0 = max(0, int((cx - radius) / P.GRID_RES))
    c1 = min(P.GW - 1, int((cx + radius) / P.GRID_RES))
    if r0 > r1 or c0 > c1:
        return mask
    rr, cc = np.ogrid[r0:r1 + 1, c0:c1 + 1]
    wy = (rr + 0.5) * P.GRID_RES
    wx = (cc + 0.5) * P.GRID_RES
    mask[r0:r1 + 1, c0:c1 + 1] |= ((wx - cx) ** 2 + (wy - cy) ** 2) <= radius ** 2
    return mask


def _blocked_at(mask, x, y, rad=1):
    """世界坐标点 (x, y) 附近 rad 格的栅格是否被占据；越界按占据处理。"""
    c = int(x / P.GRID_RES)
    r = int(y / P.GRID_RES)
    if not (0 <= r < P.GH and 0 <= c < P.GW):
        return True
    sub = mask[max(0, r - rad):r + rad + 1, max(0, c - rad):c + rad + 1]
    return bool(sub.size) and bool(sub.any())


class _PlannerMixin:
    """维护全局路径、重规划节流与卡住恢复。"""

    def _reset_planner(self):
        self.path = []
        self.last_plan = -1e9
        self._prev_pose = None
        self._stuck = 0
        self._recover = 0
        self._last_v = 0.0
        self._settle = -1          # -1 = 未触发；>0 = 剩余确认步数；0 = 已确认

    def _settle_goal(self, pose, goal):
        """两段式到点确认。

        SLAM 在终点角落可能带着 0.7 m 的漂移"自信停车"（估计已到点，
        真实还差 0.5 m）。对策：第一次满足停车判据时不立即停，先原地转
        一圈让相机从新角度扫到路标，EKF 吃到这批观测后漂移会明显收敛，
        再按放宽到 1.6 倍的判据做最终确认。真实机器人也确实会做这种
        "到点复核"动作。
        """
        if self._settle < 0:
            if goal_reached(pose, goal, tol=STOP_TOL):
                self._settle = SETTLE_STEPS
                self.path = []
            return False
        if self._settle > 0:
            self._settle -= 1
            return 'spin'
        return goal_reached(pose, goal, tol=STOP_TOL * 1.6)

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


class OraclePolicy(_PlannerMixin):
    """性能上界：位姿与静态地图都是真值，动态障碍按当前真实位置参与规划。"""

    name = 'oracle'
    label = 'Oracle (ground-truth pose + map)'

    def __init__(self, world):
        self.truth = truth_grid(world)
        self.blocked = inflate(self.truth, INFLATE_CELLS)
        self.cost = proximity_cost(self.truth)
        self._reset_planner()
        self._last_dyn = None

    def reset(self, world):
        self.__init__(world)

    def _cell_blocked(self, x, y):
        return _blocked_at(self.blocked, x, y)

    @staticmethod
    def _in_patrol_band(pose, obs):
        """车是否处于移动障碍的巡逻带内（axis=0 表示沿 x 轴往复）。"""
        dyn = obs.get('dyn_center')
        if dyn is None:
            return False
        if int(obs.get('dyn_axis', 0)) == 0:
            return abs(float(pose[1]) - float(dyn[1])) < 0.9
        return abs(float(pose[0]) - float(dyn[0])) < 0.9

    def act(self, obs):
        pose = obs['true_pose']
        settle = self._settle_goal(pose, obs['goal'])
        if settle is True:
            return 0.0, 0.0
        if settle == 'spin':
            return 0.0, 1.0

        if self._check_stuck(pose):
            return self._recovery_cmd(pose)

        dyn = obs.get('dyn_center')
        # 让行：移动障碍逼近且自己不在它的巡逻带里时，停车等它走过去。
        # 若自己已经在带内，停车反而会被扫到，必须快速通过。
        if dyn is not None and not self._in_patrol_band(pose, obs):
            if float(np.linalg.norm(dyn - np.asarray(pose[:2]))) < 1.0:
                self.path = []
                return 0.0, 0.0
        need = self._should_replan(pose, obs['goal'], obs['t'])
        if dyn is not None and (self._last_dyn is None
                                or float(np.linalg.norm(dyn - self._last_dyn)) > 0.30):
            need = True

        if need:
            blk = self.blocked
            if dyn is not None:
                blk = self.blocked.copy()
                r_dyn = float(obs.get('dyn_radius', 0.3))
                mark_circle(blk, dyn[0], dyn[1], r_dyn + 0.42)
                self._last_dyn = np.asarray(dyn, dtype=np.float64).copy()
            self.path = self._plan(blk, pose, obs['goal'], cost=self.cost)
            self.last_plan = obs['t']

        if self.path:
            v, w = pure_pursuit(pose, self.path)
        else:
            v, w = 0.0, 0.9
        # 动态障碍会跑到已规划路径上，这里保留一层近距离反应式避障
        # 记录的是"规划层希望走多快"，不是避障砍完之后的速度：
        # 被避障砍到 0 却一直不动，才是真的卡住，需要恢复动作。
        self._last_v = float(v)
        v, w = avoid_from_depth(obs['depth'], v, w)
        return v, w


class SlamNavPolicy(_PlannerMixin):
    name = 'slam_nav'
    label = 'EKF-SLAM + grid + A*'

    def __init__(self, world):
        self.slam = EKFSLAM(world.start)
        self.grid = GridMap()
        self._reset_planner()
        self.n_lm_seen = 0

    def reset(self, world):
        self.__init__(world)

    def _cell_blocked(self, x, y):
        return _blocked_at(self.grid.occupied(th=0.62), x, y, rad=2)

    # ---------------------------------------------------------- frontier
    def _frontier_target(self, pose, max_r=5.0):
        """最近的可达前沿：已知空地与未探索区的交界。"""
        free = self.grid.free_mask()
        unknown = ~self.grid.explored()
        fr = free & inflate(unknown, 1)
        if not fr.any():
            return None
        rr, cc = np.nonzero(fr)
        wx = (cc + 0.5) * P.GRID_RES
        wy = (rr + 0.5) * P.GRID_RES
        d = np.hypot(wx - pose[0], wy - pose[1])
        keep = d < max_r
        if not keep.any():
            return None
        i = int(np.argmin(np.where(keep, d, np.inf)))
        return (float(wx[i]), float(wy[i]))

    def act(self, obs):
        dx, dy, dth = obs['odom_delta']
        self.slam.predict(dx, dy, dth)
        self.slam.update(obs['landmarks'])
        self.grid.update(self.slam.pose, obs['wall_pts'])
        # 路标是真实障碍物：用 SLAM 估计的位置把它标进地图
        for mx, my, _lid in self.slam.landmark_xy():
            self.grid.mark_obstacle(mx, my)
        self.n_lm_seen = len(self.slam.ids)

        pose = self.slam.pose
        settle = self._settle_goal(pose, obs['goal'])
        if settle is True:
            return 0.0, 0.0
        if settle == 'spin':
            return 0.0, 1.0

        if self._check_stuck(pose):
            return self._recovery_cmd(pose)

        # 起步阶段先原地转一圈，把周围地图攒起来再规划
        if obs['t'] < SPIN_TIME:
            return 0.12, 1.00

        if self._should_replan(pose, obs['goal'], obs['t'], always=True):
            occ = self.grid.occupied(th=0.62)
            blocked = inflate(occ, INFLATE_CELLS)
            cost = proximity_cost(occ) + np.where(self.grid.explored(), 0.0, 1.8)
            self.path = self._plan(blocked, pose, obs['goal'], cost=cost)
            if not self.path:
                # 目标暂时不可达：先去最近的前沿探一探
                ft = self._frontier_target(pose)
                if ft is not None:
                    self.path = self._plan(blocked, pose, ft, cost=cost)
            self.last_plan = obs['t']

        if self.path:
            v, w = pure_pursuit(pose, self.path)
        else:
            v, w = 0.0, 0.9
        # 记录的是"规划层希望走多快"，不是避障砍完之后的速度：
        # 被避障砍到 0 却一直不动，才是真的卡住，需要恢复动作。
        self._last_v = float(v)
        v, w = avoid_from_depth(obs['depth'], v, w)
        return v, w


def goal_channel(odom_pose, goal, w=None, h=None):
    """把"目标在车体系下的方位"画成一条竖直亮带（HUD 式目标指示）。

    为什么必须有这一路：三房间 S 形走廊里，同一幅画面既可能对应"直行"也可能
    对应"左转"，正确动作取决于目标在哪儿，而目标完全不在图像里。纯图像策略
    在这种环境下不是学不好，而是**不可辨识** —— 行为克隆只能学到几种情形的
    均值，然后一头撞墙。方位由里程计位姿与给定目标算出，不依赖地图、不依赖
    SLAM，是任何一台车本来就有的量。

    亮带位置 = 方位角在视野内的归一化横坐标；亮度 = 1 - d/视线量程。
    """
    w = E2EPolicy.IN_W if w is None else int(w)
    h = E2EPolicy.IN_H if h is None else int(h)
    out = np.zeros((h, w), dtype=np.float32)
    if goal is None or odom_pose is None:
        return out
    dx = float(goal[0]) - float(odom_pose[0])
    dy = float(goal[1]) - float(odom_pose[1])
    dist = math.hypot(dx, dy)
    beta = wrap(math.atan2(dy, dx) - float(odom_pose[2]))
    half = math.radians(P.CAM_FOV) * 0.5
    u = beta / half if half > 0 else 0.0            # -1 ~ 1 表示视野内
    # 亮度随距离衰减但不归零：目标在量程外时仍要"知道往哪边走"，
    # 所以衰减项只压到 0.4 倍，用世界对角线做归一化而不是传感器量程。
    span = math.hypot(P.WORLD_W, P.WORLD_H)
    amp = float(1.0 - 0.6 * min(1.0, dist / span))
    if abs(u) > 1.0:
        # 目标在视野外：压到边缘并降亮，网络仍能从"贴边"读出大转向
        amp *= 0.75
        u = float(np.clip(u, -1.0, 1.0))
    # 与 camera.col_angles() 一致：列 0 对应 +FOV/2（车体左侧）
    col = (1.0 - u) * 0.5 * (w - 1)
    xs = np.arange(w, dtype=np.float32)
    out[:] = (np.exp(-0.5 * ((xs - col) / 1.4) ** 2) * amp)[None, :]
    return out


class E2EPolicy:
    name = 'e2e'
    label = 'End-to-end CNN'

    FRAMES = 3
    IN_W = 64
    IN_H = 48
    IN_CH = 5          # 3 帧灰度 + 1 帧深度 + 1 帧目标方位

    def __init__(self, net, max_v=0.42, use_goal=True):
        self.net = net
        self.buf = []
        self.max_v = float(max_v)
        self.use_goal = bool(use_goal)
        # 消融用同一份权重，但报告里必须能区分成两条曲线
        self.name = 'e2e' if self.use_goal else 'e2e_nogoal'
        self.label = 'End-to-end CNN' if self.use_goal else 'End-to-end CNN (goal off)'
        self.t = 0.0

    def reset(self, world):
        self.buf = []
        self.t = 0.0

    def _input(self, rgb, depth=None, odom_pose=None, goal=None):
        g = resize_gray(to_gray(rgb), self.IN_W, self.IN_H)
        g = (g - 0.45) / 0.25
        self.buf.append(g)
        if len(self.buf) > self.FRAMES:
            self.buf.pop(0)
        while len(self.buf) < self.FRAMES:
            self.buf.insert(0, self.buf[0])
        gray = np.stack(self.buf, axis=0)
        if depth is None:
            d = np.zeros((self.IN_H, self.IN_W), dtype=np.float32)
        else:
            d = resize_gray(np.clip(depth / P.CAM_RANGE_MAX, 0.0, 1.0).astype(np.float32),
                            self.IN_W, self.IN_H)
        # 深度是"前方多远有东西"的直接证据，纯灰度堆叠学不出这件事
        gch = goal_channel(odom_pose, goal, self.IN_W, self.IN_H) if self.use_goal \
            else np.zeros((self.IN_H, self.IN_W), dtype=np.float32)
        return np.concatenate([gray, d[None, :, :], gch[None, :, :]], axis=0)

    def act(self, obs):
        self.t += P.DT
        if obs.get('true_pose') is not None and goal_reached(obs['true_pose'], obs['goal']):
            return 0.0, 0.0
        x = self._input(obs['rgb'], obs.get('depth'), obs.get('odom'), obs.get('goal'))
        out = self.net.forward(x[None, ...], train=False)[0]
        v = float(np.clip(out[0], 0.0, 1.0)) * self.max_v
        w = float(np.clip(out[1], -1.0, 1.0)) * P.MAX_W
        return v, w


def make_policy(kind, world, net=None):
    if kind == 'oracle':
        return OraclePolicy(world)
    if kind == 'slam_nav':
        return SlamNavPolicy(world)
    if kind == 'e2e':
        if net is None:
            raise ValueError('e2e 策略需要已训练的 CNN 权重')
        return E2EPolicy(net)
    if kind == 'e2e_nogoal':
        # 消融：同一份权重，把目标方位通道置零，用来证明成败来自目标信息
        if net is None:
            raise ValueError('e2e_nogoal 策略需要已训练的 CNN 权重')
        return E2EPolicy(net, use_goal=False)
    raise ValueError('unknown policy: %s' % kind)
