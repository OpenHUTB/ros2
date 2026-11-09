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
from .nav_guard import INFLATE_CELLS, STOP_TOL, NavGuard
from .planner import inflate, proximity_cost
from .slam import EKFSLAM

SPIN_TIME = 1.6          # 起步原地扫描建图的时间（秒）


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


class OraclePolicy(NavGuard):
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


class SlamNavPolicy(NavGuard):
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
