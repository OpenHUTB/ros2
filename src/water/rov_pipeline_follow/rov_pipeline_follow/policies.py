"""四种策略，共用同一个 act(obs) 接口：

- OraclePolicy  ：真值几何感知 + 同一套 PD 控制，作为性能上界（专家）
- PipeSlamPolicy：CNN 感知 + EKF-SLAM 位姿 + 管道建图 + A* 兜底，完整流水线
- E2EPolicy     ：CNN 从图像直接回归 (u, w)，不起地图、不做规划
- e2e_nogoal    ：E2E 消融，目标方位通道置零

感知与控制是分离的：Oracle 用真值 (dx_err, dtheta)，PipeSlam 用 CNN
回归的同两个量，控制器是同一份 follow_control —— 这样两组实验的
差别就干净地落在"感知"上。
"""

import math

import numpy as np

from . import params as P
from . import detector
from .camera import resize_gray, to_gray
from .controllers import PipeFollower, goal_reached, pure_pursuit, safety_from_depth
from .gridmap import GridMap
from .kinematics import wrap
from .planner import astar, inflate, proximity_cost
from .slam import EKFSLAM

SPIN_TIME = 1.2          # 起步原地扫描建图的时间（秒）
REPLAN_PERIOD = 2.0      # 最小重规划间隔（秒）
LOST_LIMIT = 3.0         # 感知连续丢失超过这个秒数，切换到 A* 地图兜底


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


def mission_errors(world, x, y, th):
    """真值几何 → 任务系的 (dx_err, dtheta)。

    dx_err 必须按巡检方向换算：横向偏差定义在"巡检方向左法向"上，
    返程时艏向反 180°，同一个物理偏差在任务系下符号相反。不换算的
    话控制器会在返程把 ROV 往管道里推。
    """
    s, lat, (tx, ty) = world.progress_of(x, y)
    mdir = world.mission_direction(x, y)
    dx_err = mdir * (P.FOLLOW_OFFSET - lat)
    dtheta = wrap(math.atan2(mdir * ty, mdir * tx) - th)
    return dx_err, dtheta


class OraclePolicy:
    """性能上界：真值位姿 + 真值管道几何，用与流水线相同的控制器。"""

    name = 'oracle'
    label = 'Oracle (ground-truth perception)'

    def __init__(self, world):
        self.world = world
        self.t = 0.0
        self.follower = PipeFollower()
        self._last_phase = 'go'

    def reset(self, world):
        self.world = world
        self.t = 0.0
        self.follower.reset()
        self._last_phase = world.phase

    def act(self, obs):
        self.t += P.DT
        pose = obs['true_pose']
        world = self.world
        if world.done:
            return 0.0, 0.0

        # 阶段推进用真值位姿判定（按弧长进度）
        if world.advance(pose):
            return 0.0, 0.0

        # 折返机动：到达 B 端的瞬间，按"管道在鼻子哪一侧"选背离方向
        if world.phase != self._last_phase:
            self._last_phase = world.phase
            if world.phase == 'return':
                _s, lat, (tx, ty) = world.progress_of(pose[0], pose[1])
                if abs(lat) > 1e-3:
                    # 管道方向 ≈ -sign(lat)·左法向；鼻子在管道左侧就右转，
                    # 在右侧就左转，让残余推力把 ROV 甩向开阔侧
                    sgn = -math.copysign(1.0, lat)
                    rel = wrap(math.atan2(sgn * tx, -sgn * ty) - pose[2])
                else:
                    rel = 0.0
                self.follower.begin_turnaround(1.0 if rel <= 0 else -1.0)

        dx_err, dtheta = mission_errors(world, pose[0], pose[1], pose[2])
        if self.follower.turn_dir:
            return self.follower.step(dx_err, dtheta)
        v, w = self.follower.step(dx_err, dtheta)
        v, w = safety_from_depth(obs['depth'], v, w)
        return v, w


class PipeSlamPolicy:
    """完整流水线：CNN 感知 + EKF-SLAM + 管道建图 + A* 兜底。"""

    name = 'pipe_slam'
    label = 'CNN perception + EKF-SLAM + A*'

    def __init__(self, world, percep_net=None):
        self.world = world
        self.slam = EKFSLAM(world.start)
        self.grid = GridMap()
        self.net = percep_net
        self.follower = PipeFollower()
        self.t = 0.0
        self.lost_t = 0.0
        self.path = []
        self.last_plan = -1e9
        self.n_lm_seen = 0
        self._last_phase = world.phase

    def reset(self, world):
        self.world = world
        self.slam = EKFSLAM(world.start)
        self.grid = GridMap()
        self.follower.reset()
        self.t = 0.0
        self.lost_t = 0.0
        self.path = []
        self.last_plan = -1e9
        self._last_phase = world.phase

    # ---------------------------------------------------------- 感知
    def _perceive(self, obs):
        """返回 (found, dx_err, dtheta)，统一为**任务系**。

        CNN 输出是归一化的 (dx/1.2, dtheta/1.5)，训练标签本来就是任务系，
        直接反归一化；几何检测器给的是车体系横向量，按巡检方向换算 ——
        两条感知路径在这一层之后完全同构。

        """
        if self.net is not None:
            x = _percep_input(obs['rgb'], obs.get('depth'))
            out = self.net.forward(x[None, ...], train=False)[0]
            return True, float(out[0]) * 1.2, float(out[1]) * 1.5
        found, dx_raw, dtheta = detector.pipeline_observation(obs['rgb'],
                                                              obs['depth'])
        if not found:
            return False, 0.0, 0.0
        mdir = self.world.mission_direction(self.slam.pose[0], self.slam.pose[1])
        return True, dx_raw + (mdir - 1.0) * P.FOLLOW_OFFSET, dtheta

    # ---------------------------------------------------------- 阶段
    def _advance_phase(self, pose):
        world = self.world
        if world.advance(pose):
            self.path = []

    def act(self, obs):
        self.t += P.DT
        world = self.world
        if world.done:
            return 0.0, 0.0

        # ---- SLAM：预测 + 更新 + 建图
        dx, dy, dth = obs['odom_delta']
        self.slam.predict(dx, dy, dth)
        self.slam.update(obs['landmarks'])
        self.grid.update(self.slam.pose, obs['pipe_pts'])
        for mx, my, _lid in self.slam.landmark_xy():
            self.grid.mark_obstacle(mx, my)
        self.n_lm_seen = len(self.slam.ids)

        pose = self.slam.pose
        self._advance_phase(pose)

        # 折返机动：任务阶段翻转的瞬间触发。回转方向看最近的管道点在
        # 鼻子哪一侧（车体系方位左正）：在左就右转，在右就左转，
        # 让残余推力把 ROV 甩向背离管道的开阔侧。
        if world.phase != self._last_phase:
            self._last_phase = world.phase
            if world.phase == 'return' and len(obs['pipe_pts']):
                rel = float(obs['pipe_pts'][np.argmin(obs['pipe_pts'][:, 1]), 0])
                self.follower.begin_turnaround(1.0 if rel <= 0 else -1.0)
        if self.follower.turn_dir:
            _s, _l, (t2x, t2y) = world.progress_of(pose[0], pose[1])
            mdir = world.mission_direction(pose[0], pose[1])
            dth_turn = wrap(math.atan2(mdir * t2y, mdir * t2x) - pose[2])
            return self.follower.step(0.0, dth_turn)

        # 起步先原地扫一圈，把周围应答器观测攒起来
        if obs['t'] < SPIN_TIME:
            return 0.10, 0.9

        found, dx_err, dtheta = self._perceive(obs)
        self.lost_t = 0.0 if found else self.lost_t + P.DT

        # 感知正常：直接跟管（dx_err 已是任务系）
        if found and self.lost_t <= LOST_LIMIT:
            v, w = self.follower.step(dx_err, dtheta)
            v, w = safety_from_depth(obs['depth'], v, w)
            return v, w

        # 感知丢失：用 SLAM 位姿在已建图上 A* 到当前目标（应答器已膨胀）
        if self.t - self.last_plan >= REPLAN_PERIOD or not self.path:
            occ = self.grid.occupied(th=0.62)
            blocked = inflate(occ, 4)
            cost = proximity_cost(occ) + np.where(self.grid.explored(), 0.0, 1.2)
            self.path = astar(blocked, (pose[0], pose[1]),
                              (world.goal[0], world.goal[1]), cost=cost)
            self.last_plan = self.t
        if self.path:
            v, w = pure_pursuit(pose, self.path)
        else:
            # 连图都规划不出来：按航向朝目标低速试探
            a = wrap(math.atan2(world.goal[1] - pose[1],
                                world.goal[0] - pose[0]) - pose[2])
            v = 0.20 * max(0.0, math.cos(a))
            w = float(np.clip(1.2 * a, -P.MAX_W, P.MAX_W))
        v, w = safety_from_depth(obs['depth'], v, w)
        return v, w


GRAY_MEAN, GRAY_STD = 0.45, 0.25


def frame_gray(rgb, w=None, h=None):
    """采集端存进数据集的灰度帧：resize(to_gray(rgb))，值域 [0, 1]。"""
    w = E2EPolicy.IN_W if w is None else w
    h = E2EPolicy.IN_H if h is None else h
    return resize_gray(to_gray(rgb), w, h)


def net_gray(g):
    """数据集里的灰度帧 → 网络输入：亮度归一化。

    这层仿射变换必须**训练与部署用同一个函数**。早先部署端漏了它
    （直接喂 [0,1] 原始灰度），而训练张量是 stack_dataset() 里归一化过的，
    端到端策略因此在分布外推理：原地打转，12/12 全部失败。
    """
    return (g - GRAY_MEAN) / GRAY_STD


def depth_feature(depth, w=None, h=None):
    """训练集里 depths 的张成：深度 / 量程，再缩放到网络输入尺寸。"""
    w = E2EPolicy.IN_W if w is None else w
    h = E2EPolicy.IN_H if h is None else h
    if depth is None:
        return np.zeros((h, w), dtype=np.float32)
    return resize_gray(np.clip(depth / P.CAM_RANGE_MAX, 0.0, 1.0).astype(np.float32),
                       w, h)


def _percep_input(rgb, depth=None):
    """感知网络输入：单帧灰度 + 深度，与 stack_percep() 的样本同源。

    注意感知网络这一路**不做**亮度归一化（stack_percep 存的是原始灰度），
    与端到端那一路不同 —— 两条支路的张成各自与自己的训练集对齐。
    """
    g = frame_gray(rgb)
    d = depth_feature(depth)
    return np.concatenate([g[None, :, :], d[None, :, :]], axis=0)


PERCEP_IN_CH = 2


def goal_channel(odom_pose, goal, w=None, h=None):
    """把"目标在车体系下的方位"画成一条竖直亮带（HUD 式目标指示）。

    为什么必须有这一路：往返巡检里同一幅管道画面既可能对应"前进"也可能
    对应"折返"，正确动作取决于当前目标是 B 端还是 A 端，而端点完全不在
    图像里。纯图像策略在这种环境下不是学不好，而是**不可辨识** —— 行为
    克隆只能学到两种情形的均值，然后在原地打转。方位由里程计位姿与
    当前任务目标算出，不依赖地图、不依赖 SLAM。

    亮带位置 = 方位角在视野内的归一化横坐标；亮度 = 1 - d/世界对角线。
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
    u = beta / half if half > 0 else 0.0
    span = math.hypot(P.WORLD_W, P.WORLD_H)
    amp = float(1.0 - 0.6 * min(1.0, dist / span))
    if abs(u) > 1.0:
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

    def __init__(self, net, max_v=0.45, use_goal=True):
        self.net = net
        self.buf = []
        self.max_v = float(max_v)
        self.use_goal = bool(use_goal)
        self.name = 'e2e' if self.use_goal else 'e2e_nogoal'
        self.label = 'End-to-end CNN' if self.use_goal else 'End-to-end CNN (goal off)'
        self.t = 0.0

    def reset(self, world):
        self.buf = []
        self.t = 0.0

    def _input(self, rgb, depth=None, odom_pose=None, goal=None):
        # 与 stack_dataset() 的张成逐位一致：原始灰度再过 net_gray 归一化
        g = net_gray(frame_gray(rgb, self.IN_W, self.IN_H))
        self.buf.append(g)
        if len(self.buf) > self.FRAMES:
            self.buf.pop(0)
        while len(self.buf) < self.FRAMES:
            self.buf.insert(0, self.buf[0])
        gray = np.stack(self.buf, axis=0)
        d = depth_feature(depth, self.IN_W, self.IN_H)
        gch = goal_channel(odom_pose, goal, self.IN_W, self.IN_H) if self.use_goal \
            else np.zeros((self.IN_H, self.IN_W), dtype=np.float32)
        return np.concatenate([gray, d[None, :, :], gch[None, :, :]], axis=0)

    def act(self, obs):
        self.t += P.DT
        x = self._input(obs['rgb'], obs.get('depth'), obs.get('odom'), obs.get('goal'))
        out = self.net.forward(x[None, ...], train=False)[0]
        v = float(np.clip(out[0], 0.0, 1.0)) * self.max_v
        w = float(np.clip(out[1], -1.0, 1.0)) * P.MAX_W
        return v, w


def make_policy(kind, world, net=None, percep_net=None):
    if kind == 'oracle':
        return OraclePolicy(world)
    if kind == 'pipe_slam':
        return PipeSlamPolicy(world, percep_net=None)
    if kind == 'pipe_cnn':
        if percep_net is None:
            raise ValueError('pipe_cnn 策略需要已训练的感知 CNN 权重')
        pol = PipeSlamPolicy(world, percep_net=percep_net)
        # 两条支路共用 PipeSlamPolicy，但必须能分别统计：类属性是
        # 'pipe_slam'，实例上覆盖掉，否则评测 CSV 会把两支混成一行
        pol.name = 'pipe_cnn'
        pol.label = 'CNN perception + EKF-SLAM'
        return pol
    if kind in ('e2e', 'e2e_nogoal'):
        if net is None:
            raise ValueError('%s 策略需要已训练的 CNN 权重' % kind)
        return E2EPolicy(net, use_goal=(kind == 'e2e'))
    raise ValueError('unknown policy: %s' % kind)
