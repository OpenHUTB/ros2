"""
platoon_core.py —— 多车协同编队共享核心
=============================================
同一套编队控制律被两个平台复用：
  - carla_platoon.py   : CARLA 真机/仿真渲染版（本地装 CARLA 后运行）
  - sim_2d_platoon.py  : 轻量 2D matplotlib 仿真版（可直接跑，生成效果动图）

编队模型（一字纵队 platoon，沿共享参考轨迹等弧长间隔）：
  - 一条参数化参考轨迹 path，按累计弧长 s 采样成密集点列。
  - Leader 沿轨迹以设定车速匀速推进：s_leader(t) = v_set * t。
  - 第 i 辆 follower 的期望进度：s_i = s_leader - i * spacing（即沿轨迹
    相对 leader 滞后 i 个车距），期望参考点 = 轨迹上弧长 s_i 处的点。
  - 每辆车都用「纯跟踪(pure pursuit)」横向追自己的参考点，
    用「进度误差 P 控制」纵向调节车速，使实际进度逼近期望进度。
  - 效果：所有车沿同一条轨迹运动、彼此保持恒定弧长间距，转弯不散队。

只依赖 numpy，可在任意 Python 环境复用。
"""

import numpy as np


# ---------------------------------------------------------------- 工具
def normalize_angle(a):
    """把角度归一到 (-pi, pi]。"""
    return (a + np.pi) % (2 * np.pi) - np.pi


def build_path_from_waypoints(wpts, dense_per_m=2.0):
    """把一组稀疏 waypoint (N,2) 线性插值成密集路径，并计算累计弧长。
    返回: path (M,2), s (M,) 每个点的累计弧长。"""
    wpts = np.asarray(wpts, dtype=float)
    # 生成每段之间的密集采样
    segs = []
    for i in range(len(wpts) - 1):
        a, b = wpts[i], wpts[i + 1]
        d = np.linalg.norm(b - a)
        n = max(int(np.ceil(d * dense_per_m)), 2)
        segs.append(np.linspace(a, b, n, endpoint=False))
    segs.append(wpts[-1:])
    path = np.vstack(segs)
    dif = np.diff(path, axis=0)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(dif, axis=1))])
    return path, s


def sample_path(path, s, s_query):
    """在路径上按弧长 s_query 取点（越界则夹取到端点）。"""
    s_query = float(np.clip(s_query, s[0], s[-1]))
    idx = int(np.searchsorted(s, s_query))
    idx = min(max(idx, 1), len(path) - 1)
    # 线性插值到精确弧长位置
    s0, s1 = s[idx - 1], s[idx]
    if s1 - s0 < 1e-9:
        return path[idx].copy()
    t = (s_query - s0) / (s1 - s0)
    return path[idx - 1] + t * (path[idx] - path[idx - 1])


def project_to_path(path, s, point, hint_s=None, window=60):
    """把世界坐标 point 投影到折线路径，返回 (最近点弧长 s_p, 最近点坐标)。
    用于把车辆实际位置折算成路径进度，供纵向速度控制使用。
    hint_s: 上一次的投影弧长（可选）。给定时从 hint_s 附近窗口搜索，
    避免编队多车、每帧高频调用时全量遍历整条路径，显著提速；
    未给定时退化为全路径搜索（结果一致）。"""
    p = np.asarray(point, dtype=float)
    dif = np.diff(path, axis=0)
    n_seg = len(dif)
    # 由 hint_s 确定搜索的折线段区间 [i0, i1)
    if hint_s is None:
        i0, i1 = 0, n_seg
    else:
        idx = int(np.clip(np.searchsorted(s, hint_s), 0, len(s) - 1))
        i0, i1 = max(0, idx - 1 - window), min(n_seg, idx + window)
    best_s, best_d2 = s[0], 1e18
    best_pt = path[0].copy()
    for i in range(i0, i1):
        v = dif[i]
        w = p - path[i]
        L2 = np.dot(v, v)
        if L2 < 1e-12:
            proj = 0.0
        else:
            proj = np.clip(np.dot(w, v) / L2, 0.0, 1.0)
        pt = path[i] + proj * v
        d2 = np.sum((pt - p) ** 2)
        if d2 < best_d2:
            best_d2 = d2
            best_s = s[i] + proj * (s[i + 1] - s[i])
            best_pt = pt
    return best_s, best_pt


# ---------------------------------------------------------------- 运动学
class Bicycle:
    """阿克曼/自行车运动学模型（低速路径跟踪用）。
    state: x, y, yaw;  控制: 车速 v(m/s), 前轮转角 delta(rad)。"""

    def __init__(self, x, y, yaw, L=2.8):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.L = float(L)          # 轴距(m)

    def state(self):
        return np.array([self.x, self.y, self.yaw])

    def step(self, v, delta, dt):
        self.x += v * np.cos(self.yaw) * dt
        self.y += v * np.sin(self.yaw) * dt
        self.yaw += (v / self.L) * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)


def pure_pursuit_steer(x, y, yaw, target, lookahead, L, max_steer=0.6):
    """纯跟踪横向控制，返回前轮转角 delta(rad)。
    target: 世界坐标下的目标点 (2,)。"""
    dx = target[0] - x
    dy = target[1] - y
    # 转到车身坐标系
    cos_y, sin_y = np.cos(-yaw), np.sin(-yaw)
    ld_x = cos_y * dx - sin_y * dy
    ld_y = sin_y * dx + cos_y * dy
    if abs(ld_x) < 1e-6 and abs(ld_y) < 1e-6:
        return 0.0
    alpha = np.arctan2(ld_y, ld_x)          # 车身系中目标方位角
    steer = np.arctan2(2.0 * L * np.sin(alpha), lookahead)
    return float(np.clip(steer, -max_steer, max_steer))


# ---------------------------------------------------------------- 编队
def platoon_initial_progress(n, spacing):
    """编队初始基准进度：leader 置于路径起点前方 (n-1)*spacing 处，
    follower 依次沿路径滞后 spacing，起步位姿沿轨迹排开、互不重合。"""
    return float((n - 1) * spacing)


def leader_target_s(s_leader, lookahead):
    """leader 目标点弧长 = 当前进度 + 前视距离（保证前视向量非退化、能转向）。"""
    return s_leader + lookahead


def follower_target_s(s_leader, i, spacing):
    """第 i 辆 follower 目标点弧长 = 编队基准进度 - i*spacing（>=0）。"""
    return max(s_leader - i * spacing, 0.0)


class Platoon:
    """一字纵队编队控制。
    参数:
      path, s        : 参考轨迹（build_path_from_waypoints 输出）
      n_vehicles     : 车辆总数（含 leader），>=2
      spacing        : 期望车距（沿轨迹弧长，m）
      v_set          : 编队巡航车速 (m/s)
      lookahead      : 纯跟踪前视距离 (m)
      L              : 轴距 (m)
      kp             : 纵向进度 P 增益（调车速）
      max_steer      : 最大前轮转角 (rad)
      v_min, v_max   : 车速上下限 (m/s)
    """

    def __init__(self, path, s, n_vehicles=4, spacing=8.0, v_set=6.0,
                 lookahead=6.0, L=2.8, kp=1.2, max_steer=0.6,
                 v_min=0.0, v_max=10.0):
        self.path = path
        self.s = s
        self.n = int(n_vehicles)
        self.spacing = float(spacing)
        self.v_set = float(v_set)
        self.lookahead = float(lookahead)
        self.L = float(L)
        self.kp = float(kp)
        self.max_steer = float(max_steer)
        self.v_min, self.v_max = float(v_min), float(v_max)

        # 初始：leader 在路径起点前方 (n-1)*spacing，follower 依次滞后 spacing
        # → 起步位姿沿轨迹排开、互不重合（避免四车初始重叠）
        self.s_leader = platoon_initial_progress(self.n, self.spacing)
        yaw0 = np.arctan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
        self.cars = []
        for i in range(self.n):
            s_i = self.s_leader - i * self.spacing   # >= 0
            px, py = sample_path(path, s, s_i)
            self.cars.append(Bicycle(px, py, yaw0, L=L))

    def initial_states(self):
        """返回各车初始位姿 (N,3)，供测试与渲染版对照。"""
        return np.array([c.state() for c in self.cars])

    def step(self, dt):
        """推进一帧。返回 (states(N,3), targets(N,2), speeds(N,))。
        编队基准 = leader 实际投影进度（反馈，非固定累加虚拟值）；
        leader 目标 = 当前进度 + 前视（前视向量非退化，转弯时能转向）。"""
        # 各车实际进度（投影到路径）
        s_actual = np.zeros(self.n)
        for i, car in enumerate(self.cars):
            s_actual[i], _ = project_to_path(self.path, self.s, [car.x, car.y])
        s_leader = float(s_actual[0])
        self.s_leader = s_leader                  # 编队基准 = leader 实际进度

        targets = np.zeros((self.n, 2))
        speeds = np.zeros(self.n)
        states = np.zeros((self.n, 3))
        for i, car in enumerate(self.cars):
            if i == 0:
                # leader：目标 = 当前进度 + 前视（有前视向量，转弯时能转向）
                s_tgt = leader_target_s(s_leader, self.lookahead)
                v = self.v_set                     # leader 定速领航
            else:
                # follower：目标 = 基准进度 - i*spacing，纵向 P 控间距
                s_tgt = follower_target_s(s_leader, i, self.spacing)
                err = s_tgt - s_actual[i]
                v = self.v_set + self.kp * err
                v = float(np.clip(v, self.v_min, self.v_max))

            tgt = sample_path(self.path, self.s, s_tgt)
            targets[i] = tgt
            # 横向：纯跟踪追参考点
            delta = pure_pursuit_steer(
                car.x, car.y, car.yaw, tgt, self.lookahead, car.L, self.max_steer)
            car.step(v, delta, dt)
            states[i] = car.state()
            speeds[i] = v
        return states, targets, speeds
