"""
reloc_core.py —— 高精度定位与重定位共享核心（粒子滤波 / AMCL 风格）
========================================================================
同一套「地图 → 激光扫描 → 粒子滤波重定位 → 里程计航位推算」逻辑被两个平台复用：
  - turtlebot3_amcl.py : Gazebo/TurtleBot3 + ROS2 重定位（本机运行）
  - sim_2d_relocalization.py : 轻量 2D 粒子滤波重定位仿真（可直接跑，生成动图）

重定位问题：机器人知道环境地图，但不知道自己在哪（初始位姿未知）。
  - 在地图上均匀撒 N 个粒子（每个粒子 = 一个候选位姿 (x,y,θ)）
  - 每步按里程计增量推进所有粒子（运动模型，含噪声）
  - 用激光扫描与地图的匹配（likelihood field 权重）更新粒子权重
  - 低方差重采样：权重高的粒子保留/复制，低权重淘汰
  - 粒子收敛到真实位姿 = 重定位完成
同时维护一条纯里程计航位推算轨迹，展示无地图修正时的累积漂移。

只依赖 numpy。
"""

import numpy as np


# ================================================================ 地图
class GridMap:
    """栅格占据地图 + 欧氏距离场。世界坐标系：x 右, y 上。"""

    def __init__(self, world_size, res):
        self.size = float(world_size)
        self.res = float(res)
        self.N = int(round(self.size / res))
        self.occ = np.zeros((self.N, self.N), dtype=bool)   # 占据格
        self.dist_field = None                               # 到最近障碍的距离

    def world_to_idx(self, x, y):
        i = int(np.floor(y / self.res + self.N / 2))
        j = int(np.floor(x / self.res + self.N / 2))
        return i, j

    def idx_to_world(self, i, j):
        return (j - self.N / 2 + 0.5) * self.res, (i - self.N / 2 + 0.5) * self.res

    def in_bounds(self, i, j):
        return 0 <= i < self.N and 0 <= j < self.N

    def add_rect_obstacle(self, x0, y0, x1, y1):
        """添加一个矩形障碍（世界坐标，含边界线）。"""
        i0, j0 = self.world_to_idx(x0, y0)
        i1, j1 = self.world_to_idx(x1, y1)
        i0, i1 = sorted([i0, i1])
        j0, j1 = sorted([j0, j1])
        for i in range(max(i0, 0), min(i1 + 1, self.N)):
            for j in range(max(j0, 0), min(j1 + 1, self.N)):
                self.occ[i, j] = True

    def build_distance_field(self):
        """暴力欧氏距离场：每个空白格到最近障碍格的距离（m）。"""
        occ_idx = np.argwhere(self.occ)
        if len(occ_idx) == 0:
            self.dist_field = np.full((self.N, self.N), 1e6)
            return
        occ_y = occ_idx[:, 0].astype(float)
        occ_x = occ_idx[:, 1].astype(float)
        field = np.full((self.N, self.N), 1e6)
        # 只对空白格计算
        for i in range(self.N):
            for j in range(self.N):
                if not self.occ[i, j]:
                    d = np.sqrt((i - occ_y) ** 2 + (j - occ_x) ** 2).min()
                    field[i, j] = d * self.res
        self.dist_field = field

    def query_dist(self, x, y):
        """查询世界坐标点到最近障碍的距离（m）；超界返回 1e6。"""
        i, j = self.world_to_idx(x, y)
        if not self.in_bounds(i, j):
            return 1e6
        return float(self.dist_field[i, j])

    def is_free(self, x, y):
        i, j = self.world_to_idx(x, y)
        return self.in_bounds(i, j) and not self.occ[i, j]


def default_env():
    """带内墙与孤岛障碍的 12m×12m 环境。"""
    m = GridMap(12.0, 0.2)
    # 外边界
    m.add_rect_obstacle(-6, -6, 6, -5.8)
    m.add_rect_obstacle(-6, 5.8, 6, 6)
    m.add_rect_obstacle(-6, -6, -5.8, 6)
    m.add_rect_obstacle(5.8, -6, 6, 6)
    # 内部墙与孤岛
    m.add_rect_obstacle(-3.5, -5.5, -3.3, 0.5)
    m.add_rect_obstacle(0.0, -5.5, 0.2, -2.0)
    m.add_rect_obstacle(2.2, 0.8, 4.0, 1.0)
    m.add_rect_obstacle(-1.5, 2.5, 1.5, 2.7)     # 孤岛墙
    m.add_rect_obstacle(4.0, -1.5, 4.2, 2.5)
    m.build_distance_field()
    return m


# ================================================================ 激光
def raycast_dist(m, x, y, theta, max_range):
    """从 (x,y) 沿方向 theta 发射射线，返回撞到障碍的距离；超界返回 max_range。"""
    step = m.res * 0.5
    for d in np.arange(step, max_range, step):
        px, py = x + d * np.cos(theta), y + d * np.sin(theta)
        if not m.is_free(px, py):
            return d
    return max_range


def scan_sample(m, pose, angles, max_range, rng=None, noise=0.02):
    """在 pose (x,y,θ) 发一束全向激光，返回各角度上的距离数组。"""
    x, y, th = pose
    d = np.array([raycast_dist(m, x, y, th + a, max_range) for a in angles])
    if rng is not None and noise > 0:
        d = d + rng.normal(0, noise, d.shape)
        d = np.clip(d, 0.05, max_range)
    return d


def likelihood_weights(m, particles, scan, angles, sigma=0.25, max_range=4.0):
    """likelihood field 权重：每个粒子在自身位姿下把激光端点放到世界系，
    在距离场上查『端点离障碍多近』，越近权重越大（观测应落在障碍边缘）。
    只使用『实际命中障碍』的束（scan < max_range），未命中的远距束无信息、
    不计入，避免稀释区分度。"""
    hit = scan < (max_range - 0.01)
    n = len(particles)
    w = np.zeros(n)
    for k, (x, y, th) in enumerate(particles):
        if not hit.any():
            w[k] = 1e-9
            continue
        ep_x = x + scan[hit] * np.cos(th + angles[hit])
        ep_y = y + scan[hit] * np.sin(th + angles[hit])
        d = np.array([m.query_dist(ep_x[i], ep_y[i]) for i in range(hit.sum())])
        valid = d < 1e5
        if not valid.any():
            w[k] = 1e-9
            continue
        w[k] = np.exp(-(d[valid] ** 2).mean() / (2 * sigma ** 2))
    s = w.sum()
    return w / s if s > 0 else np.full(n, 1.0 / n)


# ================================================================ 粒子滤波
class ParticleFilter:
    """AMCL 风格粒子滤波：均匀初始化 → 运动预测 → 似然加权 → 重采样。"""

    def __init__(self, n_particles, rng, bounds=(-6, 6)):
        self.N = int(n_particles)
        self.rng = rng
        self.bounds = bounds
        self.particles = None
        self.weights = None

    def init_uniform(self):
        b = self.bounds
        self.particles = np.column_stack([
            self.rng.uniform(b[0], b[1], self.N),   # x
            self.rng.uniform(b[0], b[1], self.N),   # y
            self.rng.uniform(-np.pi, np.pi, self.N)])  # θ
        self.weights = np.full(self.N, 1.0 / self.N)

    def predict(self, dpos, dtheta, noise_pos=0.01, noise_th=0.012):
        """按里程计增量推进所有粒子：先旋转 dθ，再沿各自新朝向平移 d。
        叠加运动噪声（AMCL 运动模型）。"""
        d = float(np.linalg.norm(dpos))
        self.particles[:, 2] = (self.particles[:, 2] + dtheta
                                + self.rng.normal(0, noise_th, self.N))
        self.particles[:, 2] = np.arctan2(np.sin(self.particles[:, 2]),
                                          np.cos(self.particles[:, 2]))
        th = self.particles[:, 2]
        self.particles[:, 0] += d * np.cos(th) + self.rng.normal(0, noise_pos, self.N)
        self.particles[:, 1] += d * np.sin(th) + self.rng.normal(0, noise_pos, self.N)
        # 保持在地图内
        b = self.bounds
        self.particles[:, 0] = np.clip(self.particles[:, 0], b[0], b[1])
        self.particles[:, 1] = np.clip(self.particles[:, 1], b[0], b[1])

    def update(self, m, scan, angles, sigma=0.25, max_range=4.0):
        self.weights = likelihood_weights(m, self.particles, scan, angles,
                                          sigma, max_range)

    def resample(self):
        """低方差重采样。"""
        w = self.weights
        r = self.rng.uniform(0, 1.0 / self.N)
        c = np.cumsum(w)
        c[-1] = 1.0
        idx = np.zeros(self.N, dtype=int)
        j, w_acc = 0, w[0]
        for i in range(self.N):
            u = r + i / self.N
            while u > c[j]:
                j += 1
                w_acc += w[j]
            idx[i] = j
        self.particles = self.particles[idx].copy()
        self.weights = np.full(self.N, 1.0 / self.N)

    def estimate(self):
        """加权均值位姿（收敛后可靠）。"""
        w = self.weights[:, None]
        x = np.sum(self.particles[:, :2] * w, axis=0)
        th = np.arctan2(np.sum(np.sin(self.particles[:, 2]) * self.weights),
                        np.sum(np.cos(self.particles[:, 2]) * self.weights))
        return np.array([x[0], x[1], th])

    def neff(self):
        return 1.0 / np.sum(self.weights ** 2)
