#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""作业三 · CARLA 建图 + 导航（导航与 SLAM 同步仿真，神经网络版）。

对应老师任务 (3)：在虚拟环境中实现对机器人的**建图和导航**，导航与 SLAM 的同步仿真。
满足"**规划算法须为神经网络**"的硬性要求：

  * **建图**：车辆边行驶，边把激光雷达命中点投影到 2D 占用栅格地图，
    体现"边动边建图"的 SLAM 理念（含贝叶斯对数几率更新）。
  * **规划 NN**：`nn_models.MLPPolicy`，由「目标方位 + 障碍左右分布 + 最近距离」
    输出（前进量, 转向），驱车避障驶向目标。

三种模式：

  * `--mode train`   离线训练规划神经网络（仅需 numpy，**无需 CARLA**）
  * `--mode run`     连 CARLA：LiDAR 建图 + NN 规划导航
  * `--headless --demo`  离线自检取证：建图 + NN 导航全在合成环境中完成，
                         导出栅格地图、损失曲线、路径图（**无需 CARLA 与图形界面**）

用法：
    python3 main.py --mode train --epochs 300 --out models/nn_plan.json
    python3 main.py --mode run --model models/nn_plan.json --goal "20,8" --sim_time 30
    python3 main.py --headless --demo --save_dir ~/shots
    ros2 launch carla_mapping_navigation main.launch.py host:=<宿主机IP>
    roslaunch carla_mapping_navigation main.launch host:=<宿主机IP>
"""

import argparse
import math
import os
import sys

import numpy as np

from carla_mapping_navigation.gif_writer import write_gif
from carla_mapping_navigation.nn_models import MLPPolicy

try:
    from carla_mapping_navigation import carla_common as cc
    _HAVE_CARLA = getattr(cc, "carla", None) is not None
except Exception:  # noqa: BLE001
    cc = None
    _HAVE_CARLA = False

DT = 0.05            # 同步步进固定步长（秒）
GRID_M = 0.5         # 每格边长（米）
GRID_N = 200         # 栅格边长（格）→ 覆盖 100 m × 100 m
WHEELBASE = 2.5      # 轴距（m）
STEER_GAIN = 1.2217  # steer(归一化) → 前轮转角(rad)，CARLA 中 steer=1 约对应 70°
MAX_SPEED = 6.0      # 稳态速度（m/s）
GOAL_TOL = 1.5       # 到达判定（m）

# 栅格对数几率更新参数（贝叶斯占用栅格）
LOG_ODDS_HIT = 0.85   # 命中：更倾向于"被占据"
LOG_ODDS_MISS = -0.4  # 穿过（自由）：更倾向于"空闲"
LOG_ODDS_MIN, LOG_ODDS_MAX = -3.0, 3.5

# 起点与目标（世界坐标，单位 m）
START = (36.0, -5.0)
DEFAULT_GOAL = (20.0, 8.0)


# ============================================================ 栅格地图 / 建图
class OccupancyGrid:
    """2D 占用栅格地图（对数几率表示，贝叶斯更新）。

    栅格以 **start 点为中心**建立，而不是固定以世界原点为中心——这是必须的：
    车辆出生点可能距世界原点几十米（本模块默认在 (36,-5)），若栅格固定以原点
    为中心且只覆盖 ±30 m，车辆会落在栅格之外，所有雷达命中点都会被丢弃，
    建图结果恒为空。

    对数几率表达：$l = \\log\\frac{p}{1-p}$，命中时 $l \\mathrel{+}= l_{hit}$，
    穿过时 $l \\mathrel{+}= l_{miss}$，概率由 $p = \\sigma(l)$ 还原。
    """

    def __init__(self, center=START, n=GRID_N, m_per_cell=GRID_M):
        self.n = n
        self.m = m_per_cell
        self.center = (float(center[0]), float(center[1]))
        # 初始对数几率 0 → 概率 0.5（未知）
        self.log_odds = np.zeros((n, n), dtype=np.float32)

    # ---------------- 坐标变换 ----------------
    def world_to_grid(self, x, y):
        """世界坐标 → 栅格索引 (row, col)。返回浮点索引，便于判界。"""
        c = (np.asarray(x, dtype=np.float64) - self.center[0]) / self.m + self.n / 2.0
        r = (np.asarray(y, dtype=np.float64) - self.center[1]) / self.m + self.n / 2.0
        return r, c

    def grid_to_world(self, r, c):
        """栅格索引 → 世界坐标（格中心）。"""
        x = (np.asarray(c, dtype=np.float64) - self.n / 2.0) * self.m + self.center[0]
        y = (np.asarray(r, dtype=np.float64) - self.n / 2.0) * self.m + self.center[1]
        return x, y

    def in_bounds(self, r, c):
        return (r >= 0) & (r < self.n) & (c >= 0) & (c < self.n)

    # ---------------- 更新 ----------------
    def update_hits(self, world_x, world_y, hit=True):
        """把一批世界坐标点写入栅格（hit=True 记为命中，False 记为穿过）。"""
        r, c = self.world_to_grid(world_x, world_y)
        valid = self.in_bounds(r, c)
        r = r[valid].astype(int)
        c = c[valid].astype(int)
        if r.size == 0:
            return 0
        self.log_odds[r, c] += LOG_ODDS_HIT if hit else LOG_ODDS_MISS
        np.clip(self.log_odds, LOG_ODDS_MIN, LOG_ODDS_MAX, out=self.log_odds)
        return int(r.size)

    def probability(self):
        """对数几率 → 占用概率。"""
        return 1.0 / (1.0 + np.exp(-self.log_odds))

    def occupied_mask(self, thresh=0.6):
        return self.probability() > thresh

    def free_mask(self, thresh=0.4):
        return self.probability() < thresh

    @property
    def known_ratio(self):
        """已知格子比例（概率明显偏离 0.5 的格子占比）。"""
        p = self.probability()
        return float(np.mean((p > 0.55) | (p < 0.45)))

    @property
    def occupied_count(self):
        return int(self.occupied_mask().sum())


def lidar_to_world(points, x, y, yaw):
    """雷达点（传感器系 x 前 y 左）→ 世界坐标。

    $$
    \\begin{bmatrix} p_x^w \\\\ p_y^w \\end{bmatrix}
    = \\mathbf R(\\psi)\\begin{bmatrix} p_x^s \\\\ p_y^s \\end{bmatrix}
    + \\begin{bmatrix} x_v \\\\ y_v \\end{bmatrix},\\quad
    \\mathbf R(\\psi)=\\begin{bmatrix}\\cos\\psi & -\\sin\\psi \\\\
    \\sin\\psi & \\cos\\psi\\end{bmatrix}
    $$
    """
    cos, sin = math.cos(yaw), math.sin(yaw)
    px = points[:, 0] * cos - points[:, 1] * sin
    py = points[:, 0] * sin + points[:, 1] * cos
    return px + x, py + y


def build_map_from_scan(grid, pc, x, y, yaw, max_range=35.0, ray_step=1.0):
    """用一帧雷达数据更新占用栅格：命中点标记占据，射线沿途标记空闲。

    除命中点外还把**射线沿途**的格子标记为空闲（`LOG_ODDS_MISS`），
    这是占用栅格建图的标准做法——只标命中点无法区分"没扫到"与"扫到是空的"。

    返回 (命中数, 空闲数)。
    """
    if pc is None or len(pc) == 0:
        return 0, 0
    pts = pc[(pc[:, 0] > 0.3) & (pc[:, 0] < max_range)]
    if len(pts) == 0:
        return 0, 0

    # ---- 1) 命中点 → 占据 ----
    wx, wy = lidar_to_world(pts, x, y, yaw)
    n_hit = grid.update_hits(wx, wy, hit=True)

    # ---- 2) 射线沿途 → 空闲（按距离采样，避免对每个点都插值）----
    step = max(1, int(len(pts) // 200))  # 抽样，控制计算量
    miss_x, miss_y = [], []
    for i in range(0, len(pts), step):
        rng = pts[i, 0]
        if rng <= ray_step:
            continue
        for t in np.arange(ray_step, rng, ray_step):
            frac = t / rng
            miss_x.append(x + (wx[i] - x) * frac)
            miss_y.append(y + (wy[i] - y) * frac)
    n_miss = 0
    if miss_x:
        n_miss = grid.update_hits(np.asarray(miss_x), np.asarray(miss_y), hit=False)
    return n_hit, n_miss


def obs_features(pc, goal_ang_diff, max_range=8.0):
    """把「目标方位 + 前方障碍分布」压成规划 NN 的 5 维状态向量。

    返回 (state, raw)。**所有维度都归一化到相近尺度**——早期版本把"最近距离"
    以原始米数（0.3~8.0）直接放进状态，与其余维（0~1）量纲差 8 倍，
    梯度被该维主导，网络学不好。

    第 5 维是**最近障碍的方向角** \\(\\theta_{obs}\\)（传感器系，左为正）。
    早期版本这一维误写成了第 4 维的副本，等于只有 4 维有效输入——网络分不清
    "障碍在正前方"和"障碍在侧前方"，而这两种情况该往哪边打方向恰好相反。

    $$
    \\mathbf s = \\Big[\\frac{e_\\psi}{\\pi},\\ \\frac{N_L}{N_{max}},\\
    \\frac{N_R}{N_{max}},\\ \\frac{D_{\\min}}{D_{max}},\\
    \\frac{\\theta_{obs}}{\\pi/2}\\Big]^\\top
    $$
    """
    base = [goal_ang_diff / math.pi, 0.0, 0.0, 1.0, 0.0]
    if pc is None or len(pc) == 0:
        return np.array(base, dtype=np.float32), (goal_ang_diff, 0, 0, max_range, 0.0)

    x, y = pc[:, 0], pc[:, 1]
    near = (x > 0.3) & (x < max_range) & (np.abs(y) < 4.0)
    pts = pc[near]
    if len(pts) == 0:
        return np.array(base, dtype=np.float32), (goal_ang_diff, 0, 0, max_range, 0.0)

    left_cnt = int((pts[:, 1] > 0).sum())
    right_cnt = int((pts[:, 1] < 0).sum())
    k = int(np.argmin(pts[:, 0]))                          # 最近的那个障碍点
    nearest = float(pts[k, 0])
    bearing = float(math.atan2(pts[k, 1], pts[k, 0]))      # 左为正，范围 ±π/2
    dist_n = float(np.clip(nearest / max_range, 0.0, 1.0))
    state = np.array([goal_ang_diff / math.pi,
                      min(left_cnt / 50.0, 1.0),
                      min(right_cnt / 50.0, 1.0),
                      dist_n,
                      bearing / (math.pi / 2.0)], dtype=np.float32)
    return state, (goal_ang_diff, left_cnt, right_cnt, nearest, bearing)


# ============================================================ 规划神经网络
def planning_label(diff, obs_l, obs_r, nearest, max_range=8.0, bearing=0.0):
    """规划网络的监督标签：由"朝目标 + 避障"解析生成 (throttle, steer)。

    转向由三项叠加：

    * **朝目标**：$\\tanh(1.2\\,e_\\psi)$；
    * **左右障碍密度差**：$0.6(\\rho_R-\\rho_L)$，左侧障碍多则右转；
    * **最近障碍的横向排斥**：$-0.5\\,(1-\\tilde D_{\\min})\\sin\\theta_{obs}$，
      障碍偏在左侧（$\\theta_{obs}>0$）时产生向右的排斥，且越近越强。

    前两项符号相反且对称，第三项在正前方（$\\theta_{obs}=0$）时为 0，
    因此"正对目标且无障碍 → 转向为 0"，与物理直觉一致。

    前进：障碍越近越减速，映射到 $[0,1]$。

    注：$bearing$ 放在 $max_range$ **之后**，这样早期的
    ``planning_label(d, l, r, nearest, max_range)`` 位置调用不受影响。
    """
    d_n = min(max(nearest / max_range, 0.0), 1.0)
    avoid = -0.5 * (1.0 - d_n) * math.sin(bearing)
    steer = math.tanh(1.2 * diff) + 0.6 * (obs_r - obs_l) + avoid
    steer = float(np.clip(steer, -1.0, 1.0))
    # 最近距离归一化后映射到油门：>=0.75（约6m）全速，越近越低
    throttle = float(np.clip(0.25 + 0.75 * d_n, 0.0, 1.0))
    return throttle, steer


def synth_dataset(n=600, seed=0, max_range=8.0):
    """合成规划训练数据。返回 (X, Y)。

    X 为 5 维归一化状态，Y 为 2 维 (throttle, steer)。

    **油门标签 ∈[0,1] 必须用线性输出层**：`MLPPolicy` 默认 tanh 输出 ∈[-1,1]，
    用它去拟合 [0,1] 的油门只能用到一半值域，且负值区域永远学不到。
    因此这里训练时对油门单独做 [0,1]→[-1,1] 归一化，推理时再还原。
    """
    rng = np.random.RandomState(seed)
    diff = rng.uniform(-math.pi, math.pi, n)
    obs_l = rng.uniform(0, 1, n)
    obs_r = rng.uniform(0, 1, n)
    nearest = rng.uniform(0.3, max_range, n)
    bearing = rng.uniform(-math.pi / 2.0, math.pi / 2.0, n)   # 最近障碍方向角

    d_n = np.clip(nearest / max_range, 0.0, 1.0)
    X = np.stack([diff / math.pi, obs_l, obs_r, d_n,
                  bearing / (math.pi / 2.0)], axis=1).astype(np.float32)

    Y = np.zeros((n, 2), dtype=np.float32)
    for i in range(n):
        th, st = planning_label(diff[i], obs_l[i], obs_r[i], nearest[i],
                                max_range, bearing=bearing[i])
        Y[i, 0] = th
        Y[i, 1] = st
    return X, Y


def train_planning(epochs=300, out="models/nn_plan.json", verbose=None):
    """训练规划神经网络，返回 (net, history)。"""
    if verbose is None:
        verbose = max(1, epochs // 5)
    X, Y = synth_dataset()
    net = MLPPolicy([5, 64, 2], seed=0)
    print("== 训练规划 NN：5 维状态(目标方位+障碍分布) → (前进量, 转向) ==")
    hist = net.train(X, Y, epochs=epochs, lr=0.15, verbose=verbose)
    pred = net.predict(X)
    mse = float(np.mean((pred - Y) ** 2))
    print(f"规划 NN 训练 MSE = {mse:.5f}")
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    net.save(out)
    print("模型已保存:", out)
    return net, {"loss": hist["loss"], "mse": mse}


# ============================================================ 车辆运动学（回放用）
def step_bicycle(x, y, yaw, v, throttle, steer, dt=DT, max_speed=MAX_SPEED):
    """自行车模型一步更新（教学用简化模型，与几何参数统一）。"""
    v += (throttle * max_speed * 1.2 - v) * 0.1
    v = max(0.0, min(v, max_speed))
    yaw += v * math.tan(steer * STEER_GAIN) / WHEELBASE * dt
    x += v * math.cos(yaw) * dt
    y += v * math.sin(yaw) * dt
    return x, y, yaw, v


# ============================================================ PNG 导出（取证）
def _write_png(path, rgb):
    """纯 zlib 写 PNG（不依赖 Pillow/opencv）。rgb 为 (H,W,3) uint8。"""
    import struct
    import zlib
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    h, w = rgb.shape[:2]
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 6))
    png += chunk(b"IEND", b"")
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "wb") as f:
        f.write(png)
    return path


def render_grid_png(grid, path, path_pts=None, goal=None, scale=3):
    """把占用栅格渲染成 PNG：占据=深色，空闲=浅色，未知=中灰。

    栅格是 n×n 的（默认 200×200），直接画太小，按 scale 放大后再写盘。
    """
    p = grid.probability()
    img = np.full((grid.n, grid.n, 3), 128, dtype=np.uint8)      # 未知：中灰
    free = p < 0.4
    occ = p > 0.6
    img[free] = (235, 235, 235)                                   # 空闲：浅灰
    img[occ] = (30, 30, 30)                                       # 占据：深色

    # 车辆轨迹（红）、目标（绿）
    if path_pts:
        for (px, py) in path_pts:
            r, c = grid.world_to_grid(px, py)
            if grid.in_bounds(int(r), int(c)):
                img[int(r), int(c)] = (230, 40, 40)
    if goal is not None:
        r, c = grid.world_to_grid(goal[0], goal[1])
        for dr in range(-2, 3):
            for dc in range(-2, 3):
                rr, cc_ = int(r) + dr, int(c) + dc
                if grid.in_bounds(rr, cc_):
                    img[rr, cc_] = (40, 200, 60)

    # 图像行方向：栅格 r 增大对应世界 y 增大；PNG 的行从上到下，翻转一下更直观
    img = img[::-1]
    if scale > 1:
        img = np.repeat(np.repeat(img, scale, axis=0), scale, axis=1)
    return _write_png(path, img)


def render_prob_png(prob, grid, path, path_pts=None, goal=None, scale=3, title_t=None):
    """用"某一时刻的概率图副本"渲染快照，用于展示**渐进建图**过程。

    与 `render_grid_png` 的区别：这里接收的是概率矩阵**副本**而非栅格对象本身，
    因此可以还原出"地图在 t 秒时只有这么大"的中间状态。
    左上角画一条进度条表示该快照对应的仿真时刻，便于对比。
    """
    return _write_png(path, prob_canvas(prob, grid, path_pts, goal, scale, title_t))


# 快照渲染用的固定调色板。GIF 只支持索引色，而快照本身的颜色是固定的 6 种，
# 所以直接把 RGB 映射成下标，既能出 PNG 也能出 GIF。
MAP_PALETTE = [(128, 128, 128),   # 未知区域
               (235, 235, 235),   # 空闲区域
               (30, 30, 30),      # 占据区域
               (230, 40, 40),     # 车辆轨迹
               (40, 200, 60),     # 目标点
               (60, 120, 230)]    # 时刻进度条


def prob_canvas(prob, grid, path_pts=None, goal=None, scale=3, title_t=None):
    """把概率图副本画成 (H,W,3) uint8 的 RGB 数组（不写盘）。

    拆出来是为了让离线演示能逐帧收集画面、最后合成 GIF。
    """
    p = np.asarray(prob)
    img = np.full((grid.n, grid.n, 3), 128, dtype=np.uint8)
    img[p < 0.4] = (235, 235, 235)
    img[p > 0.6] = (30, 30, 30)
    if path_pts:
        for (px, py) in path_pts:
            r, c = grid.world_to_grid(px, py)
            if grid.in_bounds(int(r), int(c)):
                img[int(r), int(c)] = (230, 40, 40)
    if goal is not None:
        r, c = grid.world_to_grid(goal[0], goal[1])
        for dr in range(-2, 3):
            for dc in range(-2, 3):
                rr, cc_ = int(r) + dr, int(c) + dc
                if grid.in_bounds(rr, cc_):
                    img[rr, cc_] = (40, 200, 60)
    # 顶部画时刻进度条：长度正比于 title_t（满程按 30 s 计）
    if title_t is not None:
        frac = min(max(float(title_t) / 30.0, 0.0), 1.0)
        bar = max(1, int(frac * (grid.n - 4)))
        img[2:5, 2:2 + bar] = (60, 120, 230)
    img = img[::-1]
    if scale > 1:
        img = np.repeat(np.repeat(img, scale, axis=0), scale, axis=1)
    return img


def rgb_to_indices(img, palette=MAP_PALETTE):
    """把固定调色板的 RGB 图转成 GIF 需要的索引图（调色板外的颜色记 0）。"""
    out = np.zeros(img.shape[:2], dtype=np.uint8)
    for i, (r, g, b) in enumerate(palette):
        out[(img[:, :, 0] == r) & (img[:, :, 1] == g) & (img[:, :, 2] == b)] = i
    return out


def render_train_loss_png(losses, path, width=640, height=360):
    """导出训练损失曲线 PNG。"""
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    canvas[10:height - 10, 10:12] = 200
    canvas[10:height - 10, width - 12:width - 10] = 200
    canvas[10:12, 10:width - 10] = 200
    canvas[height - 12:height - 10, 10:width - 10] = 200
    v = np.asarray(losses, dtype=np.float32).ravel()
    if v.size < 2:
        return _write_png(path, canvas)
    vmin, vmax = float(v.min()), float(v.max())
    if vmax - vmin < 1e-9:
        vmax = vmin + 1.0
    xs = np.linspace(20, width - 20, v.size).astype(int)
    ys = (height - 20 - (v - vmin) / (vmax - vmin) * (height - 40)).astype(int)
    for i in range(v.size - 1):
        _line(canvas, xs[i], ys[i], xs[i + 1], ys[i + 1], (200, 60, 60))
    return _write_png(path, canvas)


def _line(canvas, x0, y0, x1, y1, color):
    """Bresenham 画线。"""
    h, w = canvas.shape[:2]
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        if 0 <= x0 < w and 0 <= y0 < h:
            canvas[y0, x0] = color
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


# ======================================================== 离线取证（无需 CARLA）
def _fake_scan(x, y, yaw, goal, obstacles, max_range=35.0, n_rays=180, seed=0):
    """合成一帧雷达数据：向四周发射射线，命中障碍物或地图边界即停。

    obstacles 为 [(x, y, radius), ...]。返回 (N,4) 点云（传感器系 x 前 y 左）。
    仅用于离线取证模式，让建图 + 规划链路在没有 CARLA 时也能完整跑通。
    """
    rng = np.random.RandomState(seed)
    pts = []
    for i in range(n_rays):
        ang_local = -math.pi + 2 * math.pi * i / n_rays
        # 传感器系 → 世界系
        ang_world = yaw + ang_local
        dx, dy = math.cos(ang_world), math.sin(ang_world)
        r = max_range
        for (ox, oy, orad) in obstacles:
            # 射线与圆求交
            fx, fy = x - ox, y - oy
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - orad * orad
            disc = b * b - c
            if disc >= 0:
                sq = math.sqrt(disc)
                t = -b - sq
                if t <= 0.3:
                    # 传感器已经在圆内（近交点落在自己身后），改用远交点。
                    # 否则"车已经开进障碍物"这种最该报出来的情况反而会被完全
                    # 漏掉，规划网络也就看不到这个障碍。
                    t = -b + sq
                if 0.3 < t < r:
                    r = t
        # 地图边界（±45 m）
        for lim, comp in ((45.0, 0), (-45.0, 0), (45.0, 1), (-45.0, 1)):
            d = dx if comp == 0 else dy
            if abs(d) > 1e-6:
                t = ((lim - x) if comp == 0 else (lim - y)) / d
                if 0.3 < t < r:
                    r = t
        hit_x = x + dx * r
        hit_y = y + dy * r
        # 世界系 → 传感器系（逆旋转）
        cos, sin = math.cos(yaw), math.sin(yaw)
        sx = (hit_x - x) * cos + (hit_y - y) * sin
        sy = -(hit_x - x) * sin + (hit_y - y) * cos
        pts.append((sx, sy, 0.0, 1.0))
    return np.asarray(pts, dtype=np.float32)


def run_offline_demo(save_dir, epochs=200, sim_time=40.0, goal=None, seed=0):
    """离线自检：合成环境中完成「边动边建图 + 规划 NN 导航」，导出地图与曲线。

    **不需要 CARLA、不需要图形界面**，用于在无 3D 加速的虚拟机/CI 中产出证据。
    """
    goal = goal or DEFAULT_GOAL
    print("=" * 64)
    print("  作业三 离线自检取证模式（合成环境，不需要 CARLA）")
    print("=" * 64)

    net, hist = train_planning(epochs=epochs, out=os.path.join(save_dir or ".", "nn_plan.json"))
    print(f"规划 NN 训练 MSE = {hist['mse']:.5f}")

    # ---- 障碍物布局：按「沿路径 s 米、横向偏移 d 米」摆放 ----
    # 早期布局把 (26,2) 放在「起点→目标」直线上（离直线只有 0.06 m），车必然
    # 撞上去；而当时 _fake_scan 又漏掉了"传感器在圆内"的情况，连撞上都看不出来。
    # 现在每个障碍的横向偏移都大于其半径，保证留出可通行的缝。
    ux, uy = goal[0] - START[0], goal[1] - START[1]
    _len = math.hypot(ux, uy)
    ux, uy = ux / _len, uy / _len       # 沿路径方向的单位向量
    nx, ny = -uy, ux                    # 路径左法向

    def _place(s, lateral, radius):
        """路径 s 米处、向左偏 lateral 米（负值向右）、半径 radius 的圆柱障碍。"""
        return (START[0] + ux * s + nx * lateral,
                START[1] + uy * s + ny * lateral, radius)

    obstacles = [_place(10.0, +2.8, 1.6),   # 车需向右让（间隙 1.2 m）
                 _place(16.0, -3.4, 2.0),   # 车需向左让（间隙 1.4 m）
                 _place(19.5, +5.0, 2.5)]   # 目标附近建筑，主要贡献地图细节

    grid = OccupancyGrid(center=START)
    # 初始航向由「起点→目标」方向推导，不能写死为 0（+x）：
    # 默认目标 (20,8) 位于起点 (36,-5) 的**左后方**（方位角约 +141°），
    # 若车头朝 +x，自车会先背离目标行驶再掉头，既浪费里程又让
    # "距目标最近距离"等指标被这段无效机动污染。
    yaw = math.atan2(goal[1] - START[1], goal[0] - START[0])
    x, y = START[0], START[1]
    v = 0.0
    path_pts = [(x, y)]
    snaps = []           # 渐进建图快照 [(tick, 概率图副本, 当时轨迹)]
    min_d = float("inf")
    min_clear = float("inf")   # 与障碍物表面的最近距离（负值 = 撞上）
    reached_at = None
    n_ticks = int(sim_time / DT)

    # 动图：每 0.25 s 采一帧（DT=0.05 → 每 5 拍），最后合成 GIF。
    # 栅格本身 200×200，放大 2 倍到 400×400；快照只有 6 种颜色，
    # 经 LZW 压缩后整个动图通常只有几百 KB，远低于 10 MB 的要求。
    gif_every = max(1, int(0.25 / DT))
    gif_scale = 2
    gif_frames = []

    for k in range(n_ticks):
        # ---- 建图：本帧雷达 → 占用栅格 ----
        pc = _fake_scan(x, y, yaw, goal, obstacles, seed=k)
        n_hit, n_miss = build_map_from_scan(grid, pc, x, y, yaw)

        # ---- 目标判定 ----
        dxg, dyg = goal[0] - x, goal[1] - y
        d = math.hypot(dxg, dyg)
        min_d = min(min_d, d)
        if d < GOAL_TOL:
            reached_at = k
            print(f"\n[到达] t={k * DT:.1f}s 抵达目标 ({goal[0]},{goal[1]})，停车。")
            break

        # ---- 规划 NN ----
        diff = (math.atan2(dyg, dxg) - yaw + math.pi) % (2 * math.pi) - math.pi
        state, raw = obs_features(pc, diff)
        out = net.predict(state[None, :])[0]
        throttle = float(np.clip(out[0], 0.0, 1.0))
        steer = float(np.clip(out[1], -1.0, 1.0))

        x, y, yaw, v = step_bicycle(x, y, yaw, v, throttle, steer)
        path_pts.append((x, y))
        # 与障碍物表面的最近距离：把"避障"从说法变成可核对的数字（负值即撞上）
        for (ox, oy, orad) in obstacles:
            min_clear = min(min_clear, math.hypot(x - ox, y - oy) - orad)

        if k % 50 == 0:      # 每 2.5 s 打一行。原来按 5 s 打，而本演示约 4.5 s 就到达目标，
                             # 会只剩 t=0 一行，看不出行驶过程
            _g, lc, rc, nearest, bearing = raw
            print(f"[t={k * DT:5.1f}s] pos=({x:6.1f},{y:6.1f}) 距目标={d:5.2f}m "
                  f"NN(油门={throttle:.2f}, 转向={steer:+.2f}) "
                  f"障碍(L{lc}/R{rc}, {nearest:.1f}m, 方位{math.degrees(bearing):+.0f}°) "
                  f"占据格={grid.occupied_count}")

        # ---- 渐进建图快照：体现"边动边建图"，地图随行驶逐步成形 ----
        # 每 1.0 s 采一帧。原来按 3.0 s 采，而本演示路线约 6 s 就到达目标，
        # 只攒到 2 帧，下面的 len(snaps) >= 3 判定不过，3 张进度图根本导不出来。
        if save_dir and k % max(1, int(1.0 / DT)) == 0:
            snaps.append((k, grid.probability().copy(), list(path_pts)))

        # 动图帧：与上面 3 张进度图同源，但采得更密，合成后能看出地图一格格长出来
        if save_dir and k % gif_every == 0:
            gif_frames.append(rgb_to_indices(prob_canvas(
                grid.probability().copy(), grid, path_pts=list(path_pts),
                goal=goal, scale=gif_scale, title_t=k * DT)))

    os.makedirs(save_dir, exist_ok=True)
    p1 = render_grid_png(grid, os.path.join(save_dir, "occupancy_map.png"),
                         path_pts=path_pts, goal=goal)
    p2 = render_train_loss_png(hist["loss"], os.path.join(save_dir, "plan_loss.png"))
    # 动图：把逐帧采集的建图快照合成 GIF（文档里要求放 gif 演示）
    p_gif = None
    if len(gif_frames) >= 2:
        p_gif = write_gif(os.path.join(save_dir, "map_progress.gif"),
                          gif_frames, MAP_PALETTE, delay_cs=8)
    # 渐进建图序列图（3 张，展示地图从空白到成形）
    prog_paths = []
    if len(snaps) >= 3:
        idxs = [0, len(snaps) // 2, len(snaps) - 1]
        for n, i in enumerate(idxs):
            kk, prob, pts = snaps[i]
            prog_paths.append(render_prob_png(
                prob, grid, os.path.join(save_dir, f"map_progress_{n + 1}.png"),
                path_pts=pts, goal=goal,
                title_t=k * 0 + kk * DT))

    def _snap_known(prob):
        return float(np.mean((prob > 0.55) | (prob < 0.45)))

    if prog_paths:
        seq = " → ".join(f"{_snap_known(snaps[i][1]) * 100:.1f}%" for i in idxs)
        print(f"\n渐进建图已知区域占比：{seq}（最终 {grid.known_ratio * 100:.1f}%）")

    print("\n---- 建图结果 ----")
    print(f"占据格数        = {grid.occupied_count}")
    print(f"空闲格数        = {int(grid.free_mask().sum())}")
    print(f"已知区域比例    = {grid.known_ratio * 100:.1f}%")
    print(f"栅格分辨率      = {GRID_M} m/格，覆盖 {GRID_N * GRID_M:.0f} m × {GRID_N * GRID_M:.0f} m")
    print(f"栅格中心（起点）= {grid.center}")
    print(f"车辆最终位置    = ({x:.2f}, {y:.2f})，轨迹点数 {len(path_pts)}")
    print(f"距目标最近距离  = {min_d:.3f} m（判定阈值 {GOAL_TOL} m）")
    print(f"与障碍最近距离  = {min_clear:.2f} m"
          f"{'（未发生碰撞）' if min_clear > 0 else '★发生碰撞'}")
    if reached_at is not None:
        print(f"到达耗时        = {reached_at * DT:.1f} s")
    print("\n已导出取证图：")
    for p in (p1, p2) + tuple(prog_paths) + ((p_gif,) if p_gif else ()):
        print("   ", p)
    if p_gif:
        print(f"    动图 {len(gif_frames)} 帧，"
              f"{os.path.getsize(p_gif) / 1024:.0f} KB")
    return 0


# ================================================================ 在线运行
def run_carla(host, port, town, model_path, goal, sim_time=30.0, save_dir=None):
    """连 CARLA：LiDAR 建图 + 规划 NN 导航。

    流程：先短程直行**建图**（SLAM 边动边建图），再进入 **NN 规划导航**阶段。
    """
    if not _HAVE_CARLA:
        print("[错误] 缺少 carla 模块，run 模式需要 CARLA 服务端。")
        print("       离线验证可改用： python3 main.py --headless --demo --save_dir ~/shots")
        return 1

    # 模型文件不存在时**现场训练并保存**，而不是直接崩掉。
    # 与 ROS 节点（mapping_navigation_node.py）的行为保持一致：
    # 那边在模型缺失时同样回退到现场训练，保证"节点总能运行"。
    if not os.path.isfile(model_path):
        print(f"[提示] 未找到模型文件 {model_path}，先现场训练（纯 numpy，约 1 分钟）...")
        print("       也可单独训练： python3 main.py --mode train --out " + model_path)
        # train_planning 内部会 synth_dataset + 训练 + save(out)
        train_planning(epochs=300, out=model_path)

    net = MLPPolicy.load(model_path)

    try:
        client, world = cc.connect(host, port, town)
    except Exception as exc:  # noqa: BLE001
        print(f"\n[错误] 连接 CARLA 服务端失败：{exc}")
        print(f"       目标 {host}:{port}，地图 {town}。按下面顺序排查：")
        print("       1) 宿主机 CARLA 服务端是否已启动（Windows 上运行 CarlaUE4-Win64-Shipping.exe）")
        print("       2) 宿主机防火墙是否放行 2000 端口；host 是否填宿主机 VMnet8 地址")
        print("       3) 只想验证算法可用离线模式（不需要 CARLA）：")
        print("          python3 main.py --headless --demo --save_dir ~/shots")
        return 1

    # spawn / 建图 / 导航 / 清理全部包进 try-finally：
    # 万一中途抛异常，也要保证销毁传感器与自车、并把世界恢复为异步模式，
    # 不给下一个使用者留下残留 actor 和卡住的世界。
    vehicle = None
    sensors = []
    try:
        vehicle, tf = cc.spawn_vehicle(world)

        grid = OccupancyGrid(center=(tf.location.x, tf.location.y))
        holder = {"pc": None}
        # 必须保留 sensor 对象的引用！否则 spawn_actor 返回的 sensor 无引用，
        # 函数作用域结束后被 Python 垃圾回收，Actor 虽留在仿真里但回调不再触发，
        # 表现为"共收到雷达帧 0 张"、栅格恒为空，并伴随 CARLA 警告：
        #   sensor object went out of the scope but the sensor is still alive in the simulation
        sensors.append(cc.make_lidar(
            world, vehicle, lambda pc: holder.__setitem__(
                "pc", np.frombuffer(pc.raw_data, dtype=np.float32).reshape(-1, 4)),
            tick=True))
        print(f"[就绪] 自车@({tf.location.x:.1f},{tf.location.y:.1f})，"
              f"目标=({goal[0]},{goal[1]})，规划用 NN")
        print(f"[栅格] 分辨率 {GRID_M} m/格，"
              f"覆盖 {GRID_N * GRID_M:.0f}m × {GRID_N * GRID_M:.0f}m，中心=自车起点")

        # ---- 建图阶段 ----
        print("== 建图阶段（SLAM 边动边建图）==")
        cc.apply_control(vehicle, throttle=0.4, steer=0.0)
        for _ in range(int(4.0 / DT)):
            pc = holder["pc"]
            if pc is not None:
                pos = cc.get_location(vehicle)
                build_map_from_scan(grid, pc, pos[0], pos[1], cc.get_yaw(vehicle))
            world.tick()
        print(f"  建图完成：占据格={grid.occupied_count}，已知区域={grid.known_ratio * 100:.1f}%")

        # ---- NN 导航阶段 ----
        print("== 导航阶段（神经网络规划）==")
        min_d = float("inf")
        path_pts = []
        shot = 0
        steps = int(sim_time / DT)
        for k in range(steps):
            pos = cc.get_location(vehicle)
            yaw = cc.get_yaw(vehicle)
            path_pts.append(pos)
            dx, dy = goal[0] - pos[0], goal[1] - pos[1]
            d = math.hypot(dx, dy)
            min_d = min(min_d, d)

            pc = holder["pc"]
            if pc is not None:
                build_map_from_scan(grid, pc, pos[0], pos[1], yaw)

            if d < GOAL_TOL:
                cc.apply_control(vehicle, throttle=0.0, steer=0.0, brake=0.8)
                world.tick()
                print(f"\n[到达] t={k * DT:.1f}s 抵达目标，停车。")
                break

            goal_ang = math.atan2(dy, dx)
            diff = (goal_ang - yaw + math.pi) % (2 * math.pi) - math.pi
            state, raw = obs_features(pc, diff)
            out = net.predict(state[None, :])[0]
            throttle = float(np.clip(out[0], 0.0, 1.0))
            steer = float(np.clip(out[1], -1.0, 1.0))
            cc.apply_control(vehicle, throttle=throttle, steer=steer)
            world.tick()

            if save_dir and k % 60 == 0:
                shot += 1
                render_grid_png(grid, os.path.join(save_dir, f"map_step_{shot:02d}.png"),
                                path_pts=path_pts, goal=goal)

            if k % 40 == 0:
                _g, lc, rc, nearest = raw
                print(f"[t={k * DT:.1f}] d={d:.1f} NN(th={throttle:.2f},st={steer:+.2f}) "
                      f"obs(L{lc}/R{rc}, {nearest:.1f}m) 占据格={grid.occupied_count}")

        print(f"\n导航完成，距目标最近距离: {min_d:.3f} m")
        print(f"建图结果：占据格={grid.occupied_count}，已知区域={grid.known_ratio * 100:.1f}%")
        if save_dir:
            p = render_grid_png(grid, os.path.join(save_dir, "occupancy_map_final.png"),
                                path_pts=path_pts, goal=goal)
            print("栅格地图已导出:", p)
    finally:
        # 停掉并销毁传感器：必须显式 destroy，否则 actor 会留在世界里
        for s in sensors:
            try:
                s.stop()
                s.destroy()
            except Exception:  # noqa: BLE001
                pass
        # 销毁自车
        if vehicle is not None:
            try:
                vehicle.destroy()
            except Exception:  # noqa: BLE001
                pass
        # 恢复异步模式：同步模式下服务端只在客户端 tick 时推进，
        # 脚本退出后没人 tick，CARLA 窗口会看起来「停住不动」。
        if cc.restore_async(world):
            print('[清理] 已销毁传感器与自车，世界已恢复异步模式。')
        else:
            print('[清理] 已销毁传感器与自车；世界仍为同步模式，'
                  '如需恢复可重启 CARLA 服务端。')
    return 0


# ==================================================================== CLI
def parse_goal(text, default=DEFAULT_GOAL):
    """把 ``"x,y"`` 形式的目标解析成 ``(x, y)``。

    命令行 ``--goal``、ROS 1 的 ``main.launch`` 与 ROS 2 的 ``main.launch.py``
    都用 ``"20,8"`` 这种写法传目标，因此统一在这里解析。格式非法时**回退到
    *default* 并打印告警**，而不是抛异常——launch 启动时因为一个笔误就整个
    起不来，排查成本比用默认目标高得多。
    """
    if text is None:
        return default
    parts = [p.strip() for p in str(text).strip().strip("[]()").split(",")]
    if len(parts) != 2 or not all(parts):
        print(f"[警告] 目标 {text!r} 不是 \"x,y\" 格式，改用默认 {default}",
              file=sys.stderr)
        return default
    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        print(f"[警告] 目标 {text!r} 含非数字，改用默认 {default}", file=sys.stderr)
        return default


def build_parser():
    p = argparse.ArgumentParser(description="CARLA 建图 + 导航（神经网络规划版）")
    p.add_argument("--mode", choices=["train", "run"], default="run")
    p.add_argument("--host", default="127.0.0.1", help="CARLA 服务端地址（虚拟机填宿主机 IP）")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--town", default="Town05")
    p.add_argument("--goal", default="20,8", help="导航目标 \"x,y\"")
    p.add_argument("--sim_time", type=float, default=30.0)
    p.add_argument("--epochs", type=int, default=300)
    p.add_argument("--out", default="models/nn_plan.json")
    p.add_argument("--model", default="models/nn_plan.json")
    _add_bool(p, "--headless",
              "无窗口/无 CARLA 的离线取证模式")
    _add_bool(p, "--demo",
              "运行内置演示序列")
    p.add_argument("--save_dir", nargs="?", const=None, default=None, help="取证图导出目录")
    _add_bool(p, "--launch",
              "由 ROS launch 启动（等价 run）")
    return p


def _parse_bool(value):
    """Parse a boolean that arrives as text.

    ROS 1 launch files can only pass arguments as plain text (e.g.
    ``--follow true`` / ``--follow false``) -- unlike ROS 2, which passes a real
    parameter and keeps the bool type.  Without this, ``--follow false`` makes
    ``store_true`` set follow=True and the stray ``false`` is swallowed by
    ``parse_known_args`` as an unknown positional.
    """
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on", "y", "t"):
        return True
    if text in ("0", "false", "no", "off", "n", "f", ""):
        return False
    raise argparse.ArgumentTypeError(f"无法识别的布尔值：{value!r}")


def _add_bool(parser, name, help_text):
    """Boolean switch: bare ``--flag`` or explicit ``--flag true|false``."""
    parser.add_argument(name, type=_parse_bool, nargs="?", const=True,
                        default=False, help=help_text)


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.headless or args.demo:
        return run_offline_demo(args.save_dir or "shots", epochs=min(args.epochs, 200),
                                sim_time=args.sim_time, goal=parse_goal(args.goal))

    gx, gy = parse_goal(args.goal)
    if args.mode == "train":
        train_planning(args.epochs, args.out)
        return 0
    return run_carla(args.host, args.port, args.town, args.model, (gx, gy),
                     args.sim_time, save_dir=args.save_dir)


if __name__ == "__main__":
    sys.exit(main())
