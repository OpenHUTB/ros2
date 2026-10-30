#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""屋顶场景合成器：生成「理想几何 + 机载 LiDAR 退化点云 + 落点标注」。

为什么要有这个文件
------------------
学习型落点选择网络需要**带真值标注**的数据。真实仿真器里拿不到「完美落点」的标注，
所以我们按 `rooftop_landing` 里已经验证过的几何定义来造：

    真值落点 = 屋顶（扣除障碍物后）区域的最大内切圆圆心
             = 到屋顶边界距离最远的点

这与 ``rooftop_perception.polylabel`` 求解的目标**完全一致**，只不过这里是在
「理想高清几何」上算，因此它就是 polylabel 想逼近的那个点；同时点云则按真实
LiDAR 的退化方式生成（稀疏化、离群点、噪声、立面回波），于是可以定量回答：

    同样的点云退化条件下，「学出来的落点」与「几何 polylabel 落点」谁更稳？

真值计算不依赖 scipy：用两遍 chamfer 距离变换 + 局部加权质心精化（0.2 m 栅格）。

本文件只依赖 numpy，可以独立运行、也可以被训练/评估脚本导入。
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "scripts"))

import heightmap as hm_mod  # noqa: E402

#: 理想几何栅格边长（m）：越小真值越准，但 chamfer 越慢
IDEAL_STEP = 0.2
#: 默认 ROI 边长（m），与 config / launch / 模型 meta 保持一致
ROI_SIZE_M = 16.0
#: 屋顶高度范围（ENU，m）
ROOF_Z_RANGE = (4.0, 20.0)
#: 点云噪声
NOISE_RANGE = (0.005, 0.05)


class RoofScene(object):
    """一个场景的理想几何 + 真值落点（与点云退化无关）。

    关于真值的两种读法（很重要）：

    * ``clearance(x, y)``：点 (x, y) 到屋面边界的距离（m），来自理想几何上的距离变换。
      这是**不依赖具体选点**的物理量，评估时用它最公平 —— 细长矩形的
      「离边界最远的点」是一整条中轴，任何中轴上的点都同样安全，
      拿"与某个特定点的距离"当指标会把正确解判成错误解。
    * ``gt_xy``：训练用的**规范落点** —— 最大净空连通域的中心（若中心落在空洞里，
      就投影到该连通域内离中心最近的点）。给出唯一、可复现的回归目标。
    """

    __slots__ = ("rects", "obstacles", "roof_z", "center", "size_m",
                 "gt_xy", "gt_radius", "shape_name", "ideal_occ", "ideal_step",
                 "clear_dist")

    def __init__(self, rects, obstacles, roof_z, center, size_m, shape_name):
        self.rects = list(rects)
        self.obstacles = list(obstacles)
        self.roof_z = float(roof_z)
        self.center = (float(center[0]), float(center[1]))
        self.size_m = float(size_m)
        self.shape_name = shape_name
        self.ideal_step = IDEAL_STEP
        self.ideal_occ = self._build_ideal_occupancy()
        self.clear_dist = self._chamfer(self.ideal_occ)
        self.gt_xy, self.gt_radius = self._solve_gt()

    # ---------------- 理想几何 ----------------
    def _grid_axes(self):
        m = int(round(self.size_m / self.ideal_step))
        half = 0.5 * self.size_m
        xs = self.center[0] - half + (np.arange(m, dtype=np.float64) + 0.5) * self.ideal_step
        ys = self.center[1] - half + (np.arange(m, dtype=np.float64) + 0.5) * self.ideal_step
        return xs, ys, m

    def _build_ideal_occupancy(self):
        """屋顶区域（可降落）= 各矩形并集 - 障碍物，布尔栅格 (m, m)，row 沿 +y。"""
        xs, ys, m = self._grid_axes()
        gx = xs.reshape(1, -1)
        gy = ys.reshape(-1, 1)
        occ = np.zeros((m, m), dtype=bool)
        for (x0, y0, x1, y1) in self.rects:
            occ |= (gx >= x0) & (gx <= x1) & (gy >= y0) & (gy <= y1)
        for (x0, y0, x1, y1) in self.obstacles:
            occ &= ~((gx >= x0) & (gx <= x1) & (gy >= y0) & (gy <= y1))
        return occ

    def _chamfer(self, occ):
        """两遍 chamfer（3-4 近似欧氏距离，误差 ~2%，远小于 0.2 m 栅格）。

        返回「每个栅格到最近的**非**屋面栅格的距离」，非屋面处为 0。
        """
        m = occ.shape[0]
        inf = 1e9
        dist = np.where(occ, inf, 0.0).astype(np.float64)
        step = float(self.ideal_step)
        d1 = step
        d2 = math.sqrt(2.0) * step
        for i in range(m):
            row = dist[i]
            for j in range(m):
                if row[j] == 0.0:
                    continue
                best = row[j]
                if j > 0:
                    best = min(best, row[j - 1] + d1)
                if i > 0:
                    best = min(best, dist[i - 1][j] + d1)
                    if j > 0:
                        best = min(best, dist[i - 1][j - 1] + d2)
                    if j + 1 < m:
                        best = min(best, dist[i - 1][j + 1] + d2)
                row[j] = best
        for i in range(m - 1, -1, -1):
            row = dist[i]
            for j in range(m - 1, -1, -1):
                if row[j] == 0.0:
                    continue
                best = row[j]
                if j + 1 < m:
                    best = min(best, row[j + 1] + d1)
                if i + 1 < m:
                    best = min(best, dist[i + 1][j] + d1)
                    if j + 1 < m:
                        best = min(best, dist[i + 1][j + 1] + d2)
                    if j > 0:
                        best = min(best, dist[i + 1][j - 1] + d2)
                row[j] = best
        return dist

    def _near_max_component(self, rel=0.97):
        """最大净空附近（>= rel * max）的栅格集合中，含峰值的那个连通域（8 邻域）。"""
        occ = self.ideal_occ
        finite = np.where(occ, self.clear_dist, -1.0)
        peak = float(finite.max())
        if peak <= 0.0:
            return np.zeros_like(occ), peak
        hot = occ & (finite >= rel * peak)
        # BFS 从峰值格出发，只走 hot 的 8 邻域
        flat = int(np.argmax(finite))
        m = occ.shape[0]
        start = (flat // m, flat % m)
        seen = np.zeros_like(occ)
        stack = [start]
        seen[start] = True
        while stack:
            i, j = stack.pop()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    ni, nj = i + di, j + dj
                    if 0 <= ni < m and 0 <= nj < m and hot[ni, nj] and not seen[ni, nj]:
                        seen[ni, nj] = True
                        stack.append((ni, nj))
        return seen, peak

    def _solve_gt(self):
        """规范落点 = 最大净空连通域的中心（中心落在空洞/边界外时投影到域内最近点）。"""
        comp, radius = self._near_max_component()
        ii, jj = np.nonzero(comp)
        if ii.size == 0:
            return self.center, 0.0
        i_f = float(ii.mean())
        j_f = float(jj.mean())
        # 质心可能落在洞/域外（环形屋顶）：投影到域内离质心最近的格
        d2 = (ii - i_f) ** 2 + (jj - j_f) ** 2
        k = int(np.argmin(d2))
        return self._cell_center(float(ii[k]), float(jj[k])), float(radius)

    def clearance(self, x, y):
        """点 (x, y) 处的净空（到屋面边界的距离，m）：不在屋内返回 0。"""
        half = 0.5 * self.size_m
        step = self.ideal_step
        m = self.ideal_occ.shape[0]
        fcol = (float(x) - (self.center[0] - half)) / step - 0.5
        frow = (float(y) - (self.center[1] - half)) / step - 0.5
        c0 = int(math.floor(fcol))
        r0 = int(math.floor(frow))
        if not (0 <= r0 < m - 1 and 0 <= c0 < m - 1):
            return 0.0
        dc = fcol - c0
        dr = frow - r0
        d = self.clear_dist
        v = ((1 - dr) * ((1 - dc) * d[r0, c0] + dc * d[r0, c0 + 1]) +
             dr * ((1 - dc) * d[r0 + 1, c0] + dc * d[r0 + 1, c0 + 1]))
        return float(v)

    def clearance_grid(self, hm):
        """把理想净空距离场最近邻采样到高度图栅格上（用于构造训练标签）。"""
        cells = int(hm["cells"])
        half = 0.5 * hm["size_m"]
        xs = self.center[0] - half + (np.arange(cells, dtype=np.float64) + 0.5) * hm["cell"]
        ys = self.center[1] - half + (np.arange(cells, dtype=np.float64) + 0.5) * hm["cell"]
        m = self.ideal_occ.shape[0]
        step = self.ideal_step
        fcol = (xs - (self.center[0] - half)) / step - 0.5
        frow = (ys - (self.center[1] - half)) / step - 0.5
        c = np.clip(np.rint(fcol).astype(np.int64), 0, m - 1)
        r = np.clip(np.rint(frow).astype(np.int64), 0, m - 1)
        return self.clear_dist[np.ix_(r, c)]      # (row=y, col=x)

    def _cell_center(self, i_f, j_f):
        half = 0.5 * self.size_m
        x = self.center[0] - half + (float(j_f) + 0.5) * self.ideal_step
        y = self.center[1] - half + (float(i_f) + 0.5) * self.ideal_step
        return (float(x), float(y))

    def in_roof(self, x, y, margin=0.0):
        """点是否落在屋顶「可降落区域」内（可要求离边界至少 margin 米）。"""
        for (x0, y0, x1, y1) in self.rects:
            if (x0 - margin) <= x <= (x1 + margin) and (y0 - margin) <= y <= (y1 + margin):
                break
        else:
            return False
        for (x0, y0, x1, y1) in self.obstacles:
            if (x0 - margin) <= x <= (x1 + margin) and (y0 - margin) <= y <= (y1 + margin):
                return False
        return True


# --------------------------------------------------------------------------
# 场景采样
# --------------------------------------------------------------------------
def _rand_rect(rng, w_range, h_range):
    w = rng.uniform(*w_range)
    h = rng.uniform(*h_range)
    return (-0.5 * w, -0.5 * h, 0.5 * w, 0.5 * h)


def make_scene(rng, size_m=16.0, shape=None):
    """随机生成一个屋顶场景（默认中心在原点，调用方再平移到全局坐标）。"""
    roof_z = float(rng.uniform(*ROOF_Z_RANGE))
    shapes = ["rect", "L", "U", "courtyard", "twin", "rect_obstacle"]
    name = shape if shape in shapes else shapes[int(rng.integers(0, len(shapes)))]
    rects = []
    obstacles = []

    if name == "rect":
        rects.append(_rand_rect(rng, (4.0, 11.0), (4.0, 11.0)))
    elif name == "L":
        w = rng.uniform(7.0, 11.0)
        h = rng.uniform(7.0, 11.0)
        cut_w = rng.uniform(2.5, w - 3.0)
        cut_h = rng.uniform(2.5, h - 3.0)
        rects.append((-0.5 * w, -0.5 * h, 0.5 * w, 0.5 * h))
        obstacles.append((0.5 * w - cut_w, 0.5 * h - cut_h, 0.5 * w + 0.1, 0.5 * h + 0.1))
    elif name == "U":
        w = rng.uniform(8.0, 11.5)
        h = rng.uniform(8.0, 11.5)
        notch_w = rng.uniform(2.5, w - 4.0)
        notch_h = rng.uniform(3.0, h - 3.5)
        rects.append((-0.5 * w, -0.5 * h, 0.5 * w, 0.5 * h))
        obstacles.append((-0.5 * notch_w, 0.5 * h - notch_h, 0.5 * notch_w, 0.5 * h + 0.1))
    elif name == "courtyard":
        w = rng.uniform(9.0, 12.0)
        h = rng.uniform(9.0, 12.0)
        hole_w = rng.uniform(2.0, 0.45 * w)
        hole_h = rng.uniform(2.0, 0.45 * h)
        rects.append((-0.5 * w, -0.5 * h, 0.5 * w, 0.5 * h))
        obstacles.append((-0.5 * hole_w, -0.5 * hole_h, 0.5 * hole_w, 0.5 * hole_h))
    elif name == "twin":
        # 两栋相邻但不相连的楼：总跨度限制在 ROI 内（否则会被 ROI 裁掉，真值落到边界外）
        w1 = rng.uniform(4.0, 6.5)
        h1 = rng.uniform(5.0, 9.0)
        gap = rng.uniform(0.6, 2.4)
        w2 = rng.uniform(3.0, min(w1, 6.0))
        h2 = rng.uniform(3.0, h1)
        total = w1 + gap + w2
        if total > 12.0:                     # 等比缩到 12 m 以内
            k = 12.0 / total
            w1, gap, w2 = w1 * k, gap * k, w2 * k
        rects.append((-0.5 * w1, -0.5 * h1, 0.5 * w1, 0.5 * h1))
        rects.append((0.5 * w1 + gap, -0.5 * h2, 0.5 * w1 + gap + w2, 0.5 * h2))
    else:  # rect_obstacle：屋顶上放 1~3 个空调外机 / 天窗
        rects.append(_rand_rect(rng, (7.0, 11.0), (7.0, 11.0)))
        x0, y0, x1, y1 = rects[0]
        for _ in range(int(rng.integers(1, 4))):
            ow = rng.uniform(0.8, 2.0)
            oh = rng.uniform(0.8, 2.0)
            ox = rng.uniform(x0 + 0.4, x1 - ow - 0.4)
            oy = rng.uniform(y0 + 0.4, y1 - oh - 0.4)
            obstacles.append((ox, oy, ox + ow, oy + oh))

    # 屋顶整体平移（保证在 ROI 内且留出周边地面），ROI 中心再加一点抖动
    all_x = [r[0] for r in rects] + [r[2] for r in rects]
    all_y = [r[1] for r in rects] + [r[3] for r in rects]
    roof_c = (0.5 * (min(all_x) + max(all_x)), 0.5 * (min(all_y) + max(all_y)))
    jitter = rng.uniform(-2.0, 2.0, size=2)
    center = (roof_c[0] + jitter[0], roof_c[1] + jitter[1])
    shift = (-roof_c[0] + center[0], -roof_c[1] + center[1])
    rects = [(x0 + shift[0], y0 + shift[1], x1 + shift[0], y1 + shift[1]) for (x0, y0, x1, y1) in rects]
    obstacles = [(x0 + shift[0], y0 + shift[1], x1 + shift[0], y1 + shift[1])
                 for (x0, y0, x1, y1) in obstacles]
    return RoofScene(rects, obstacles, roof_z, center, size_m, name)


def sample_cloud(scene, rng, density=1.0, outlier_ratio=0.0, noise_sigma=0.02,
                 roof_step=0.22, ground_step=0.9, wall_step=0.5):
    """按给定的退化参数生成点云（ENU，z 向上为正）。

    * ``density``       屋顶点保留比例（0~1），模拟点云稀疏 / 远处回波少
    * ``outlier_ratio`` ROI 内均匀离群点比例（模拟杂波 / 反光）
    * ``noise_sigma``   每条点的测距噪声标准差 (m)
    * 地面：z=0 的规则网格（参考面），立面：屋顶四周的竖直回波
    """
    size_m = scene.size_m
    half = 0.5 * size_m
    cx, cy = scene.center
    pts = []

    for (x0, y0, x1, y1) in scene.rects:
        xs = np.arange(x0, x1 + 1e-9, roof_step)
        ys = np.arange(y0, y1 + 1e-9, roof_step)
        gx, gy = np.meshgrid(xs, ys)
        roof = np.stack([gx.ravel(), gy.ravel(),
                         np.full(gx.size, scene.roof_z)], axis=1)
        keep = rng.random(roof.shape[0]) <= float(density)
        pts.append(roof[keep])

    # 屋顶上的障碍物（空调外机等）：顶面比屋面高 0.6~1.4 m
    for (x0, y0, x1, y1) in scene.obstacles:
        ox0, oy0 = max(x0, cx - half), max(y0, cy - half)
        ox1, oy1 = min(x1, cx + half), min(y1, cy + half)
        if ox1 <= ox0 or oy1 <= oy0:
            continue
        xs = np.arange(ox0, ox1 + 1e-9, roof_step)
        ys = np.arange(oy0, oy1 + 1e-9, roof_step)
        gx, gy = np.meshgrid(xs, ys)
        top = np.stack([gx.ravel(), gy.ravel(),
                        np.full(gx.size, scene.roof_z + 1.0)], axis=1)
        keep = rng.random(top.shape[0]) <= float(density)
        pts.append(top[keep])

    # 地面（参考面）
    xs = np.arange(cx - half, cx + half + 1e-9, ground_step)
    ys = np.arange(cy - half, cy + half + 1e-9, ground_step)
    gx, gy = np.meshgrid(xs, ys)
    ground = np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], axis=1)
    pts.append(ground)

    # 立面（屋顶边缘的竖直回波，高度方向稀疏采样）：每条边一次性构造，避免上千次 vstack
    zs = np.arange(0.0, scene.roof_z + 1e-9, wall_step)
    if zs.size == 0:
        zs = np.zeros(1)
    for (x0, y0, x1, y1) in scene.rects:
        xs_edge = np.arange(x0, x1 + 1e-9, 0.6)
        ys_edge = np.arange(y0, y1 + 1e-9, 0.6)
        if xs_edge.size == 0:
            xs_edge = np.array([0.5 * (x0 + x1)])
        if ys_edge.size == 0:
            ys_edge = np.array([0.5 * (y0 + y1)])
        wx = np.concatenate([xs_edge, xs_edge,
                             np.full(ys_edge.size, x0), np.full(ys_edge.size, x1)])
        wy = np.concatenate([np.full(xs_edge.size, y0), np.full(xs_edge.size, y1),
                             ys_edge, ys_edge])
        gz, gx = np.meshgrid(zs, wx)
        _, gy = np.meshgrid(zs, wy)
        pts.append(np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1))

    cloud = np.vstack([p for p in pts if p.size])
    if noise_sigma > 0.0:
        cloud = cloud + rng.normal(0.0, float(noise_sigma), size=cloud.shape)

    n_out = int(round(float(outlier_ratio) * cloud.shape[0]))
    if n_out > 0:
        out = np.stack([rng.uniform(cx - half, cx + half, n_out),
                        rng.uniform(cy - half, cy + half, n_out),
                        rng.uniform(0.0, scene.roof_z + 3.0, n_out)], axis=1)
        cloud = np.vstack([cloud, out])

    # 不做 shuffle：栅格化只取每格的 max/min/计数，与点序无关（省一次 O(N) 置换）
    return cloud.astype(np.float32)


# --------------------------------------------------------------------------
# 栅格化 / 标签
# --------------------------------------------------------------------------
def scene_to_input(scene, points, cells=64, size_m=None, z_scale=None, z_ref=None):
    """点云 -> 高度图（训练与部署共用 heightmap.points_to_heightmap）。

    ``z_scale`` / ``z_ref`` 默认留空，由点云现算（见 ``heightmap.robust_z_scale``），
    这样不同屋顶高度、不同点云密度的场景都能落在同一动态范围里。
    """
    size_m = scene.size_m if size_m is None else float(size_m)
    return hm_mod.points_to_heightmap(points, scene.center, size_m=size_m,
                                      cells=cells, z_scale=z_scale, z_ref=z_ref)


def target_heatmap(scene, hm, sigma_cells=1.2):
    """训练标签：高斯热力图 + 有效格掩码。

    标签中心默认是规范化落点 ``gt_xy``；只有当真值所在格**没有点**（稀疏点云里很常见）
    时，才**就近投影**到 1.0 m 以内最近的有效格 —— 这样既保证监督信号落在
    网络看得见的地方，又不会让标签随点云密度跳来跳去（试过"直接改选净空最优的
    有效格"，标签在低密度下剧烈抖动，网络反而学不动，落点误差从 0.9 m 涨到 4 m）。

    ``valid`` = 有点的格 ∩ 高度接近屋面高度的格（点云太稀时退回"有点的格"）。
    """
    cells = int(hm["cells"])
    half = 0.5 * hm["size_m"]
    gx = hm["center"][0] - half + (np.arange(cells, dtype=np.float64) + 0.5) * hm["cell"]
    gy = hm["center"][1] - half + (np.arange(cells, dtype=np.float64) + 0.5) * hm["cell"]

    roof_h = (scene.roof_z - hm["z_ref"]) / float(hm["z_scale"])
    valid = np.asarray(hm["mask"], dtype=bool) & (np.abs(hm["height"] - roof_h) <= 0.35)
    if valid.sum() < 8:
        valid = np.asarray(hm["mask"], dtype=bool)

    label_xy = scene.gt_xy
    if valid.any():
        rows, cols = np.nonzero(valid)
        px = gx[cols]
        py = gy[rows]
        d2 = (px - scene.gt_xy[0]) ** 2 + (py - scene.gt_xy[1]) ** 2
        k = int(np.argmin(d2))
        if d2[k] <= 1.0 ** 2:                 # 只在 1 m 内就近投影
            label_xy = (float(px[k]), float(py[k]))

    dx = (gx.reshape(1, -1) - label_xy[0]) / (sigma_cells * hm["cell"])
    dy = (gy.reshape(-1, 1) - label_xy[1]) / (sigma_cells * hm["cell"])
    target = np.exp(-0.5 * (dx * dx + dy * dy)).astype(np.float32)
    return target, valid


def gt_cell(scene, hm):
    """真值落点在高度图里的浮点栅格坐标 (row_f, col_f)，用于统计误差。"""
    half = 0.5 * hm["size_m"]
    col_f = (scene.gt_xy[0] - (hm["center"][0] - half)) / hm["cell"] - 0.5
    row_f = (scene.gt_xy[1] - (hm["center"][1] - half)) / hm["cell"] - 0.5
    return float(row_f), float(col_f)
