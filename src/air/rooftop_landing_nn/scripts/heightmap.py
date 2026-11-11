#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""点云 -> 高度图栅格化（纯 numpy，不依赖 ROS / torch）。

本文件是整个模块的「公共预处理」：训练脚本（tools/train_landing_cnn.py）与
ROS 节点（scripts/rooftop_landing_nn_node.py）都用同一份代码把点云变成网络输入，
避免出现「训练时这样栅格化、部署时那样栅格化」的经典事故。

坐标约定（与 rooftop_landing / carlair_ros_bridge 一致）：
    * 点云是 ENU，x 向前、y 向左、z 向上为正；
    * 栅格数组 h[row, col]，``row`` 沿 +y 增大，``col`` 沿 +x 增大；
    * ROI 是以 ``center_xy`` 为中心、边长 ``size_m`` 的正方形。
"""
from __future__ import annotations

import math

import numpy as np

#: 默认栅格参数（与 config/rooftop_landing_nn.yaml、models/*.npz 里的 meta 保持一致）
DEFAULT_CELLS = 64
DEFAULT_SIZE_M = 16.0
DEFAULT_Z_SCALE = 4.0


def _as_points(points):
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError("points must be (N, 3) or (N, >=3), got %r" % (pts.shape,))
    pts = pts[:, :3]
    return pts[np.isfinite(pts).all(axis=1)]


def robust_floor_z(points, percentile=10.0, band=0.5):
    """稳健估计「参考面」高度：先取 z 的 ``percentile`` 分位数，再取落在
    ``[q, q + band]`` 内的点的均值。

    为什么不直接用最小值 / 极低分位数：会被个别离群低点带偏。
    为什么不直接用中位数：当 ROI 里「地面点比屋面点少」（远处高层屋顶）
    时会把参考面抬到屋面高度，于是整张高度图都接近 0。
    先分位数定位、再取带内均值，对「地面稀疏、屋面密集」的真实点云更稳。
    """
    pts = _as_points(points)
    if pts.shape[0] == 0:
        return 0.0
    z = pts[:, 2]
    q = float(np.percentile(z, percentile))
    band_pts = z[z <= q + float(band)]
    if band_pts.size == 0:
        return q
    return float(np.mean(band_pts))


def robust_z_scale(points, z_ref=None, percentile=98.0, min_scale=2.0, headroom=1.15):
    """ROI 内「高于参考面的高度」的稳健上界，作为高度归一化尺度 (m)。

    为什么不用固定值（例如写死 30 m）：4 m 的矮屋顶会挤到 0.13，网络几乎没有对比度；
    为什么不用 ``max(z)``：个别离群高点会把尺度拉爆。
    取 98% 分位数再留 15% 余量，屋面大致落在 0.8~0.9，既保留对比度又给障碍物留空间，
    而且**训练与部署都从点云现算**，天然尺度无关。
    """
    pts = _as_points(points)
    if pts.shape[0] == 0:
        return float(min_scale)
    if z_ref is None:
        z_ref = robust_floor_z(pts)
    above = pts[:, 2] - float(z_ref)
    scale = float(np.percentile(above, percentile)) * float(headroom)
    return float(max(float(min_scale), scale))


def points_to_heightmap(points,
                        center_xy,
                        size_m=DEFAULT_SIZE_M,
                        cells=DEFAULT_CELLS,
                        z_scale=None,
                        z_ref=None,
                        min_points=1):
    """把 ROI 内的点栅格化成高度图。

    返回 dict：

    ==========  ==========================================================
    ``height``  (cells, cells) float32，每格最高点相对 ``z_ref`` 的高度，
                截断到 [0, z_scale] 后除以 ``z_scale``（即 0~1）
    ``rough``   (cells, cells) float32，每格内 (max z - min z) 同样归一化，
                平坦屋面接近 0，边缘 / 障碍物 / 立面处明显变大
    ``mask``    (cells, cells) bool，该格是否至少有 ``min_points`` 个点
    ``count``   (cells, cells) int32，每格点数（调试 / 可视化用）
    ``cell``    单格边长 (m)
    ``center``  ROI 中心 (x, y)
    ``z_ref``   参考面高度 (m)
    ==========  ==========================================================
    """
    pts = _as_points(points)
    size_m = float(size_m)
    cells = int(cells)
    if cells <= 0 or size_m <= 0.0:
        raise ValueError("cells / size_m must be positive")
    if z_ref is None:
        z_ref = robust_floor_z(pts)
    if z_scale is None:
        z_scale = robust_z_scale(pts, z_ref=z_ref)
    z_scale = float(z_scale)
    if z_scale <= 0.0:
        raise ValueError("z_scale must be positive")

    out = {
        "height": np.zeros((cells, cells), dtype=np.float32),
        "rough": np.zeros((cells, cells), dtype=np.float32),
        "mask": np.zeros((cells, cells), dtype=bool),
        "count": np.zeros((cells, cells), dtype=np.int32),
        "cell": size_m / float(cells),
        "center": (float(center_xy[0]), float(center_xy[1])),
        "size_m": size_m,
        "cells": cells,
        "z_scale": z_scale,
        "z_ref": float(z_ref),
    }
    if pts.shape[0] == 0:
        return out

    half = 0.5 * size_m
    dx = pts[:, 0] - out["center"][0]
    dy = pts[:, 1] - out["center"][1]
    inside = (np.abs(dx) <= half) & (np.abs(dy) <= half)
    dx, dy = dx[inside], dy[inside]
    z = pts[inside, 2]
    if z.size == 0:
        return out

    cell = out["cell"]
    col = np.floor((dx + half) / cell).astype(np.int64)
    row = np.floor((dy + half) / cell).astype(np.int64)
    np.clip(col, 0, cells - 1, out=col)
    np.clip(row, 0, cells - 1, out=row)

    # 每格的 max / min / count：排序 + reduceat 比 np.maximum.at 快一个量级
    # （np.*.at 是逐元素 ufunc，几千点就要十几毫秒，实时节点里累积多帧会很吃紧）
    flat_idx = row * cells + col
    order = np.argsort(flat_idx, kind="stable")
    idx_sorted = flat_idx[order]
    z_sorted = z[order]
    uniq, first = np.unique(idx_sorted, return_index=True)
    zmax_flat = np.maximum.reduceat(z_sorted, first)
    zmin_flat = np.minimum.reduceat(z_sorted, first)
    cnt_flat = np.diff(np.append(first, idx_sorted.size))

    zmax_vec = np.full(cells * cells, -np.inf)
    zmin_vec = np.full(cells * cells, np.inf)
    cnt_vec = np.zeros(cells * cells, dtype=np.int64)
    zmax_vec[uniq] = zmax_flat
    zmin_vec[uniq] = zmin_flat
    cnt_vec[uniq] = cnt_flat

    zmax = zmax_vec.reshape(cells, cells)
    zmin = zmin_vec.reshape(cells, cells)
    cnt = cnt_vec.reshape(cells, cells)

    filled = cnt > 0
    zmax_f = np.where(filled, zmax, out["z_ref"])
    zmin_f = np.where(filled, zmin, out["z_ref"])

    height = (zmax_f - out["z_ref"]) / z_scale
    rough = (zmax_f - zmin_f) / z_scale
    out["height"] = np.clip(height, 0.0, 1.0).astype(np.float32)
    out["rough"] = np.clip(rough, 0.0, 1.0).astype(np.float32)
    out["count"] = cnt.astype(np.int32)
    out["mask"] = cnt >= int(min_points)
    return out


def heightmap_channels(hm):
    """高度图 -> 网络输入张量 (3, cells, cells) float32。

    通道顺序固定为：``[归一化高度, 有效掩码, 粗糙度]``，训练与推理必须一致。
    """
    h = hm["height"]
    m = hm["mask"].astype(np.float32)
    r = hm["rough"]
    return np.stack([h.astype(np.float32), m, r.astype(np.float32)], axis=0)


def cell_to_world(row, col, hm):
    """栅格 (row, col) -> 该格中心的 ENU (x, y)。"""
    half = 0.5 * hm["size_m"]
    cx, cy = hm["center"]
    x = cx - half + (float(col) + 0.5) * hm["cell"]
    y = cy - half + (float(row) + 0.5) * hm["cell"]
    return float(x), float(y)


def world_to_cell(x, y, hm):
    """ENU (x, y) -> 栅格 (row, col)；落在 ROI 外返回 ``None``。"""
    half = 0.5 * hm["size_m"]
    cx, cy = hm["center"]
    col = math.floor((float(x) - cx + half) / hm["cell"])
    row = math.floor((float(y) - cy + half) / hm["cell"])
    if 0 <= row < hm["cells"] and 0 <= col < hm["cells"]:
        return int(row), int(col)
    return None


def cell_surface_z(points, row, col, hm, band=0.30):
    """取某个栅格内「上表面」的高度（m）。

    屋面点云在竖直方向也有噪声，直接取 max 会被离群高点带偏，因此取
    「与本格最高点的差在 ``band`` 米以内的那些点的中位数」，对稀疏点云更稳。
    该格没有点则退回 ``z_ref``。
    """
    pts = _as_points(points)
    if pts.shape[0] == 0:
        return float(hm["z_ref"])
    half = 0.5 * hm["size_m"]
    cx, cy = hm["center"]
    cell = hm["cell"]
    x0 = cx - half + col * cell
    y0 = cy - half + row * cell
    sel = ((pts[:, 0] >= x0) & (pts[:, 0] < x0 + cell) &
           (pts[:, 1] >= y0) & (pts[:, 1] < y0 + cell))
    z = pts[sel, 2]
    if z.size == 0:
        return float(hm["z_ref"])
    top = float(np.max(z))
    near = z[z >= top - float(band)]
    return float(np.median(near))


def roi_center_from_points(points, fallback_xy=(0.0, 0.0)):
    """没有先验目标时，用点云的 XY 中位数作为 ROI 中心（节点里配合 UAV 位置用）。"""
    pts = _as_points(points)
    if pts.shape[0] == 0:
        return (float(fallback_xy[0]), float(fallback_xy[1]))
    return (float(np.median(pts[:, 0])), float(np.median(pts[:, 1])))
