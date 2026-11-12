#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""感知模块：传感器原始数据 → 定长可用的环境表示。

对应课程任务（2）的「感知」部分。

本模块全部为**纯函数**，不依赖 ROS、不依赖仿真器，可在无环境机器上单测。

两个核心变换
-----------
1. ``sector_histogram``：三维点云 → 16 维扇形最近距离直方图
2. ``depth_passability``：深度图像 → M 维可通行性度量

设计说明
--------
* 直方图每扇取**最小值**而非均值。均值会被远处空旷点稀释，
  导致近处障碍被平均掉；避障关心的是最近障碍。
* 只统计无人机高度 ±z_band 内的点，剔除地面回波，否则地面会被误判为障碍。
* 扇形 0 = 正东（+x），逆时针递增，与 uav_ppo_nav 的约定保持一致。
"""

from __future__ import annotations

import numpy as np

N_SECTORS = 16
R_MAX = 12.0
Z_BAND = 1.5


def sector_histogram(points, n_sectors=N_SECTORS, r_max=R_MAX, z_band=Z_BAND):
    """机体系点云 → 扇形最近距离直方图 ``h[i] ∈ [0, 1]``。

    Parameters
    ----------
    points : array_like, shape (N, 3)
        机体系点云，``(x 前, y 左, z 上)``。空数组/None 表示无回波。
    n_sectors : int
        扇形数量，默认 16（每扇 22.5°）。
    r_max : float
        归一化用的最大量程 (m)。超出此距离的点饱和为 1.0。
    z_band : float
        高度带半宽 (m)，用于剔除地面点。

    Returns
    -------
    numpy.ndarray, shape (n_sectors,), dtype=float32
        每扇的最近障碍归一化距离；1.0 表示该扇无障碍。

    Examples
    --------
    >>> h = sector_histogram([[5.0, 0.0, 0.0]])   # 正东 5 m 处有障碍
    >>> round(float(h[0]), 3)                      # 扇形 0 = 正东
    0.417
    """
    h = np.ones(n_sectors, dtype=np.float32)
    if points is None:
        return h

    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        return h
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError("points 必须是 (N, 3) 形状的数组")

    # 剔除地面：只保留无人机高度附近的点
    z_ok = np.abs(pts[:, 2]) <= z_band
    pts = pts[z_ok]
    if pts.size == 0:
        return h

    # 水平距离与方位角（0 = +x，逆时针）
    r = np.hypot(pts[:, 0], pts[:, 1])
    theta = np.arctan2(pts[:, 1], pts[:, 0])

    delta = 2.0 * np.pi / n_sectors
    idx = np.floor((theta + delta / 2.0) / delta).astype(np.int64) % n_sectors

    norm = np.clip(r / r_max, 0.0, 1.0)

    # 每扇取最小值（最近障碍）
    for i in range(n_sectors):
        sel = norm[idx == i]
        if sel.size:
            h[i] = float(sel.min())
    return h


def depth_passability(depth, n_bands=8, d_safe=3.0):
    """深度图 → 竖直条带可通行性 ``g[j] ∈ [0, 1]``。

    Parameters
    ----------
    depth : array_like, shape (H, W)
        深度图（单位 m）。0 或负数视为无效像素，不参与统计。
        在 ROS 的 ``32FC1`` 深度图中，无穷大表示无回波（可通行）。
    n_bands : int
        竖直条带数量。
    d_safe : float
        安全距离 (m)。深度大于此值判定为可通行。

    Returns
    -------
    numpy.ndarray, shape (n_bands,), dtype=float32
        每条带中可通行像素的占比；1.0 表示整条带都空旷。
    """
    g = np.ones(n_bands, dtype=np.float32)
    if depth is None:
        return g

    d = np.asarray(depth, dtype=np.float64)
    if d.size == 0:
        return g
    if d.ndim != 2:
        raise ValueError("depth 必须是 (H, W) 形状的二维数组")

    h, w = d.shape
    # 有效像素：有限且为正；inf 视为无回波（可通行）
    valid = np.isfinite(d) & (d > 0)
    passable = (~np.isfinite(d)) | (d > d_safe)
    passable &= ~np.isnan(d)

    for j in range(n_bands):
        u0 = int(round(j * w / n_bands))
        u1 = int(round((j + 1) * w / n_bands))
        if u1 <= u0:
            continue
        band_valid = valid[:, u0:u1]
        band_pass = passable[:, u0:u1]
        # 只对有效像素求占比；整条带都无效时保守返回 1.0（视为可通行）
        n_valid = int(band_valid.sum())
        if n_valid == 0:
            g[j] = 1.0
        else:
            g[j] = float(band_pass[band_valid].sum()) / float(n_valid)
    return g


def pointcloud2_to_xyz(msg):
    """``sensor_msgs/PointCloud2`` → ``(N, 3)`` numpy 数组。

    只支持 ``x/y/z`` 为 ``FLOAT32`` 且字段连续排布的常见格式
    （AirSim 与多数雷达驱动的默认输出）。
    """
    import sensor_msgs.point_cloud2 as pc2

    pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if not pts:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(pts, dtype=np.float64)


def safety_scale(hist, front=(15, 0, 1), s_safe=0.6, s_min=0.25):
    """前方安全系数：把感知结果折算成速度缩放比例。

    取正前方若干扇形的最小值作为安全指标 ``s``：

    * ``s > s_safe``  → 返回 1.0（全速）
    * ``s <= s_min``  → 返回 0.0（停止）
    * 之间            → 线性缩放，保证速度**连续无跳变**

    Parameters
    ----------
    hist : array_like
        扇形直方图。
    front : tuple of int
        代表「前方」的扇形索引。默认 (15, 0, 1)
        —— 扇形 0 是正前方偏东，其左右两侧是 15 与 1。
    s_safe, s_min : float
        安全阈值与停车阈值。

    Returns
    -------
    float in [0, 1]
    """
    if hist is None:
        return 1.0
    h = np.asarray(hist, dtype=np.float64)
    if h.size == 0:
        return 1.0

    idx = [i % h.size for i in front]
    s = float(np.min(h[idx]))

    if s > s_safe:
        return 1.0
    if s <= s_min:
        return 0.0
    return (s - s_min) / (s_safe - s_min)
