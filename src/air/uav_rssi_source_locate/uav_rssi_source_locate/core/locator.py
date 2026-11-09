#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Search pattern, measurement rings and least-squares localization.

Single-line summary: expanding-square search until first detection, then
measurement rings around the current estimate with Levenberg-Marquardt
trilateration; each ring shrinks as the residual drops.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .rssi import RssiCfg


@dataclass(frozen=True)
class SearchCfg:
    """搜索与定位参数。

    测量圈分两级：探圈（explore）在检测点附近小半径转一圈，用信号弧
    段上的样本拿到粗估计；精圈（fine）以估计点为圆心——只要估计误差
    + 半径 < 可检测半径，整圈都有信号，几何完整，误差收敛到米级。
    检测发生在可检测半径边界（~43 m）处，若直接在检测点拉一个
    0.6×43 m 的大圈，远端半圈收不到信号，只剩单侧弧样本，定位误差
    15~25 m（实测）。
    """

    area: Tuple[float, float] = (220.0, 160.0)   # 搜索区（x, y）m
    grid_step: float = 30.0                      # 扩张方形步长 m
    cruise_speed: float = 9.0                    # m/s
    alt_m: float = 20.0                          # 搜索高度
    explore_radius_m: float = 15.0               # 检测点探圈半径
    explore_points: int = 12                     # 探圈测量点数
    approach_range_m: float = 20.0               # 反演距离近于此值转精圈
    fine_radius0_m: float = 10.0                 # 精圈初始半径
    ring_points: int = 8                         # 精圈测量点数
    ring_shrink: float = 0.55                    # 每轮定位后半径收缩比
    max_refine_rounds: int = 2

    def __post_init__(self):
        if self.grid_step <= 0.0 or self.ring_points < 3:
            raise ValueError("grid_step must be positive, ring_points >= 3")


def expanding_square(cfg: SearchCfg) -> List[Tuple[float, float]]:
    """从区域中心出发的扩张方形航点序列（覆盖全区或直到飞出边界）。"""
    cx, cy = cfg.area[0] / 2.0, cfg.area[1] / 2.0
    wpts = [(cx, cy)]
    k = 1
    while True:
        half = k * cfg.grid_step
        # 一圈：右 → 上 → 左 → 下（回到左下角再扩张）
        ring = [(cx + half, cy - half + 0), (cx + half, cy + half),
                (cx - half, cy + half), (cx - half, cy - half)]
        out_of_bounds = all(not (0 <= x <= cfg.area[0] and 0 <= y <= cfg.area[1])
                            for x, y in ring)
        wpts += ring
        k += 1
        if out_of_bounds or half > max(cfg.area) * 0.75:
            break
    return wpts


def ring_points(center, radius: float, k: int) -> List[np.ndarray]:
    """估计点周围的测量圈：k 个等分角度的圆上点。"""
    c = np.asarray(center, dtype=float)[:2]
    return [c + radius * np.array([np.cos(2 * np.pi * i / k),
                                   np.sin(2 * np.pi * i / k)])
            for i in range(k)]


def best_waypoint(waypoints: Sequence, rssi_fn) -> Tuple[np.ndarray, float]:
    """朴素基线：在航点上取 RSSI 最强的一点当信标估计。

    rssi_fn(x, y) -> dBm 或 nan。返回 (估计点, 峰值 RSSI)。这是"照着
    网格飞、哪里信号强就停哪"的做法，精度被航点间距锁死（本配置
    grid_step=30 m，误差约十来米），本模块的三边定位把它压到米级。
    """
    best_p, best_r = None, -np.inf
    for w in waypoints:
        r = rssi_fn(float(w[0]), float(w[1]))
        if np.isfinite(r) and r > best_r:
            best_p, best_r = np.asarray(w, dtype=float)[:2], float(r)
    if best_p is None:
        raise ValueError("no waypoint received any signal")
    return best_p, best_r


def localize(samples: Sequence[Tuple[np.ndarray, float]],
             rssi_cfg: RssiCfg,
             init: Optional[np.ndarray] = None,
             beacon_z: float = 0.5) -> dict:
    """由 (位置, RSSI) 样本做三边定位（LM 阻尼最小二乘）。

    位置样本用三维 (x, y, z)：RSSI 来自信标真实 3D 距离，若把样本压
    到 2D 再反演，20 m 高度会变成 ~6 m 的系统性距离偏差，估计整体
    偏移几十米。模型：|p_i − (bx, by, beacon_z)| = d_i，只解 (bx, by)，
    高度差 h_i = p_i.z − beacon_z 进残差 r_i = sqrt(d2d² + h_i²) − d_i。
    样本 < 3 个或全部共线时方程退化：返回 feasible=False（共线在
    阻尼下也能解，但残差/协方差不可信，直接拒绝并建议换测量点）。
    返回 estimate、residual_m（均方根）、n_samples。
    """
    if len(samples) < 3:
        return dict(feasible=False, reason="need >= 3 samples",
                    estimate=None, residual_m=None, n_samples=len(samples))
    P3 = np.asarray([np.asarray(p, dtype=float)[:3] for p, _r in samples])
    P = P3[:, :2]
    hz = np.abs(P3[:, 2] - beacon_z)
    D = np.asarray([rssi_cfg.distance_from(r) for _p, r in samples])
    # 共线检查：采样点协方差矩阵最小奇异值
    sv = np.linalg.svd(P - P.mean(axis=0), compute_uv=False)
    if sv.min() < 1e-6:
        return dict(feasible=False, reason="collinear samples",
                    estimate=None, residual_m=None, n_samples=len(samples))
    centroid = P.mean(axis=0)
    spread = float(np.linalg.svd(P - centroid, compute_uv=False)[0]) \
        if len(P) >= 2 else 10.0
    # 多起点：GN 从差的初值会掉进局部极小（同一组样本，初值 (146,99)
    # 收敛到残差 6.3 m 的假点，质心初值收敛到 1.4 m 的真解），网格撒
    # 一圈起点取残差最小者
    inits = [centroid]
    if init is not None:
        inits.append(np.asarray(init, dtype=float)[:2])
    for ang in np.arange(0.0, 2 * np.pi, np.pi / 4):
        inits.append(centroid + spread * 1.5
                     * np.array([np.cos(ang), np.sin(ang)]))

    def _resid(b2):
        d2d = np.linalg.norm(P - b2, axis=1)
        return np.sqrt(d2d ** 2 + hz ** 2) - D

    def _gn(b, lam=1e-3):
        res = _resid(b)
        for _ in range(60):
            d2d = np.linalg.norm(P - b, axis=1)
            d3d = np.sqrt(d2d ** 2 + hz ** 2)
            J = ((b - P) / np.maximum(d3d, 1e-9)[:, None])
            H = J.T @ J + lam * np.eye(2)
            g = J.T @ res
            try:
                step = np.linalg.solve(H, g)
            except np.linalg.LinAlgError:
                lam *= 10.0
                continue
            b_new = b - step
            res_new = _resid(b_new)
            if float(np.sum(res_new ** 2)) < float(np.sum(res ** 2)):
                b = b_new
                res = res_new
                lam = max(lam * 0.3, 1e-6)
                if float(np.linalg.norm(step)) < 1e-9:
                    break
            else:
                lam *= 10.0
                if lam > 1e6:
                    break
        return b, float(np.sum(res ** 2))

    best_b, best_sse = None, np.inf
    for b0 in inits:
        b_i, sse_i = _gn(np.asarray(b0, dtype=float))
        if sse_i < best_sse:
            best_b, best_sse = b_i, sse_i
    b = best_b

    rms = float(np.sqrt(np.mean(_resid(b) ** 2)))
    return dict(feasible=True, reason="", estimate=np.asarray(b, dtype=float),
                residual_m=round(rms, 3), n_samples=len(samples))


@dataclass
class SearchState:
    """一次搜索任务的簿记（航点序列 + 测量样本 + 历轮估计）。"""

    cfg: SearchCfg
    waypoints: List[Tuple[float, float]] = field(default_factory=list)
    samples: List[Tuple[np.ndarray, float]] = field(default_factory=list)  # (3D 位置, RSSI)
    estimate: Optional[np.ndarray] = None
    residual_m: Optional[float] = None
    round_no: int = 0
    detected_at: Optional[np.ndarray] = None
