"""观测编码：局部占据栅格 + 局部到目标距离 + 目标相对位置。"""
from __future__ import annotations

import math

import numpy as np

from .grid import H, RES, W

PATCH = 21
HALF = PATCH // 2
GOAL_RANGE = 3.0
DIST_CLIP = 12.0
N_OBS = PATCH * PATCH * 2 + 2      # 884

__all__ = ["PATCH", "HALF", "GOAL_RANGE", "DIST_CLIP", "N_OBS", "encode"]

_RR, _CC = np.meshgrid(np.arange(PATCH), np.arange(PATCH), indexing="ij")
_FWD = (HALF - _RR) * RES          # 行 0 最靠前
_LAT = (_CC - HALF) * RES          # 列中心为正（左侧）


def _patch(grid: np.ndarray, x: float, y: float, c: float, s: float,
           fill: float) -> np.ndarray:
    wx = x + _FWD * c - _LAT * s
    wy = y + _FWD * s + _LAT * c
    ii = np.floor(wx / RES + 1e-3).astype(np.int32)
    jj = np.floor(wy / RES + 1e-3).astype(np.int32)
    inside = (ii >= 0) & (ii < W) & (jj >= 0) & (jj < H)
    out = np.full((PATCH, PATCH), fill, np.float32)
    out[inside] = grid[jj[inside], ii[inside]]
    return out


def encode(g: np.ndarray, pose, goal, dist: np.ndarray | None = None) -> np.ndarray:
    """pose=(x,y,yaw)，goal=(x,y)，dist 为到目标的测地距离图。

    输出长度 884：21×21 占据栅格 + 21×21 归一化距离 + 目标机体坐标。
    """
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)

    occ = _patch(g.astype(np.float32), x, y, c, s, 1.0)

    if dist is None:
        dmap = np.zeros((PATCH, PATCH), np.float32)
    else:
        d = _patch(dist.astype(np.float32), x, y, c, s, DIST_CLIP)
        d = np.where(np.isfinite(d), d, DIST_CLIP)
        dmap = np.clip(d / DIST_CLIP, 0.0, 1.0)

    gx, gy = goal[0] - x, goal[1] - y
    norm = math.hypot(gx, gy)
    if norm > GOAL_RANGE:
        k = GOAL_RANGE / norm
        gx, gy = gx * k, gy * k
    gxb = (gx * c + gy * s) / GOAL_RANGE
    gyb = (-gx * s + gy * c) / GOAL_RANGE

    return np.concatenate(
        [occ.ravel(), dmap.ravel(),
         np.array([gxb, gyb], np.float32)]).astype(np.float32)
