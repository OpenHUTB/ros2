#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""激光雷达 → 3×36 距离栅格，以及与速度、目标方向拼成的网络输入。"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np

__all__ = ["RangeConfig", "N_AZ", "N_EL", "MAX_RANGE", "FEAT_DIM",
           "scan_grid", "points_to_grid", "corrupt_grid", "build_features",
           "grid_min_range", "direction_range", "lidar_lower_bound"]

N_AZ = 36
N_EL = 3
MAX_RANGE = 8.0
ELEVATIONS_DEG = (-15.0, 0.0, 15.0)
FEAT_DIM = N_EL * N_AZ + 3 + 3 + 1 + 1


@dataclass
class RangeConfig:
    n_azimuth: int = N_AZ
    elevations_deg: Sequence[float] = ELEVATIONS_DEG
    max_range: float = MAX_RANGE
    min_range: float = 0.3
    azimuth_fov_deg: float = 360.0
    frame: str = "body"


def _elevations() -> np.ndarray:
    return np.deg2rad(np.asarray(ELEVATIONS_DEG, float))


def points_to_grid(pts: np.ndarray, max_range: float = MAX_RANGE,
                   n_az: int = N_AZ) -> np.ndarray:
    """机头系点云 (M,3) → (N_EL, N_AZ) 最小距离栅格；无命中为 ``max_range``。"""
    grid = np.full((N_EL, n_az), float(max_range), np.float32)
    p = np.asarray(pts, float).reshape(-1, 3)
    if len(p) == 0:
        return grid
    d = np.linalg.norm(p, axis=1)
    ok = (d > 0.05) & (d < max_range)
    p, d = p[ok], d[ok]
    if len(p) == 0:
        return grid
    az = np.arctan2(p[:, 1], p[:, 0])
    el = np.arcsin(np.clip(p[:, 2] / np.maximum(d, 1e-9), -1.0, 1.0))
    ai = np.rint((az + np.pi) / (2 * np.pi) * n_az).astype(int) % n_az
    ei = np.argmin(np.abs(el[:, None] - _elevations()[None, :]), axis=1)
    np.minimum.at(grid, (ei, ai), d.astype(np.float32))
    return grid


def scan_grid(lidar, data, pos: np.ndarray, yaw: float) -> np.ndarray:
    """在机头系里取一帧距离栅格；``lidar`` 为底座 ``LidarSensor``。"""
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    pts = lidar.scan(data, state_pos=np.asarray(pos, float), state_mat=Rz)
    return points_to_grid(pts, max_range=float(lidar.cfg.max_range),
                          n_az=int(lidar.cfg.n_azimuth))


def corrupt_grid(grid: np.ndarray, rng: np.random.Generator,
                 sigma: float = 0.0, drop: float = 0.0,
                 max_range: float = MAX_RANGE) -> np.ndarray:
    """给距离栅格加测距噪声与随机丢点（鲁棒性评估用）。"""
    g = np.asarray(grid, np.float32).copy()
    if sigma > 0:
        g = np.clip(g + rng.normal(0.0, sigma, g.shape).astype(np.float32),
                    0.0, max_range)
    if drop > 0:
        g[rng.random(g.shape) < drop] = max_range
    return g


def build_features(grid: np.ndarray, vel: np.ndarray, goal_dir: np.ndarray,
                   dist: float, z: float) -> np.ndarray:
    """距离栅格 + 机头系速度 + 目标方向 + 距离/高度 → 116 维特征。"""
    r = np.asarray(grid, np.float32).reshape(-1) / MAX_RANGE
    v = np.asarray(vel, np.float32).reshape(3) / 3.0
    g = np.asarray(goal_dir, np.float32).reshape(3)
    return np.concatenate([r, v, g,
                           np.array([min(float(dist), 12.0) / 12.0,
                                     float(z) / 3.0], np.float32)])


def grid_min_range(grid: np.ndarray) -> float:
    return float(np.min(np.asarray(grid, np.float32)))


def direction_range(grid: np.ndarray, u: Sequence[float]) -> Tuple[float, float]:
    """指令方向上的最小距离（中心方位与相邻方位），归一化到 [0,1]。"""
    g = np.asarray(grid, np.float32)
    if math.hypot(float(u[0]), float(u[1])) < 0.2:
        return 1.0, 1.0          # 纯垂直/悬停指令没有水平方向，不施加方向约束
    az = math.atan2(float(u[1]), float(u[0]))
    ai = int(round((az + np.pi) / (2 * np.pi) * g.shape[1])) % g.shape[1]
    center = float(g[:, ai].min()) / MAX_RANGE
    nb = float(min(g[:, (ai - 1) % g.shape[1]].min(),
                   g[:, (ai + 1) % g.shape[1]].min())) / MAX_RANGE
    return center, nb


def lidar_lower_bound(grid: np.ndarray, u: Sequence[float],
                      horizon: float = 0.6) -> float:
    """保守一步下界：指令方向的最近回波减去该方向的行进距离。"""
    r = direction_range(grid, u)[0] * MAX_RANGE
    return float(r - math.hypot(float(u[0]), float(u[1])) * horizon)
