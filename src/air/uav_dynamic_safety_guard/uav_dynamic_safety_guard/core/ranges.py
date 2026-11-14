#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""激光点云 → 3×36 距离栅格 → 单帧/时序特征。

单帧特征 116 维（沿用课程已有的安全过滤器约定），时序特征在单帧基础上
再拼两帧历史栅格，共 332 维；历史帧是学生唯一能看出障碍在动的信息来源。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np

__all__ = ["N_AZ", "N_EL", "MAX_RANGE", "HISTORY", "FEAT_DIM", "FEAT_DIM_SEQ",
           "RangeConfig", "points_to_grid", "scan_grid", "corrupt_grid",
           "build_features", "build_sequence_features", "grid_min_range",
           "direction_range", "lidar_lower_bound", "mirror_grid", "MIRROR_IDX"]

N_AZ = 36
N_EL = 3
MAX_RANGE = 8.0
HISTORY = 3
ELEVATIONS_DEG = (-15.0, 0.0, 15.0)
FEAT_DIM = N_EL * N_AZ + 3 + 3 + 1 + 1
FEAT_DIM_SEQ = HISTORY * N_EL * N_AZ + 3 + 3 + 1 + 1

# 机头系镜像（y → -y）时方位角的置换：az_i = -pi + 2 pi i / N_AZ，镜像后 i → -i
MIRROR_IDX = np.array([(-i) % N_AZ for i in range(N_AZ)], dtype=int)


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
    """机头系点云 (M,3) → (N_EL, n_az) 最小距离栅格；无回波记为量程。"""
    grid = np.full((N_EL, n_az), float(max_range), np.float32)
    p = np.asarray(pts, float).reshape(-1, 3)
    if len(p) == 0:
        return grid
    d = np.linalg.norm(p, axis=1)
    keep = (d > 0.05) & (d < float(max_range))
    p, d = p[keep], d[keep]
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
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    pts = lidar.scan(data, state_pos=np.asarray(pos, float), state_mat=rot)
    return points_to_grid(pts, max_range=float(lidar.cfg.max_range),
                          n_az=int(lidar.cfg.n_azimuth))


def corrupt_grid(grid: np.ndarray, rng: np.random.Generator,
                 sigma: float = 0.0, drop: float = 0.0,
                 max_range: float = MAX_RANGE) -> np.ndarray:
    """加测距噪声与随机丢点（鲁棒性评估用）。"""
    g = np.asarray(grid, np.float32).copy()
    if sigma > 0:
        g = np.clip(g + rng.normal(0.0, sigma, g.shape).astype(np.float32),
                    0.0, float(max_range))
    if drop > 0:
        g[rng.random(g.shape) < float(drop)] = float(max_range)
    return g


def _tail(vel: np.ndarray, goal_dir: np.ndarray, dist: float,
          z: float) -> np.ndarray:
    return np.concatenate([
        np.asarray(vel, np.float32).reshape(3) / 3.0,
        np.asarray(goal_dir, np.float32).reshape(3),
        np.array([min(float(dist), 12.0) / 12.0, float(z) / 3.0], np.float32)])


def build_features(grid: np.ndarray, vel: np.ndarray, goal_dir: np.ndarray,
                   dist: float, z: float) -> np.ndarray:
    """单帧特征：距离栅格 + 机头系速度 + 目标方向 + 距离/高度 → 116 维。"""
    r = np.asarray(grid, np.float32).reshape(-1) / MAX_RANGE
    return np.concatenate([r, _tail(vel, goal_dir, dist, z)])


def build_sequence_features(history: np.ndarray, vel: np.ndarray,
                            goal_dir: np.ndarray, dist: float,
                            z: float) -> np.ndarray:
    """时序特征：3 帧历史栅格（新→旧）+ 速度 + 目标 + 距离/高度 → 332 维。"""
    h = np.asarray(history, np.float32).reshape(HISTORY, N_EL, N_AZ)
    r = (h.reshape(-1) / MAX_RANGE).astype(np.float32)
    return np.concatenate([r, _tail(vel, goal_dir, dist, z)])


def grid_min_range(grid: np.ndarray) -> float:
    return float(np.min(np.asarray(grid, np.float32)))


def direction_range(grid: np.ndarray, u: Sequence[float]) -> Tuple[float, float]:
    """指令方向（中心方位与相邻方位）上的最小距离，归一化到 [0,1]。"""
    g = np.asarray(grid, np.float32)
    if math.hypot(float(u[0]), float(u[1])) < 0.2:
        return 1.0, 1.0
    az = math.atan2(float(u[1]), float(u[0]))
    ai = int(round((az + np.pi) / (2 * np.pi) * g.shape[1])) % g.shape[1]
    center = float(g[:, ai].min()) / MAX_RANGE
    near = float(min(g[:, (ai - 1) % g.shape[1]].min(),
                     g[:, (ai + 1) % g.shape[1]].min())) / MAX_RANGE
    return center, near


def lidar_lower_bound(grid: np.ndarray, u: Sequence[float],
                      horizon: float = 0.6) -> float:
    """保守下界：指令方向最近回波减去该方向上的行进距离。"""
    r = direction_range(grid, u)[0] * MAX_RANGE
    return float(r - math.hypot(float(u[0]), float(u[1])) * float(horizon))


def mirror_grid(grid: np.ndarray) -> np.ndarray:
    """沿机头系 x-z 平面镜像（y → -y）后的距离栅格。"""
    g = np.asarray(grid, np.float32)
    return g[..., MIRROR_IDX]
