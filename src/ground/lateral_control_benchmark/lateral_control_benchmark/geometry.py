#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平面几何与参考轨迹。

本模块是纯函数库，不依赖 ROS，便于离线单元测试：
- 角度归一化、折线最近点投影
- 五种参考轨迹（直线 / 圆弧 / 正弦 / 双移线 / 显式航点）
- 按弧长参数化的 `ReferencePath`，提供切向、法向、曲率
"""

from __future__ import annotations

import math
from typing import List, NamedTuple, Sequence, Tuple

import numpy as np


def wrap_to_pi(angle: float | np.ndarray) -> float | np.ndarray:
    """把角度归一化到 (-pi, pi]。"""
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


class Projection(NamedTuple):
    """车辆在参考折线上的投影结果。"""

    index: int              # 投影所在线段的起点索引
    t: float                # 线段上的插值比例 [0,1]
    point: np.ndarray       # 投影点坐标
    tangent: np.ndarray     # 单位切向
    normal: np.ndarray      # 单位左法向（切向逆时针转 90°）
    curvature: float        # 投影点曲率（左转为正）
    s: float                # 投影点弧长
    signed_distance: float  # 有符号横向偏差（左正右负）


class ReferencePath:
    """按弧长参数化的参考路径。"""

    def __init__(self, points: np.ndarray, closed: bool = False) -> None:
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("points 形状必须是 (N, 2)")
        if points.shape[0] < 2:
            raise ValueError("至少需要 2 个点")

        self.closed = bool(closed)
        if self.closed and not np.allclose(points[0], points[-1]):
            points = np.vstack([points, points[0]])
        self.points = points

        segments = np.diff(points, axis=0)
        lengths = np.hypot(segments[:, 0], segments[:, 1])
        if np.any(lengths < 1e-12):
            raise ValueError("存在重合点，请先去重")
        self.segments = segments
        self.lengths = lengths
        self.tangents = segments / lengths[:, None]
        self.normals = np.column_stack([-self.tangents[:, 1], self.tangents[:, 0]])
        self.s = np.concatenate([[0.0], np.cumsum(lengths)])
        self.total_length = float(self.s[-1])
        self.vertex_curvature = self._vertex_curvature()

    def _vertex_curvature(self) -> np.ndarray:
        """用相邻线段夹角估计各折点曲率。"""
        n = self.points.shape[0]
        curvature = np.zeros(n)
        for i in range(1, n - 1):
            dtheta = self._turning_angle(self.tangents[i - 1], self.tangents[i])
            ds = 0.5 * (self.lengths[i - 1] + self.lengths[i])
            curvature[i] = dtheta / ds if ds > 1e-12 else 0.0
        if self.closed:
            dtheta = self._turning_angle(self.tangents[-2], self.tangents[-1])
            ds = 0.5 * (self.lengths[-2] + self.lengths[-1])
            curvature[0] = curvature[-1] = dtheta / ds if ds > 1e-12 else 0.0
        else:
            curvature[0] = curvature[1]
            curvature[-1] = curvature[-2]
        return curvature

    @staticmethod
    def _turning_angle(a: np.ndarray, b: np.ndarray) -> float:
        cross = a[0] * b[1] - a[1] * b[0]
        dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
        return math.atan2(cross, dot)

    def __len__(self) -> int:
        return self.points.shape[0]

    def point_at_s(self, s: float) -> np.ndarray:
        s = float(np.clip(s, 0.0, self.total_length))
        i = int(np.searchsorted(self.s, s, side="right") - 1)
        i = min(max(i, 0), self.lengths.shape[0] - 1)
        t = (s - self.s[i]) / self.lengths[i]
        return self.points[i] + t * self.segments[i]

    def curvature_at_s(self, s: float) -> float:
        s = float(np.clip(s, 0.0, self.total_length))
        i = int(np.searchsorted(self.s, s, side="right") - 1)
        i = min(max(i, 0), self.lengths.shape[0] - 1)
        t = (s - self.s[i]) / self.lengths[i]
        j = (i + 1) % self.points.shape[0] if self.closed else min(i + 1, self.points.shape[0] - 1)
        return float((1.0 - t) * self.vertex_curvature[i] + t * self.vertex_curvature[j])

    def project(self, point: np.ndarray) -> Projection:
        point = np.asarray(point, dtype=float)
        rel = point[None, :] - self.points[:-1, :]
        t = np.clip((rel * self.segments).sum(axis=1) / (self.lengths ** 2), 0.0, 1.0)
        proj = self.points[:-1, :] + t[:, None] * self.segments
        diff = point[None, :] - proj
        i = int(np.argmin((diff ** 2).sum(axis=1)))
        return Projection(
            index=i, t=float(t[i]), point=proj[i],
            tangent=self.tangents[i], normal=self.normals[i],
            curvature=self.curvature_at_s(float(self.s[i] + t[i] * self.lengths[i])),
            s=float(self.s[i] + t[i] * self.lengths[i]),
            signed_distance=float(np.dot(diff[i], self.normals[i])),
        )

    def sample(self, count: int = 500) -> np.ndarray:
        return np.array([self.point_at_s(s) for s in np.linspace(0.0, self.total_length, count)])


# ---------------------------------------------------------------- 轨迹生成
def straight_line(length: float = 80.0, step: float = 0.5) -> ReferencePath:
    x = np.arange(0.0, length + 1e-9, step)
    return ReferencePath(np.column_stack([x, np.zeros_like(x)]))


def circular_arc(radius: float = 20.0, sweep_deg: float = 360.0, step_deg: float = 1.0) -> ReferencePath:
    if radius <= 0:
        raise ValueError("半径必须为正")
    angles = np.radians(np.arange(0.0, sweep_deg + 1e-9, step_deg))
    x = radius * np.sin(angles)
    y = radius * (1.0 - np.cos(angles))
    return ReferencePath(np.column_stack([x, y]), closed=abs(sweep_deg - 360.0) < 1e-6)


def sine_wave(amplitude: float = 3.0, wavelength: float = 40.0,
              length: float = 120.0, step: float = 0.25) -> ReferencePath:
    x = np.arange(0.0, length + 1e-9, step)
    y = amplitude * np.sin(2.0 * np.pi * x / wavelength)
    return ReferencePath(np.column_stack([x, y]))


def double_lane_change(length: float = 120.0, width: float = 3.5, step: float = 0.25) -> ReferencePath:
    x = np.arange(0.0, length + 1e-9, step)
    u = x / length
    y = width * 0.5 * (np.tanh(6 * (u - 0.2)) - np.tanh(6 * (u - 0.4))
                       - np.tanh(6 * (u - 0.6)) + np.tanh(6 * (u - 0.8)))
    return ReferencePath(np.column_stack([x, y]))


def from_waypoints(points: Sequence[Tuple[float, float]], closed: bool = False) -> ReferencePath:
    return ReferencePath(np.asarray(points, dtype=float), closed=closed)


def from_csv(path: str, closed: bool = False) -> ReferencePath:
    data = np.genfromtxt(path, delimiter=",", comments="#")
    if data.ndim != 2 or data.shape[1] < 2:
        raise ValueError("CSV 至少两列 (x, y)")
    return ReferencePath(data[:, :2], closed=closed)


SCENARIOS = {
    "straight": lambda: straight_line(),
    "arc": lambda: circular_arc(),
    "sine": lambda: sine_wave(),
    "lane_change": lambda: double_lane_change(),
}


def build_scenario(name: str) -> ReferencePath:
    if name not in SCENARIOS:
        raise KeyError(f"未知场景 '{name}'，可选: {', '.join(SCENARIOS)}")
    return SCENARIOS[name]()
