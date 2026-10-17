#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平面几何工具：角度归一化、参考折线上的最近点投影与切向。

本模块不依赖 ROS 与 matplotlib，可单独导入用于离线测试。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np


def wrap_to_pi(angle: float | np.ndarray) -> float | np.ndarray:
    """把角度归一化到 (-pi, pi]。

    约定：车辆朝向与参考路径切向的偏差必须用该函数处理，
    否则在 +/-pi 附近会产生 2*pi 的假跳变。
    """
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


class ProjectionResult(NamedTuple):
    """车辆在参考折线上的投影结果。"""

    index: int          # 投影点所在线段的起点索引
    t: float            # 该线段上的插值比例，范围 [0, 1]
    point: np.ndarray   # 投影点坐标（x, y）
    tangent: np.ndarray # 投影点处单位切向量
    normal: np.ndarray  # 投影点处单位左法向量（切向逆时针旋转 90 度）
    curvature: float    # 投影点处曲率（左转为正）
    s: float            # 投影点沿参考路径的弧长
    signed_distance: float  # 有符号横向偏差，车辆在路径左侧为正，右侧为负


def normalize(vector: np.ndarray) -> np.ndarray:
    """把向量归一化为单位向量；零向量返回 (1, 0)。"""
    vector = np.asarray(vector, dtype=float)
    norm = float(np.hypot(vector[0], vector[1]))
    if norm < 1e-12:
        return np.array([1.0, 0.0])
    return vector / norm


class ReferencePath:
    """参考路径：由一串二维离散点构成，按里程均匀或非均匀参数化。

    与常见的"均匀重采样后按索引取点"不同，这里统一按弧长 s 做参数化，
    从而保证相邻点间距不均匀时切向和曲率仍然正确。
    """

    def __init__(self, points: np.ndarray, closed: bool = False) -> None:
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("points 的形状必须是 (N, 2)")
        if points.shape[0] < 2:
            raise ValueError("参考路径至少需要 2 个点")

        self.closed = bool(closed)
        if self.closed and not np.allclose(points[0], points[-1]):
            points = np.vstack([points, points[0]])
        self.points = points

        # 线段向量、长度、累计弧长
        segments = np.diff(points, axis=0)
        lengths = np.hypot(segments[:, 0], segments[:, 1])
        if np.any(lengths < 1e-12):
            raise ValueError("参考路径存在重合点，请先去除重复点")
        self.segments = segments
        self.lengths = lengths
        self.tangents = segments / lengths[:, None]
        self.normals = np.column_stack([-self.tangents[:, 1], self.tangents[:, 0]])
        self.s = np.concatenate([[0.0], np.cumsum(lengths)])
        self.total_length = float(self.s[-1])

        # 每个折点的曲率：用相邻线段的转角除以两段长度之和
        self.vertex_curvature = self._compute_vertex_curvature()

    def _compute_vertex_curvature(self) -> np.ndarray:
        """用相邻线段夹角估计各折点处曲率，用于前馈转向。"""
        n = self.points.shape[0]
        curvature = np.zeros(n)
        for i in range(1, n - 1):
            t_prev = self.tangents[i - 1]
            t_next = self.tangents[i]
            cross = t_prev[0] * t_next[1] - t_prev[1] * t_next[0]
            dot = float(np.clip(np.dot(t_prev, t_next), -1.0, 1.0))
            dtheta = math.atan2(cross, dot)
            ds = 0.5 * (self.lengths[i - 1] + self.lengths[i])
            curvature[i] = dtheta / ds if ds > 1e-12 else 0.0
        if self.closed:
            # 闭合路径的首点：转角取"倒数第二段 -> 收尾段"。
            # points 首尾相接，收尾段（tangents[-1]）正是从最后一个独立点回到
            # 起点的线段，它与 tangents[-2] 构成首点两侧的真实夹角；
            # 若改用 tangents[0]，首点两侧的夹角会被算成约两倍，曲率偏大。
            t_prev, t_next = self.tangents[-2], self.tangents[-1]
            cross = t_prev[0] * t_next[1] - t_prev[1] * t_next[0]
            dot = float(np.clip(np.dot(t_prev, t_next), -1.0, 1.0))
            ds = 0.5 * (self.lengths[-2] + self.lengths[0])
            curvature[0] = curvature[-1] = math.atan2(cross, dot) / ds
        else:
            curvature[0] = curvature[1]
            curvature[-1] = curvature[-2]
        return curvature

    def __len__(self) -> int:
        return self.points.shape[0]

    def point_at_index(self, index: int) -> np.ndarray:
        return self.points[index % self.points.shape[0]]

    def sample(self, count: int = 400) -> np.ndarray:
        """按弧长均匀采样，用于绘图或导出参考轨迹。"""
        s_query = np.linspace(0.0, self.total_length, count)
        return np.array([self.point_at_s(float(s)) for s in s_query])

    def point_at_s(self, s: float) -> np.ndarray:
        """按弧长取参考点坐标（线性插值）。"""
        s = float(np.clip(s, 0.0, self.total_length))
        index = int(np.searchsorted(self.s, s, side="right") - 1)
        index = min(max(index, 0), self.lengths.shape[0] - 1)
        t = (s - self.s[index]) / self.lengths[index]
        return self.points[index] + t * self.segments[index]

    def curvature_at_index(self, index: int, t: float = 0.5) -> float:
        """投影点处曲率：相邻两折点曲率按 t 线性插值。"""
        n = self.points.shape[0]
        i0 = index % n
        i1 = (index + 1) % n if self.closed else min(index + 1, n - 1)
        return float((1.0 - t) * self.vertex_curvature[i0] + t * self.vertex_curvature[i1])

    def curvature_at_s(self, s: float) -> float:
        """按弧长取曲率，供前馈项使用。"""
        s = float(np.clip(s, 0.0, self.total_length))
        index = int(np.searchsorted(self.s, s, side="right") - 1)
        index = min(max(index, 0), self.lengths.shape[0] - 1)
        t = (s - self.s[index]) / self.lengths[index]
        return self.curvature_at_index(index, t)

    def project(self, point: np.ndarray) -> ProjectionResult:
        """把任意点投影到参考折线上，返回最近点及其切向、法向、曲率与横向偏差。

        signed_distance 的符号由左法向量决定：车辆位于路径左侧为正，右侧为负。
        """
        point = np.asarray(point, dtype=float)
        rel = point[None, :] - self.points[:-1, :]
        seg_len2 = self.lengths ** 2
        t = np.clip((rel * self.segments).sum(axis=1) / seg_len2, 0.0, 1.0)
        proj = self.points[:-1, :] + t[:, None] * self.segments
        diff = point[None, :] - proj
        dist2 = (diff ** 2).sum(axis=1)
        index = int(np.argmin(dist2))
        t_i = float(t[index])

        tangent = self.tangents[index]
        normal = self.normals[index]
        signed_distance = float(np.dot(diff[index], normal))
        s_i = float(self.s[index] + t_i * self.lengths[index])
        return ProjectionResult(
            index=index,
            t=t_i,
            point=proj[index],
            tangent=tangent,
            normal=normal,
            curvature=self.curvature_at_index(index, t_i),
            s=s_i,
            signed_distance=signed_distance,
        )

    def project_forward(self, point: np.ndarray, hint_index: int = 0,
                        backward: int = 2, forward: int = 40) -> ProjectionResult:
        """只在上一次投影位置附近向前搜索的投影，用于闭环控制。

        直接对全路径求最近点有两个问题：车辆偏离较远或已经掉头时，
        最近点可能跳到车辆**后方**，使切向、航向偏差与弧长发生跳变，
        进而让控制器输出突变。这里把搜索范围限制在
        ``[hint_index - backward, hint_index + forward]`` 内，
        保证投影沿参考路径单调前进。
        """
        segment_count = self.lengths.shape[0]
        if self.closed:
            lo = hint_index - backward
            hi = hint_index + forward
            indices = np.arange(lo, hi + 1) % segment_count
        else:
            lo = max(0, hint_index - backward)
            hi = min(segment_count - 1, hint_index + forward)
            indices = np.arange(lo, hi + 1)

        point = np.asarray(point, dtype=float)
        starts = self.points[indices]
        rel = point[None, :] - starts
        segs = self.segments[indices]
        len2 = self.lengths[indices] ** 2
        t = np.clip((rel * segs).sum(axis=1) / len2, 0.0, 1.0)
        proj = starts + t[:, None] * segs
        diff = point[None, :] - proj
        dist2 = (diff ** 2).sum(axis=1)
        pick = int(np.argmin(dist2))
        index = int(indices[pick])
        t_i = float(t[pick])

        tangent = self.tangents[index]
        normal = self.normals[index]
        signed_distance = float(np.dot(diff[pick], normal))
        s_i = float(self.s[index] + t_i * self.lengths[index])
        return ProjectionResult(
            index=index,
            t=t_i,
            point=proj[pick],
            tangent=tangent,
            normal=normal,
            curvature=self.curvature_at_index(index, t_i),
            s=s_i,
            signed_distance=signed_distance,
        )
