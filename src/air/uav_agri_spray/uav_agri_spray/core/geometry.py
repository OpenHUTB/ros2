#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Field geometry primitives: polygons, obstacles, min-width direction.

田块用顶点序列表示（逆时针），障碍用圆形禁区表示。这里的函数都是无状态
纯函数，供 planner / checker / 测试复用。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CircleObstacle:
    """Circular no-fly obstacle (pole / tree trunk)."""

    x: float
    y: float
    radius: float

    @property
    def center(self) -> np.ndarray:
        return np.array([self.x, self.y])


def polygon_area(vertices: np.ndarray) -> float:
    """Shoelace area, sign-independent."""
    v = np.asarray(vertices, dtype=float)
    x, y = v[:, 0], v[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def point_in_polygon(point, vertices: np.ndarray) -> bool:
    """Ray-casting containment test (boundary treated as inside)."""
    p = np.asarray(point, dtype=float)
    v = np.asarray(vertices, dtype=float)
    n = len(v)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = v[i]
        xj, yj = v[j]
        # 顶点/边上的点直接判内，避免栅格边缘抖动
        cross = (xj - xi) * (p[1] - yi) - (yj - yi) * (p[0] - xi)
        if abs(cross) < 1e-12 and min(xi, xj) - 1e-9 <= p[0] <= max(xi, xj) + 1e-9 \
                and min(yi, yj) - 1e-9 <= p[1] <= max(yi, yj) + 1e-9:
            return True
        if (yi > p[1]) != (yj > p[1]):
            xint = (xj - xi) * (p[1] - yi) / (yj - yi) + xi
            if p[0] < xint:
                inside = not inside
        j = i
    return inside


def min_width_direction(vertices: np.ndarray) -> tuple:
    """Rotating-calipers min-width direction for a convex polygon.

    返回 (方向单位向量 d, 最小宽度 w)。扫描线沿 d 平行布设时条数最少、
    转弯次数最少；对凸多边形，极小宽度必在某条边的方向上取得。
    """
    v = np.asarray(vertices, dtype=float)
    n = len(v)
    best = None
    for i in range(n):
        edge = v[(i + 1) % n] - v[i]
        norm = float(np.linalg.norm(edge))
        if norm < 1e-12:
            continue
        d = edge / norm
        nvec = np.array([-d[1], d[0]])
        proj = v @ nvec
        width = float(proj.max() - proj.min())
        if best is None or width < best[1]:
            best = (d, width)
    return best


def segment_point_distance(seg_p0, seg_p1, points: np.ndarray) -> np.ndarray:
    """Vectorized distance from an array of points to a segment."""
    a = np.asarray(seg_p0, dtype=float)
    b = np.asarray(seg_p1, dtype=float)
    pts = np.atleast_2d(np.asarray(points, dtype=float))
    ab = b - a
    len2 = float(ab @ ab)
    if len2 < 1e-12:
        return np.linalg.norm(pts - a, axis=1)
    t = np.clip((pts - a) @ ab / len2, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return np.linalg.norm(pts - proj, axis=1)


def line_polygon_span(base: np.ndarray, direction: np.ndarray,
                      vertices: np.ndarray) -> tuple:
    """Intersect an infinite line with a convex polygon.

    返回 (t_enter, t_exit)：base + t*direction 与凸多边形交点的参数区间。
    凸多边形与直线至多交出一段区间，取所有边交参数的最小/最大值。
    """
    v = np.asarray(vertices, dtype=float)
    n = len(v)
    ts = []
    for i in range(n):
        p1, p2 = v[i], v[(i + 1) % n]
        e = p2 - p1
        mat = np.array([[direction[0], -e[0]], [direction[1], -e[1]]])
        det = float(np.linalg.det(mat))
        if abs(det) < 1e-12:
            continue
        rhs = p1 - base
        t = (rhs[0] * (-e[1]) - (-e[0]) * rhs[1]) / det
        s = (direction[0] * rhs[1] - direction[1] * rhs[0]) / det
        if -1e-9 <= s <= 1.0 + 1e-9:
            ts.append(t)
    if len(ts) < 2:
        return None
    return float(min(ts)), float(max(ts))
