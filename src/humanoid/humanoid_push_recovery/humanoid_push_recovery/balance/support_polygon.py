# -*- coding: utf-8 -*-
"""支撑多边形：由着地脚构建，并回答平衡控制所需的几何查询。

改写自（Apache License 2.0）：
    PrageethM1702/Bipedal-Walking-and-Push-Recovery
    https://github.com/PrageethM1702/Bipedal-Walking-and-Push-Recovery
    （原文件 ``balance/support_polygon.py``，Copyright (c) the original authors）

本站改动：
  * 顶点不再由「脚部矩形半长/半宽近似」生成，而是直接取本模型脚掌 geom
    （``foot_geom_l`` / ``foot_geom_r``）的 8 个底面角点，见 ``sim_core.foot_corners()``；
  * 原类的几何方法同时以模块级函数 ``convex_hull`` / ``point_in_polygon`` /
    ``distance_to_boundary`` 暴露，供仿真核心与评价脚本直接复用；
  * ``signed_distance`` 对边界外的点返回负值（原实现同样如此），语义在文档中明确为
    「稳定裕度」。

几何约定：返回的顶点按逆时针排列；点在多边形内为正、在外为负。
"""

import numpy as np


def convex_hull(points):
    """二维凸包（Andrew 单调链），返回逆时针顶点序列 (N×2)。"""
    pts = sorted({(round(float(p[0]), 9), round(float(p[1]), 9)) for p in points})
    if len(pts) <= 2:
        return np.array(pts, dtype=float)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1], dtype=float)


def point_in_polygon(point, polygon):
    """点是否在（凸）多边形内，含边界。"""
    poly = np.asarray(polygon, dtype=float)
    if len(poly) < 3:
        return False
    px, py = float(point[0]), float(point[1])
    sign = None
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
        if abs(cross) < 1e-12:
            continue
        s = cross > 0
        if sign is None:
            sign = s
        elif s != sign:
            return False
    return True


def distance_to_boundary(point, polygon):
    """点到多边形边界的最近距离：内部为正、外部为负（m）。"""
    poly = np.asarray(polygon, dtype=float)
    if len(poly) < 3:
        return float("nan")
    p = np.asarray(point, dtype=float)[:2]
    best = float("inf")
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        ab = b - a
        length2 = float(np.dot(ab, ab))
        if length2 < 1e-12:
            continue
        t = float(np.clip(np.dot(p - a, ab) / length2, 0.0, 1.0))
        best = min(best, float(np.linalg.norm(p - (a + t * ab))))
    return best if point_in_polygon(p, poly) else -best


class SupportPolygon:
    """由着地脚构成的（凸）支撑多边形。

    本模型直接传入脚底角点：``SupportPolygon(corners)``；也提供
    ``SupportPolygon.from_corners(...)`` 与 ``from_robot(sim)`` 便于从仿真对象构建。
    """

    def __init__(self, corners=None, vertices=None):
        if vertices is not None:
            self.vertices = np.asarray(vertices, dtype=float).tolist()
        elif corners is not None and len(corners):
            self.vertices = _convex_hull_list(corners)
        else:
            self.vertices = []

    @classmethod
    def from_robot(cls, sim):
        """从 ``BalanceSim`` 实例构建（取脚掌底面角点）。"""
        return cls(corners=sim.foot_corners()[:, :2])

    def centroid(self):
        if not self.vertices:
            return np.zeros(2)
        return np.mean(np.array(self.vertices), dtype=float), 0.0

    def contains(self, point):
        return point_in_polygon(point, self.vertices)

    def signed_distance(self, point):
        return distance_to_boundary(point, self.vertices)

    def margin(self, point):
        """点的平衡裕度（等价于 ``signed_distance``）。"""
        return self.signed_distance(point)

    def as_array(self):
        return np.asarray(self.vertices, dtype=float)


def _convex_hull_list(points):
    return convex_hull(points).tolist()
