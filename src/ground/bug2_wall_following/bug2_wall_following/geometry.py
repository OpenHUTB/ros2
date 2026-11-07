"""与 Bug2 相关的几何工具：点到线段距离、线段是否相交、最近障碍点。"""

import math


def dist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def point_segment_dist(p, a, b):
    """点 p 到线段 ab 的最近距离与最近点。"""
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return dist(p, a), a
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    proj = (ax + t * dx, ay + t * dy)
    return dist(p, proj), proj


def circle_obstacle_nearest(p, obs):
    """返回机器人到圆形障碍轮廓的最近点与距离（障碍表面外）。"""
    cx, cy, r = obs
    d = dist(p, (cx, cy))
    if d < 1e-6:
        return (cx + r, cy), r
    ux, uy = (p[0] - cx) / d, (p[1] - cy) / d
    surface = (cx + ux * r, cy + uy * r)
    return surface, d - r


def segments_cross(p1, p2, q1, q2):
    """线段 p1p2 与 q1q2 是否相交（用于判断是否回到 Lm 线）。"""
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    d1 = cross(q1, q2, p1)
    d2 = cross(q1, q2, p2)
    d3 = cross(p1, p2, q1)
    d4 = cross(p1, p2, q2)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))
