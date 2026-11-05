#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2D 小车目标追踪：几何与碰撞小工具（纯函数，无第三方依赖）。"""
from __future__ import annotations

import math


def normalize_angle(a: float) -> float:
    """把角度归一化到 (-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def wrap_deg(a: float) -> float:
    return math.degrees(normalize_angle(a))


def dist(x1, y1, x2, y2) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def point_circle_gap(px, py, cx, cy, r) -> float:
    """点到圆面的**有符号间隙**：圆外为正（离表面多远），圆内为负。"""
    return dist(px, py, cx, cy) - float(r)


def collides(px, py, car_radius, obstacles) -> bool:
    """车（半径 car_radius 的圆）是否与任一障碍相交。"""
    for (ox, oy, r) in obstacles:
        if dist(px, py, ox, oy) < float(r) + float(car_radius):
            return True
    return False


def nearest_obstacle(px, py, obstacles):
    """返回最近的障碍 (ox, oy, r, gap)；无障碍返回 None。"""
    best = None
    for (ox, oy, r) in obstacles:
        gap = point_circle_gap(px, py, ox, oy, r)
        if best is None or gap < best[3]:
            best = (float(ox), float(oy), float(r), gap)
    return best


def segment_clearance(x1, y1, x2, y2, obstacles) -> float:
    """线段到各障碍圆的最小有符号间隙（用于"路径是否擦着障碍走"的检查）。"""
    if not obstacles:
        return float("inf")
    best = float("inf")
    for (ox, oy, r) in obstacles:
        best = min(best, _segment_circle_gap(x1, y1, x2, y2, ox, oy, r))
    return best


def _segment_circle_gap(x1, y1, x2, y2, cx, cy, r) -> float:
    dx, dy = x2 - x1, y2 - y1
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-12:
        return point_circle_gap(x1, y1, cx, cy, r)
    t = ((cx - x1) * dx + (cy - y1) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    px, py = x1 + t * dx, y1 + t * dy
    return point_circle_gap(px, py, cx, cy, r)


def block_fraction(px, py, nx, ny, car_radius, obstacles, max_step=None) -> float:
    """沿 (px,py)→(nx,ny) 走，返回**第一个碰撞点之前**的安全比例 t ∈ [0,1]。

    为什么不能用二分：碰撞判定沿路径是**非单调**的（障碍只占中间一小段），
    二分会直接跳过障碍收敛到 t=1（实测 500 px 一步能把车"穿"到障碍另一侧）。
    这里沿路径按不超过一个车半径的间距采样，找到第一个碰撞样本后回退一格。
    """
    step = float(max_step) if max_step else float(car_radius)
    total = dist(px, py, nx, ny)
    if total < 1e-9:
        return 1.0
    n = max(1, int(math.ceil(total / max(step, 1e-6))))
    for i in range(1, n + 1):
        t = i / n
        if collides(px + (nx - px) * t, py + (ny - py) * t, car_radius, obstacles):
            return max(0.0, (i - 1) / n)
    return 1.0


def sweep_hits(px, py, nx, ny, car_radius, obstacles, max_step=None):
    """从 (px,py) 走到 (nx,ny) 的**整段**是否撞到障碍。

    原实现只检查终点：一步跨过障碍（高速/大步长）时会"穿模"。这里沿线段采样，
    采样间距不超过 ``max_step``（默认一个车半径），保证不会整段跳过。
    """
    return block_fraction(px, py, nx, ny, car_radius, obstacles, max_step) < 1.0
