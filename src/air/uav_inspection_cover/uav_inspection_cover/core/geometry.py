#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Observe-point generation and segment flyability checks.

杆塔观察点 = 塔位 + 方位角偏移 + 相对塔高的观察高度；可飞性用
"航段到塔身（二维点）的点到线段距离 ≥ 安全距离" 判定，禁飞区
（矩形）按同样的线段-矩形距离判定。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

Rect = Tuple[float, float, float, float]      # (x0, y0, x1, y1)
Waypoint = Tuple[float, float, float]


@dataclass(frozen=True)
class Params:
    """模块级配置对象：算法层与 ROS 节点共用。"""

    offset_m: float = 60.0        # 观察点水平偏移（保证同塔观察点弦距 ≥ 安全距离）
    height_ratio: float = 0.7     # 观察高度 = 塔高比例
    min_height_m: float = 12.0
    max_height_m: float = 60.0
    clearance_m: float = 25.0     # 航段距塔身最小安全距离
    cruise_speed: float = 8.0     # m/s
    hover_time: float = 6.0       # s / 观察点
    cover_radius_m: float = 15.0  # 覆盖判定球半径
    grid_step_m: float = 40.0     # 可飞图采样间距
    zone_margin_m: float = 15.0   # 禁飞区外扩裕度
    legs: int = 4                 # 闭环真飞航点段数
    dt: float = 0.01
    ref_tau: float = 0.5


@dataclass(frozen=True)
class Tower:
    name: str
    x: float
    y: float
    height: float


@dataclass(frozen=True)
class ObservePoint:
    tower: int          # 塔在走廊里的下标
    tag: str            # 例如 T03-A1
    azimuth: float      # 塔位指向观察点的方位角（rad）
    x: float
    y: float
    z: float

    def xyz(self) -> Waypoint:
        return (self.x, self.y, self.z)


# 观察角度数：塔高达到高塔门槛取 3 个，否则 2 个
TALL_HEIGHT_M = 32.0


def n_angles(tower: Tower) -> int:
    """观察角度数由塔高决定，保证每塔 2~3 个。"""
    return 3 if tower.height >= TALL_HEIGHT_M else 2


def observe_points(tower: Tower, tower_idx: int, base_azimuth: float,
                   params: Params) -> List[ObservePoint]:
    """生成单塔观察点集合：方位角均匀铺开，高度取塔高比例并夹到上下限。"""
    out = []
    z = min(max(params.height_ratio * tower.height,
                params.min_height_m), params.max_height_m)
    for k in range(n_angles(tower)):
        az = base_azimuth + 2.0 * math.pi * k / 3.0
        out.append(ObservePoint(
            tower=tower_idx, tag="T%02d-A%d" % (tower_idx + 1, k + 1),
            azimuth=az,
            x=tower.x + params.offset_m * math.cos(az),
            y=tower.y + params.offset_m * math.sin(az), z=z))
    return out


def corridor_viewpoints(towers: Sequence[Tower], params: Params
                        ) -> List[ObservePoint]:
    """整条走廊的观察点：基準方位角取走廊排布方向 + 90°，保证侧向观察。"""
    pts: List[ObservePoint] = []
    for i, tw in enumerate(towers):
        nxt = towers[min(i + 1, len(towers) - 1)]
        prv = towers[max(i - 1, 0)]
        heading = math.atan2(nxt.y - prv.y, nxt.x - prv.x)
        pts.extend(observe_points(tw, i, heading + math.pi / 2.0, params))
    return pts


# ------------------------------------------------------------------ 距离工具
def point_seg_dist(px: float, py: float, ax: float, ay: float,
                   bx: float, by: float) -> float:
    """二维点到线段距离（可飞性判定的核心几何）。"""
    vx, vy = bx - ax, by - ay
    l2 = vx * vx + vy * vy
    if l2 <= 1e-12:
        return math.hypot(px - ax, py - ay)
    u = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / l2))
    return math.hypot(px - (ax + u * vx), py - (ay + u * vy))


def point_rect_dist(px: float, py: float, r: Rect) -> float:
    """二维点到矩形距离（矩形内为 0）。"""
    dx = max(r[0] - px, 0.0, px - r[2])
    dy = max(r[1] - py, 0.0, py - r[3])
    return math.hypot(dx, dy)


def seg_rect_dist(a: Waypoint, b: Waypoint, r: Rect, samples: int = 24) -> float:
    """线段到矩形的近似距离：沿段采样取最小（采样密度远小于间距即可）。"""
    best = min(point_rect_dist(a[0], a[1], r), point_rect_dist(b[0], b[1], r))
    for k in range(1, samples):
        u = k / float(samples)
        best = min(best, point_rect_dist(a[0] + (b[0] - a[0]) * u,
                                         a[1] + (b[1] - a[1]) * u, r))
    return best


# ------------------------------------------------------------------ 可飞性
def leg_clears_towers(a: Waypoint, b: Waypoint, towers: Sequence[Tower],
                      clearance_m: float) -> bool:
    """航段与每座塔身的二维距离都 ≥ 安全距离。"""
    return all(point_seg_dist(t.x, t.y, a[0], a[1], b[0], b[1]) >= clearance_m
               for t in towers)


def leg_clears_zones(a: Waypoint, b: Waypoint, zones: Sequence[Rect],
                     margin_m: float) -> bool:
    """航段与每个禁飞区保持外扩裕度。"""
    return all(seg_rect_dist(a, b, z) >= margin_m for z in zones)


def leg_flyable(a: Waypoint, b: Waypoint, towers: Sequence[Tower],
                zones: Sequence[Rect], params: Params) -> bool:
    """可飞 = 既不侵入塔身安全距离，也不侵入任何禁飞区。"""
    return (leg_clears_towers(a, b, towers, params.clearance_m)
            and leg_clears_zones(a, b, zones, params.zone_margin_m))


def polyline_length(wps: Sequence[Waypoint]) -> float:
    """三维折线总长（m）。"""
    arr = np.asarray(wps, dtype=float)
    if len(arr) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(arr, axis=0), axis=1).sum())
