#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
geofence_detector/core.py - 无人机禁飞区（电子围栏）检测核心（ENU 坐标系）

支持的禁飞区类型:
    - CylinderGeofence: 圆柱形禁飞区 (常用: 机场净空、活动管制区)
    - RectGeofence:     轴对齐矩形禁飞区 (带高度范围)
检测输出:
    - INSIDE   : 已在禁飞区内 (需立即爬升/脱离)
    - WARNING  : 距边界 < warning_distance (预警)
    - OK       : 安全
"""

import math

OK, WARNING, INSIDE = 'OK', 'WARNING', 'INSIDE'


class CylinderGeofence:
    """圆柱形禁飞区: 平面圆形 + 高度范围 (z 为高度, 向上为正)。"""

    def __init__(self, center, radius, z_min=0.0, z_max=120.0,
                 warning_distance=30.0):
        self.center = (float(center[0]), float(center[1]))
        self.radius = float(radius)
        self.z_min, self.z_max = float(z_min), float(z_max)
        self.warning_distance = float(warning_distance)

    def _in_altitude(self, z):
        return self.z_min <= z <= self.z_max

    def check(self, x, y, z):
        """返回 (状态, 到边界距离 m; 状态为 INSIDE 时为负值表示深入距离)。"""
        dist_xy = math.hypot(x - self.center[0], y - self.center[1])
        surface_dist = dist_xy - self.radius
        if not self._in_altitude(z):
            return OK, float('inf')
        if surface_dist < 0:
            return INSIDE, surface_dist
        if surface_dist < self.warning_distance:
            return WARNING, surface_dist
        return OK, surface_dist


class RectGeofence:
    """轴对齐矩形禁飞区 (x0,x1,y0,y1 + 高度范围)。"""

    def __init__(self, x0, x1, y0, y1, z_min=0.0, z_max=120.0,
                 warning_distance=30.0):
        self.x0, self.x1 = sorted((float(x0), float(x1)))
        self.y0, self.y1 = sorted((float(y0), float(y1)))
        self.z_min, self.z_max = float(z_min), float(z_max)
        self.warning_distance = float(warning_distance)

    def check(self, x, y, z):
        if not (self.z_min <= z <= self.z_max):
            return OK, float('inf')
        dx = max(self.x0 - x, 0.0, x - self.x1)
        dy = max(self.y0 - y, 0.0, y - self.y1)
        if dx == 0 and dy == 0:
            # 在矩形内部: 负值 = 深入距离 (到最近边的距离取负)
            depth = min(x - self.x0, self.x1 - x, y - self.y0, self.y1 - y)
            return INSIDE, -depth
        dist = math.hypot(dx, dy)
        if dist < self.warning_distance:
            return WARNING, dist
        return OK, dist


class GeofenceDetector:
    """多禁飞区管理器: 任一禁飞区触发即取最严格状态。"""

    PRIORITY = {INSIDE: 2, WARNING: 1, OK: 0}

    def __init__(self, fences):
        self.fences = list(fences)

    def check(self, x, y, z):
        best = (OK, float('inf'))
        for f in self.fences:
            status, dist = f.check(x, y, z)
            if self.PRIORITY[status] > self.PRIORITY[best[0]] or \
                    (status == best[0] and dist < best[1]):
                best = (status, dist)
        return best
