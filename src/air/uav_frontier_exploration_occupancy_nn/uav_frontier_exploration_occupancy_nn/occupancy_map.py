#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""占据栅格地图：未知/自由/占据三态，射线探测与前沿提取。

地图不依赖 ROS，可在离线仿真中直接维护；传感器沿射线标记自由格，
命中障碍时在端点标记占据格。
"""

from __future__ import annotations

import math

import numpy as np

UNKNOWN = -1
FREE = 0
OCCUPIED = 1


class OccupancyMap:
    """二维占据栅格地图。"""

    def __init__(self, width: int = 40, height: int = 30, resolution: float = 0.25):
        self.w = width
        self.h = height
        self.res = resolution
        self.grid = np.full((height, width), UNKNOWN, dtype=np.int8)

    # ---- 坐标换算 ----
    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        c = int(x / self.res)
        r = int(y / self.res)
        return min(max(c, 0), self.w - 1), min(max(r, 0), self.h - 1)

    def cell_to_world(self, c: int, r: int) -> tuple[float, float]:
        return (c + 0.5) * self.res, (r + 0.5) * self.res

    def in_bounds(self, c: int, r: int) -> bool:
        return 0 <= c < self.w and 0 <= r < self.h

    # ---- 障碍与探测 ----
    def add_obstacle_rect(self, x0: float, y0: float, x1: float, y1: float) -> None:
        c0, r0 = self.world_to_cell(x0, y0)
        c1, r1 = self.world_to_cell(x1, y1)
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                self.grid[r, c] = OCCUPIED

    def raycast(self, ox: float, oy: float, angle: float,
                max_range: float) -> bool:
        """沿单条射线探测，沿途标记自由格，命中障碍返回 True。"""
        dx, dy = math.cos(angle), math.sin(angle)
        step = self.res * 0.5
        t = step
        while t <= max_range:
            x, y = ox + dx * t, oy + dy * t
            c, r = self.world_to_cell(x, y)
            if not self.in_bounds(c, r):
                return False
            if self.grid[r, c] == OCCUPIED:
                self.grid[r, c] = OCCUPIED
                return True
            self.grid[r, c] = FREE
            t += step
        return False

    def sense(self, ox: float, oy: float, angles: list[float],
              max_range: float) -> None:
        for angle in angles:
            self.raycast(ox, oy, angle, max_range)

    # ---- 前沿与统计 ----
    def frontier_cells(self) -> list[tuple[int, int]]:
        """自由格且八邻域内存在未知格，即前沿格。"""
        frontiers: list[tuple[int, int]] = []
        for r in range(self.h):
            for c in range(self.w):
                if self.grid[r, c] != FREE:
                    continue
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        nc, nr = c + dc, r + dr
                        if self.in_bounds(nc, nr) and self.grid[nr, nc] == UNKNOWN:
                            frontiers.append((c, r))
                            break
                    else:
                        continue
                    break
        return frontiers

    def explored_free_count(self) -> int:
        return int(np.count_nonzero(self.grid == FREE))

    def unknown_count(self) -> int:
        return int(np.count_nonzero(self.grid == UNKNOWN))
