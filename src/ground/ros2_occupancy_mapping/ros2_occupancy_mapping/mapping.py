"""Log-odds occupancy grid mapping from 2-D laser scans."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

FREE = 0
OCCUPIED = 100
UNKNOWN = -1


def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """Integer line cells from (x0,y0) to (x1,y1), both inclusive."""
    cells = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return cells


@dataclass
class LogOddsGrid:
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    resolution: float = 0.05
    l_free: float = -0.40
    l_occupied: float = 0.85
    l_min: float = -5.0
    l_max: float = 5.0

    def __post_init__(self) -> None:
        self.width = int(math.ceil((self.x_max - self.x_min) / self.resolution))
        self.height = int(math.ceil((self.y_max - self.y_min) / self.resolution))
        self.log_odds = np.zeros((self.height, self.width), dtype=float)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        return (int((x - self.x_min) / self.resolution),
                int((y - self.y_min) / self.resolution))

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        return (self.x_min + (gx + 0.5) * self.resolution,
                self.y_min + (gy + 0.5) * self.resolution)

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def update_scan(self, pose: Tuple[float, float, float],
                    ranges: Sequence[float], angle_min: float,
                    angle_increment: float, max_range: float = 8.0) -> None:
        x, y, yaw = pose
        gx0, gy0 = self.world_to_grid(x, y)
        if not self.in_bounds(gx0, gy0):
            return
        for i, r in enumerate(np.asarray(ranges, dtype=float)):
            if not math.isfinite(r) or r <= 0.0:
                continue
            hit = r < max_range
            angle = yaw + angle_min + i * angle_increment
            ex = x + r * math.cos(angle)
            ey = y + r * math.sin(angle)
            gx1, gy1 = self.world_to_grid(ex, ey)
            cells = bresenham(gx0, gy0, gx1, gy1)
            for gx, gy in cells[:-1]:
                if self.in_bounds(gx, gy):
                    self.log_odds[gy, gx] = max(
                        self.l_min, self.log_odds[gy, gx] + self.l_free)
            if hit:
                gx, gy = cells[-1]
                if self.in_bounds(gx, gy):
                    self.log_odds[gy, gx] = min(
                        self.l_max, self.log_odds[gy, gx] + self.l_occupied)

    def as_occupancy(self) -> np.ndarray:
        data = np.full((self.height, self.width), UNKNOWN, dtype=np.int8)
        data[self.log_odds < -0.2] = FREE
        data[self.log_odds > 0.2] = OCCUPIED
        return data

    def occupied_mask(self) -> np.ndarray:
        return self.as_occupancy() == OCCUPIED

    def known_mask(self) -> np.ndarray:
        return self.as_occupancy() != UNKNOWN
