"""Log-odds 2D occupancy grid mapping for the parking lot."""

import math
from typing import Dict, Iterable, Tuple

import numpy as np


def bresenham(x0: int, y0: int, x1: int, y1: int) -> Iterable[Tuple[int, int]]:
    """Yield integer grid cells on the line from (x0, y0) to (x1, y1)."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx - dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        doubled = 2 * error
        if doubled > -dy:
            error -= dy
            x += sx
        if doubled < dx:
            error += dx
            y += sy


class OccupancyGridMap:
    """Probabilistic occupancy grid updated from 2D LiDAR scans."""

    def __init__(
        self,
        width: int = 200,
        height: int = 200,
        resolution: float = 0.1,
        origin: Tuple[float, float] = (-10.0, -10.0),
        free_log_odds: float = -0.4,
        occupied_log_odds: float = 0.85,
        clamp: Tuple[float, float] = (-4.0, 4.0),
    ) -> None:
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.free_log_odds = float(free_log_odds)
        self.occupied_log_odds = float(occupied_log_odds)
        self.clamp = (float(clamp[0]), float(clamp[1]))
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.updates = 0

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(round((float(x) - self.origin[0]) / self.resolution))
        gy = int(round((float(y) - self.origin[1]) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = self.origin[0] + int(gx) * self.resolution
        y = self.origin[1] + int(gy) * self.resolution
        return x, y

    def _inside(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def update(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        scan_ranges,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
    ) -> None:
        ranges = np.asarray(scan_ranges, dtype=np.float64).reshape(-1)
        start = self.world_to_grid(robot_x, robot_y)
        if not self._inside(*start):
            return
        for index, distance in enumerate(ranges):
            angle = float(robot_yaw) + float(angle_min) + index * float(angle_increment)
            hit = math.isfinite(distance) and range_min <= distance <= range_max
            ray_length = float(distance) if hit else float(range_max)
            end_x = float(robot_x) + ray_length * math.cos(angle)
            end_y = float(robot_y) + ray_length * math.sin(angle)
            end = self.world_to_grid(end_x, end_y)
            cells = list(bresenham(start[0], start[1], end[0], end[1]))
            for gx, gy in cells[:-1] if hit else cells:
                if self._inside(gx, gy):
                    self.log_odds[gy, gx] += self.free_log_odds
            if hit and self._inside(*end):
                self.log_odds[end[1], end[0]] += self.occupied_log_odds
        np.clip(self.log_odds, self.clamp[0], self.clamp[1], out=self.log_odds)
        self.updates += 1

    def to_occupancy_grid(self, unknown_as_free: bool = True) -> np.ndarray:
        """Return int8 grid: free 0, occupied 100, unknown 0 or -1."""
        grid = np.zeros((self.height, self.width), dtype=np.int8)
        if not unknown_as_free:
            grid[:] = -1
        occupied = self.log_odds > 0.4
        free = self.log_odds < -0.4
        grid[free] = 0
        grid[occupied] = 100
        return grid

    def to_ros_grid(self) -> np.ndarray:
        """Occupancy grid with -1 for unknown cells (ROS convention)."""
        return self.to_occupancy_grid(unknown_as_free=False)

    def statistics(self) -> Dict[str, float]:
        occupied = int(np.count_nonzero(self.log_odds > 0.4))
        free = int(np.count_nonzero(self.log_odds < -0.4))
        known = occupied + free
        total = self.width * self.height
        return {
            'occupied_cells': occupied,
            'free_cells': free,
            'known_cells': known,
            'coverage_pct': 100.0 * known / total,
        }
