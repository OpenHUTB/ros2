"""A lightweight occupancy grid and A* global planner.

The grid is built from the same world-frame laser points that feed the DWA
local planner.  Unknown cells are treated as free: this is a *local* rolling
planner, so the robot must be allowed to head into space it has not observed
yet, and the DWA layer keeps it safe when new obstacles appear.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class OccupancyGrid:
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    resolution: float = 0.10
    occupied: Optional[np.ndarray] = None

    def __post_init__(self) -> None:
        self.width = max(1, int(math.ceil((self.x_max - self.x_min) / self.resolution)))
        self.height = max(1, int(math.ceil((self.y_max - self.y_min) / self.resolution)))
        if self.occupied is None:
            self.occupied = np.zeros((self.height, self.width), dtype=bool)
        elif self.occupied.shape != (self.height, self.width):
            raise ValueError("occupied array does not match grid dimensions")

    # -- coordinate helpers -------------------------------------------------
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(math.floor((x - self.x_min) / self.resolution))
        gy = int(math.floor((y - self.y_min) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        return (self.x_min + (gx + 0.5) * self.resolution,
                self.y_min + (gy + 0.5) * self.resolution)

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    # -- construction -------------------------------------------------------
    def add_points(self, points: np.ndarray, inflation: float) -> None:
        """Mark every cell whose centre lies within ``inflation`` of a point."""
        if points.size == 0:
            return
        radius_cells = int(math.ceil(inflation / self.resolution))
        offsets = []
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if (dx * dx + dy * dy) * self.resolution ** 2 <= inflation ** 2 + 1e-9:
                    offsets.append((dx, dy))
        for px, py in points:
            gx, gy = self.world_to_grid(px, py)
            for dx, dy in offsets:
                nx, ny = gx + dx, gy + dy
                if self.in_bounds(nx, ny):
                    self.occupied[ny, nx] = True

    def clear_disc(self, x: float, y: float, radius: float) -> None:
        gx, gy = self.world_to_grid(x, y)
        r = int(math.ceil(radius / self.resolution))
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                nx, ny = gx + dx, gy + dy
                if self.in_bounds(nx, ny) and (dx * dx + dy * dy) * self.resolution ** 2 <= radius ** 2:
                    self.occupied[ny, nx] = False

    def is_free(self, gx: int, gy: int) -> bool:
        return self.in_bounds(gx, gy) and not self.occupied[gy, gx]

    def nearest_free(self, gx: int, gy: int, max_radius: int = 20) -> Optional[Tuple[int, int]]:
        if self.is_free(gx, gy):
            return gx, gy
        for r in range(1, max_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    nx, ny = gx + dx, gy + dy
                    if self.is_free(nx, ny):
                        return nx, ny
        return None


_NEIGHBOURS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
]


def astar(grid: OccupancyGrid, start: Tuple[float, float],
          goal: Tuple[float, float]) -> List[Tuple[float, float]]:
    """8-connected A* on ``grid``.  Returns world-frame waypoints (start..goal)."""
    s = grid.nearest_free(*grid.world_to_grid(*start))
    g = grid.nearest_free(*grid.world_to_grid(*goal))
    if s is None or g is None:
        return []

    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_heap: List[Tuple[float, Tuple[int, int]]] = [(heuristic(s, g), s)]
    came_from = {s: None}
    g_score = {s: 0.0}
    closed = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        if current == g:
            break
        for dx, dy, cost in _NEIGHBOURS:
            nx, ny = current[0] + dx, current[1] + dy
            if not grid.is_free(nx, ny):
                continue
            # Do not cut corners between two occupied cells.
            if dx != 0 and dy != 0:
                if not grid.is_free(current[0] + dx, current[1]) or \
                        not grid.is_free(current[0], current[1] + dy):
                    continue
            neighbour = (nx, ny)
            tentative = g_score[current] + cost
            if tentative < g_score.get(neighbour, float("inf")):
                g_score[neighbour] = tentative
                came_from[neighbour] = current
                heapq.heappush(open_heap, (tentative + heuristic(neighbour, g), neighbour))

    if g not in came_from:
        return []
    cells = []
    node: Optional[Tuple[int, int]] = g
    while node is not None:
        cells.append(node)
        node = came_from[node]
    cells.reverse()
    return [grid.grid_to_world(gx, gy) for gx, gy in cells]


def line_of_sight(grid: OccupancyGrid, a: Tuple[float, float], b: Tuple[float, float]) -> bool:
    """Supercover-ish check that the straight segment a-b stays in free cells."""
    ax, ay = a
    bx, by = b
    distance = math.hypot(bx - ax, by - ay)
    steps = max(2, int(distance / (grid.resolution * 0.5)))
    for i in range(steps + 1):
        t = i / steps
        x = ax + (bx - ax) * t
        y = ay + (by - ay) * t
        if not grid.is_free(*grid.world_to_grid(x, y)):
            return False
    return True


def simplify_path(grid: OccupancyGrid, path: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Greedy string pulling: keep only the waypoints needed for line of sight."""
    if len(path) <= 2:
        return list(path)
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_of_sight(grid, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def point_at_distance(path: Sequence[Tuple[float, float]], origin: Tuple[float, float],
                      distance: float) -> Tuple[float, float]:
    """Return the point ``distance`` metres ahead of ``origin`` along ``path``.

    The robot is first projected onto the closest path segment; the arc length
    is then measured forward from that projection.  Measuring from ``path[0]``
    instead would return a point behind the robot whenever it has already
    advanced along the path.
    """
    if len(path) < 2:
        return path[0] if path else origin
    ox, oy = origin
    best = (float("inf"), 0, 0.0)
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        vx, vy = bx - ax, by - ay
        seg2 = vx * vx + vy * vy
        t = 0.0 if seg2 < 1e-12 else max(0.0, min(1.0, ((ox - ax) * vx + (oy - ay) * vy) / seg2))
        px, py = ax + t * vx, ay + t * vy
        d = math.hypot(ox - px, oy - py)
        if d < best[0]:
            best = (d, i, t)
    _, idx, t = best
    ax, ay = path[idx]
    bx, by = path[idx + 1]
    current = (ax + t * (bx - ax), ay + t * (by - ay))
    remaining = distance
    prev = current
    for point in path[idx + 1:]:
        seg = math.hypot(point[0] - prev[0], point[1] - prev[1])
        if seg >= remaining:
            ratio = remaining / max(seg, 1e-9)
            return (prev[0] + (point[0] - prev[0]) * ratio,
                    prev[1] + (point[1] - prev[1]) * ratio)
        remaining -= seg
        prev = point
    return path[-1]
