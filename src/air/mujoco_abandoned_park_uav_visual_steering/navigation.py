"""Sensor-to-map updates and deterministic A* navigation helpers."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
from typing import Iterable


Cell = tuple[int, int]


@dataclass
class OccupancyGrid:
    """Small online occupancy grid used by the simulated range sensor."""

    width: int = 32
    height: int = 32
    occupied: set[Cell] | None = None

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("grid dimensions must be positive")
        if self.occupied is None:
            self.occupied = set()

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def mark_sensor_ray(self, origin: Cell, hits: Iterable[Cell]) -> None:
        """Integrate range hits; cells before each hit become known free."""
        for hit in hits:
            if not self.in_bounds(hit):
                continue
            x0, y0 = origin
            x1, y1 = hit
            dx, dy = abs(x1 - x0), abs(y1 - y0)
            sx = 1 if x0 < x1 else -1
            sy = 1 if y0 < y1 else -1
            err = dx - dy
            while (x0, y0) != (x1, y1):
                self.occupied.discard((x0, y0))
                double = 2 * err
                if double > -dy:
                    err -= dy
                    x0 += sx
                if double < dx:
                    err += dx
                    y0 += sy
            self.occupied.add(hit)

    def neighbors(self, cell: Cell) -> Iterable[Cell]:
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            candidate = (cell[0] + dx, cell[1] + dy)
            if self.in_bounds(candidate) and candidate not in self.occupied:
                yield candidate


def astar(grid: OccupancyGrid, start: Cell, goal: Cell) -> list[Cell]:
    """Return a four-connected shortest path, or an empty path if blocked."""
    if not grid.in_bounds(start) or not grid.in_bounds(goal):
        raise ValueError("start and goal must be inside the grid")
    if start in grid.occupied or goal in grid.occupied:
        return []
    frontier: list[tuple[int, Cell]] = [(0, start)]
    came_from: dict[Cell, Cell | None] = {start: None}
    cost: dict[Cell, int] = {start: 0}
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            path = [current]
            while came_from[path[-1]] is not None:
                path.append(came_from[path[-1]])
            return list(reversed(path))
        for nxt in grid.neighbors(current):
            new_cost = cost[current] + 1
            if nxt not in cost or new_cost < cost[nxt]:
                cost[nxt] = new_cost
                priority = new_cost + abs(goal[0] - nxt[0]) + abs(goal[1] - nxt[1])
                heapq.heappush(frontier, (priority, nxt))
                came_from[nxt] = current
    return []


def keyboard_command(key: str) -> tuple[float, float]:
    """Map a key to (forward_speed, normalized_steering)."""
    commands = {
        "w": (0.8, 0.0), "a": (0.4, -1.0), "d": (0.4, 1.0),
        "s": (-0.4, 0.0), "x": (0.0, 0.0),
    }
    try:
        return commands[key.lower()]
    except KeyError as exc:
        raise ValueError("use W/A/S/D for motion or X to stop") from exc
