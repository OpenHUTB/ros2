"""只允许在已观测空闲栅格内通行的占据栅格与 A*。"""

from __future__ import annotations

import heapq
import math


class OccupancyGrid:
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2

    def __init__(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            raise ValueError("grid dimensions must be positive")
        self.width, self.height = width, height
        self.cells = [[self.UNKNOWN for _ in range(width)] for _ in range(height)]

    def set_cell(self, x: int, y: int, state: int) -> None:
        if state not in (self.UNKNOWN, self.FREE, self.OCCUPIED):
            raise ValueError("invalid occupancy state")
        if 0 <= x < self.width and 0 <= y < self.height:
            self.cells[y][x] = state

    def integrate_ray(self, start: tuple[int, int], end: tuple[int, int]) -> None:
        """Bresenham update: traversed cells become free, the endpoint occupied."""
        x0, y0 = start
        x1, y1 = end
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        error = dx - dy
        while (x0, y0) != (x1, y1):
            self.set_cell(x0, y0, self.FREE)
            twice = 2 * error
            if twice > -dy:
                error -= dy
                x0 += sx
            if twice < dx:
                error += dx
                y0 += sy
        self.set_cell(x1, y1, self.OCCUPIED)

    def plan(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        """A* over observed-free cells; unknown cells are conservatively blocked."""
        if not self._free(start) or not self._free(goal):
            return []
        queue = [(0.0, start)]
        came_from = {start: None}
        cost = {start: 0.0}
        while queue:
            _, current = heapq.heappop(queue)
            if current == goal:
                break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = (current[0] + dx, current[1] + dy)
                if not self._free(nxt):
                    continue
                if dx and dy and (not self._free((current[0] + dx, current[1])) or not self._free((current[0], current[1] + dy))):
                    continue
                new_cost = cost[current] + (math.sqrt(2.0) if dx and dy else 1.0)
                if new_cost < cost.get(nxt, float("inf")):
                    cost[nxt] = new_cost
                    came_from[nxt] = current
                    heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                    heapq.heappush(queue, (new_cost + heuristic, nxt))
        if goal not in came_from:
            return []
        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = came_from[node]
        return list(reversed(path))

    def _free(self, cell: tuple[int, int]) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height and self.cells[y][x] == self.FREE
