"""Observed-free occupancy grid and A* path planning primitives."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import heapq
import math

Cell = tuple[int, int]


class PlanFailure(str, Enum):
    START_NOT_FREE = "start_not_free"
    GOAL_NOT_FREE = "goal_not_free"
    NO_PATH = "no_path"


@dataclass(frozen=True)
class PlanResult:
    path: tuple[Cell, ...] = ()
    failure: PlanFailure | None = None

    @property
    def found(self) -> bool:
        return self.failure is None


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

    def integrate_ray(self, start: Cell, end: Cell, *, hit: bool = True) -> None:
        """Mark traversed cells free; mark an obstacle only for a measured hit."""
        if start == end:
            return

        x0, y0 = start
        x1, y1 = end
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        error = dx - dy

        while True:
            cell = (x0, y0)
            at_endpoint = cell == end
            state = self._state(cell)

            if cell != start and state == self.OCCUPIED:
                return
            if at_endpoint:
                self.set_cell(x0, y0, self.OCCUPIED if hit else self.FREE)
                return
            if cell == start or state != self.OCCUPIED:
                self.set_cell(x0, y0, self.FREE)

            twice = 2 * error
            if twice > -dy:
                error -= dy
                x0 += sx
            if twice < dx:
                error += dx
                y0 += sy

    def plan(self, start: Cell, goal: Cell) -> PlanResult:
        """Plan through observed-free cells and report why no route was found."""
        if not self._free(start):
            return PlanResult(failure=PlanFailure.START_NOT_FREE)
        if not self._free(goal):
            return PlanResult(failure=PlanFailure.GOAL_NOT_FREE)

        queue = [(0.0, start)]
        came_from: dict[Cell, Cell | None] = {start: None}
        cost = {start: 0.0}
        while queue:
            _, current = heapq.heappop(queue)
            if current == goal:
                break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = (current[0] + dx, current[1] + dy)
                if not self._free(nxt):
                    continue
                if dx and dy and (
                    not self._free((current[0] + dx, current[1]))
                    or not self._free((current[0], current[1] + dy))
                ):
                    continue
                new_cost = cost[current] + (math.sqrt(2.0) if dx and dy else 1.0)
                if new_cost >= cost.get(nxt, float("inf")):
                    continue
                cost[nxt] = new_cost
                came_from[nxt] = current
                heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(queue, (new_cost + heuristic, nxt))

        if goal not in came_from:
            return PlanResult(failure=PlanFailure.NO_PATH)
        path = []
        node: Cell | None = goal
        while node is not None:
            path.append(node)
            node = came_from[node]
        return PlanResult(tuple(reversed(path)))

    def _state(self, cell: Cell) -> int:
        x, y = cell
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.cells[y][x]
        return self.UNKNOWN

    def _free(self, cell: Cell) -> bool:
        return self._state(cell) == self.FREE
