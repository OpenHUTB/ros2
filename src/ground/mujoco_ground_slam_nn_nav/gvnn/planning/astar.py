"""A* global planning on a 2-D occupancy grid (NumPy + heapq only)."""

from __future__ import annotations

import heapq
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from ..geometry import bresenham, inflate, line_of_sight

Point = Tuple[float, float]


class GridPlanner:
    """Inflated-grid A* planner.

    Parameters
    ----------
    width, height:
        World size in metres.
    resolution:
        Planning cell size (metres).
    inflation:
        Obstacle inflation radius (metres).  Should be >= robot radius.
    unknown_cost:
        Extra traversal cost for cells whose occupancy is unknown.  Unknown
        cells stay traversable so that navigation can start before SLAM has
        seen the whole map.
    """

    def __init__(self, width: float, height: float, resolution: float = 0.10,
                 inflation: float = 0.42, unknown_cost: float = 2.0):
        self.width = float(width)
        self.height = float(height)
        self.resolution = float(resolution)
        self.inflation = float(inflation)
        self.unknown_cost = float(unknown_cost)
        self.w = int(round(width / resolution))
        self.h = int(round(height / resolution))
        self.occupied = np.zeros((self.h, self.w), dtype=bool)
        self.unknown = np.zeros((self.h, self.w), dtype=bool)
        self.blocked = np.zeros((self.h, self.w), dtype=bool)
        self.traversable = np.ones((self.h, self.w), dtype=bool)
        self.version = 0

    # ------------------------------------------------------------------ maps
    def set_map(self, occupied: np.ndarray, unknown: Optional[np.ndarray] = None,
                inflate_obstacles: bool = True) -> None:
        """Set a planning map.

        ``occupied`` and ``unknown`` may be at any resolution; they are
        nearest-neighbour resampled to the planner grid.
        """
        occ = self._resample(occupied)
        self.occupied = occ
        if unknown is None:
            self.unknown = np.zeros_like(occ)
        else:
            self.unknown = self._resample(unknown)
        if inflate_obstacles:
            self.blocked = inflate(self.occupied, int(round(self.inflation / self.resolution)))
        else:
            self.blocked = self.occupied.copy()
        self.traversable = ~self.blocked
        self.version += 1

    def _resample(self, grid: np.ndarray) -> np.ndarray:
        grid = np.asarray(grid)
        if grid.shape == (self.h, self.w):
            return grid.astype(bool, copy=True)
        gh, gw = grid.shape
        ys = np.clip((np.arange(self.h) * gh / self.h).astype(np.int32), 0, gh - 1)
        xs = np.clip((np.arange(self.w) * gw / self.w).astype(np.int32), 0, gw - 1)
        return grid[np.ix_(ys, xs)].astype(bool, copy=True)

    # ------------------------------------------------------------------ A*
    def _nearest_free(self, cell: Tuple[int, int], radius: int = 12) -> Optional[Tuple[int, int]]:
        cx, cy = cell
        if self._free(cx, cy):
            return cx, cy
        best = None
        best_d = 1e9
        for r in range(1, radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    x, y = cx + dx, cy + dy
                    if self._free(x, y):
                        d = dx * dx + dy * dy
                        if d < best_d:
                            best_d = d
                            best = (x, y)
        return best

    def _free(self, x: int, y: int) -> bool:
        return 0 <= x < self.w and 0 <= y < self.h and self.traversable[y, x]

    def _cell(self, p: Sequence[float]) -> Tuple[int, int]:
        return int(p[0] / self.resolution), int(p[1] / self.resolution)

    def plan(self, start: Sequence[float], goal: Sequence[float],
             max_expansions: int = 200000) -> Optional[List[Point]]:
        s = self._nearest_free(self._cell(start))
        g = self._nearest_free(self._cell(goal))
        if s is None or g is None:
            return None
        sx, sy = s
        gx, gy = g
        if (sx, sy) == (gx, gy):
            return [(float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))]

        w, h = self.w, self.h
        g_score = np.full(h * w, np.inf, dtype=np.float64)
        parent = np.full(h * w, -1, dtype=np.int32)
        start_i = sy * w + sx
        goal_i = gy * w + gx
        g_score[start_i] = 0.0

        def heuristic(ix: int) -> float:
            x, y = ix % w, ix // w
            dx, dy = abs(x - gx), abs(y - gy)
            return (dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy)

        open_heap = [(heuristic(start_i), 0.0, start_i)]
        closed = np.zeros(h * w, dtype=bool)
        expansions = 0
        neigh = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
                 (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)))
        while open_heap:
            _, cost, idx = heapq.heappop(open_heap)
            if closed[idx]:
                continue
            if idx == goal_i:
                return self._reconstruct(parent, idx, start, goal)
            closed[idx] = True
            expansions += 1
            if expansions > max_expansions:
                break
            x, y = idx % w, idx // w
            for dx, dy, step in neigh:
                nx, ny = x + dx, y + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                if not self.traversable[ny, nx]:
                    continue
                nidx = ny * w + nx
                if closed[nidx]:
                    continue
                extra = self.unknown_cost if self.unknown[ny, nx] else 0.0
                tentative = cost + step + extra
                if tentative + 1e-12 < g_score[nidx]:
                    g_score[nidx] = tentative
                    parent[nidx] = idx
                    heapq.heappush(open_heap, (tentative + heuristic(nidx), tentative, nidx))
        return None

    def _reconstruct(self, parent: np.ndarray, idx: int,
                     start: Sequence[float], goal: Sequence[float]) -> List[Point]:
        w = self.w
        cells: List[Tuple[int, int]] = []
        while idx >= 0:
            cells.append((idx % w, idx // w))
            idx = int(parent[idx])
        cells.reverse()
        raw = [((x + 0.5) * self.resolution, (y + 0.5) * self.resolution) for x, y in cells]
        raw[0] = (float(start[0]), float(start[1]))
        if len(raw) > 1:
            raw[-1] = (float(goal[0]), float(goal[1]))
        return self.simplify(raw)

    def simplify(self, path: Sequence[Point]) -> List[Point]:
        """Greedy line-of-sight shortcutting on the inflated grid."""
        if len(path) <= 2:
            return [(float(x), float(y)) for x, y in path]
        out: List[Point] = [(float(path[0][0]), float(path[0][1]))]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if line_of_sight(~self.traversable, path[i], path[j], self.resolution, 0):
                    break
                j -= 1
            out.append((float(path[j][0]), float(path[j][1])))
            i = j
        # Remove a duplicated endpoint if A* landed on a snapped cell.
        if len(out) >= 2 and math.hypot(out[-1][0] - path[-1][0], out[-1][1] - path[-1][1]) > 1e-6:
            out.append((float(path[-1][0]), float(path[-1][1])))
        return out

    def path_cost(self, path: Sequence[Point]) -> float:
        return float(sum(math.hypot(path[i + 1][0] - path[i][0],
                                    path[i + 1][1] - path[i][1])
                         for i in range(len(path) - 1)))
