"""Workspace path planning for the musculoskeletal arm.

The arm base is fixed at the origin, so ``planning`` happens directly in the
2-D end-effector workspace (x forward, y up):

* :class:`GridPlanner` -- 8-connected A* on the workspace occupancy grid
  (obstacles / shelf / crates blocked), circular inflation by at least the
  end-effector radius, octile heuristic, no corner cutting and line-of-sight
  smoothing.
* :func:`plan_sequence` -- multi-target planning (``draw`` scene): one A* path
  per consecutive target pair.

Poses are end-effector positions ``(x, y)`` in metres; the corresponding
template pose is ``(x, y, 0, 0)``.
"""

from __future__ import annotations

import heapq
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import bresenham, wrap_angle

__all__ = ["GridPlanner", "scene_planning_grid", "plan_sequence",
           "densify_path", "lookahead_point", "polyline_length"]

_SQRT2 = math.sqrt(2.0)
_MOVES = (
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, _SQRT2), (1, -1, _SQRT2), (-1, 1, _SQRT2), (-1, -1, _SQRT2),
)


def _workspace_shape(resolution: float) -> Tuple[int, int]:
    x0, x1 = C.WORK_X
    y0, y1 = C.WORK_Y
    return (int(round((y1 - y0) / resolution)),
            int(round((x1 - x0) / resolution)))


def scene_planning_grid(scene, resolution: float = C.PLAN_RESOLUTION
                        ) -> np.ndarray:
    """Blocked workspace grid for a :class:`scenes.Scene`.

    Uses :meth:`scenes.Scene.coarse_map` when it is sampled at exactly
    ``resolution``; otherwise rebuilds it from :meth:`scenes.Scene.occupancy`.
    """
    expected = _workspace_shape(resolution)
    cm = getattr(scene, "coarse_map", None)
    if cm is not None:
        grid = np.asarray(scene.coarse_map(resolution), dtype=bool)
        if grid.shape == expected:
            return grid
    return np.asarray(scene.occupancy(resolution), dtype=bool)


def _inflate_circular(grid: np.ndarray, radius_cells: float) -> np.ndarray:
    """Dilate a boolean grid with a circular structuring element."""
    if radius_cells <= 0.0:
        return grid.copy()
    k = int(math.ceil(radius_cells))
    h, w = grid.shape
    out = grid.copy()
    padded = np.pad(grid, k, mode="constant", constant_values=False)
    r2 = radius_cells * radius_cells + 1e-9
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            if dx * dx + dy * dy <= r2:
                out |= padded[k + dy: k + dy + h, k + dx: k + dx + w]
    return out


class GridPlanner:
    """8-connected A* workspace planner (no corner cutting, EE-radius inflation).

    Parameters
    ----------
    occupancy_bool:
        2-D boolean grid, ``True`` = blocked (build it with
        :func:`scene_planning_grid` for a :class:`scenes.Scene`).
    resolution:
        Grid cell size in metres, default :data:`config.PLAN_RESOLUTION`.
    inflate_cells:
        Obstacle inflation radius in cells.  Defaults to
        :data:`config.INFLATION_RADIUS` (0.05 m), which is larger than the
        end-effector radius (:data:`config.EE_RADIUS`, 0.03 m).
    origin:
        World ``(x, y)`` coordinate of the lower-left cell corner, default
        ``(config.WORK_X[0], config.WORK_Y[0])``.
    """

    def __init__(self, occupancy_bool: np.ndarray,
                 resolution: float = C.PLAN_RESOLUTION,
                 inflate_cells: Optional[float] = None,
                 origin: Tuple[float, float] = (C.WORK_X[0], C.WORK_Y[0])
                 ) -> None:
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        base = np.asarray(occupancy_bool, dtype=bool)
        if base.ndim != 2:
            raise ValueError("occupancy_bool must be a 2-D array")
        self.base = base
        self.height, self.width = base.shape
        self.ee_radius = float(C.EE_RADIUS)
        if inflate_cells is None:
            self.inflate_radius_m = max(float(C.EE_RADIUS),
                                        float(C.INFLATION_RADIUS))
            self.inflate_radius_cells = self.inflate_radius_m / self.resolution
        else:
            self.inflate_radius_cells = max(0.0, float(inflate_cells))
            self.inflate_radius_m = self.inflate_radius_cells * self.resolution
        self.inflate_cells = int(math.ceil(self.inflate_radius_cells))
        self.grid = _inflate_circular(base, self.inflate_radius_cells)

    # ------------------------------------------------------------ conversion
    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        c = int(math.floor((float(x) - self.origin[0]) / self.resolution))
        r = int(math.floor((float(y) - self.origin[1]) / self.resolution))
        return min(max(c, 0), self.width - 1), min(max(r, 0), self.height - 1)

    def cell_to_world(self, c: int, r: int) -> Tuple[float, float]:
        return (self.origin[0] + (float(c) + 0.5) * self.resolution,
                self.origin[1] + (float(r) + 0.5) * self.resolution)

    def is_free(self, x: float, y: float) -> bool:
        """True when the inflated workspace grid is free at ``(x, y)``."""
        c, r = self.world_to_cell(x, y)
        return not bool(self.grid[r, c])

    def clearance(self, x: float, y: float, max_cells: int = 12) -> float:
        """Distance (m) from ``(x, y)`` to the nearest raw blocked cell."""
        c, r = self.world_to_cell(x, y)
        if self.base[r, c]:
            return 0.0
        h, w = self.height, self.width
        for rad in range(1, int(max_cells) + 1):
            r0, r1 = max(0, r - rad), min(h, r + rad + 1)
            c0, c1 = max(0, c - rad), min(w, c + rad + 1)
            window = self.base[r0:r1, c0:c1]
            if window.any():
                rr, cc = np.nonzero(window)
                d = np.hypot((cc + c0 - c) * self.resolution,
                             (rr + r0 - r) * self.resolution)
                return float(d.min())
        return float(max_cells) * self.resolution

    # ---------------------------------------------------------------- search
    def _snap(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """Nearest free cell to world ``(x, y)`` (``(col, row)``) or ``None``."""
        c0, r0 = self.world_to_cell(x, y)
        if not self.grid[r0, c0]:
            return c0, r0
        limit = max(self.width, self.height)
        window = 2
        while window <= limit:
            r_lo, r_hi = max(0, r0 - window), min(self.height - 1, r0 + window)
            c_lo, c_hi = max(0, c0 - window), min(self.width - 1, c0 + window)
            free = ~self.grid[r_lo: r_hi + 1, c_lo: c_hi + 1]
            if np.any(free):
                rr, cc = np.nonzero(free)
                rr = rr + r_lo
                cc = cc + c_lo
                d = (cc - c0) ** 2 + (rr - r0) ** 2
                k = int(np.argmin(d))
                return int(cc[k]), int(rr[k])
            window *= 2
        return None

    def nearest_free(self, x: float, y: float) -> Tuple[float, float]:
        """World coordinates of the nearest free cell to ``(x, y)``."""
        cell = self._snap(float(x), float(y))
        if cell is None:
            return float(x), float(y)
        return self.cell_to_world(*cell)

    @staticmethod
    def _heuristic(c: int, r: int, gc: int, gr: int) -> float:
        dc, dr = abs(gc - c), abs(gr - r)
        return (dc + dr) + (_SQRT2 - 2.0) * min(dc, dr)

    def plan(self, start_xy: Sequence[float],
             goal_xy: Sequence[float]) -> Optional[List[Tuple[float, float]]]:
        """A* path ``[(x, y), ...]`` from start to goal (world coords) or ``None``."""
        start = self._snap(float(start_xy[0]), float(start_xy[1]))
        goal = self._snap(float(goal_xy[0]), float(goal_xy[1]))
        if start is None or goal is None:
            return None
        sc, sr = start
        gc, gr = goal
        if (sc, sr) == (gc, gr):
            return [self.cell_to_world(sc, sr)]
        w, h = self.width, self.height
        start_idx = sr * w + sc
        goal_idx = gr * w + gc

        g = np.full(h * w, np.inf, dtype=np.float64)
        parent = np.full(h * w, -1, dtype=np.int64)
        closed = np.zeros(h * w, dtype=bool)
        g[start_idx] = 0.0
        heap: List[Tuple[float, int, int]] = [
            (self._heuristic(sc, sr, gc, gr), 0, start_idx)]
        counter = 1
        grid = self.grid
        found = False
        while heap:
            _, _, cur = heapq.heappop(heap)
            if closed[cur]:
                continue
            closed[cur] = True
            if cur == goal_idx:
                found = True
                break
            r, c = divmod(cur, w)
            base_cost = g[cur]
            for dc, dr, cost in _MOVES:
                nc, nr = c + dc, r + dr
                if nc < 0 or nc >= w or nr < 0 or nr >= h:
                    continue
                if grid[nr, nc]:
                    continue
                if dc != 0 and dr != 0 and (grid[r, nc] or grid[nr, c]):
                    continue  # no corner cutting
                nxt = nr * w + nc
                if closed[nxt]:
                    continue
                ng = base_cost + cost
                if ng < g[nxt] - 1e-9:
                    g[nxt] = ng
                    parent[nxt] = cur
                    heapq.heappush(heap, (ng + self._heuristic(nc, nr, gc, gr),
                                          counter, nxt))
                    counter += 1
        if not found:
            return None
        cells: List[Tuple[int, int]] = []
        cur = goal_idx
        while cur != -1:
            r, c = divmod(cur, w)
            cells.append((c, r))
            cur = int(parent[cur])
        cells.reverse()
        pts = [self.cell_to_world(c, r) for c, r in cells]
        return self._simplify(pts)

    # ------------------------------------------------------------ smoothing
    def _line_free(self, a: Sequence[float], b: Sequence[float]) -> bool:
        """Line-of-sight check on the inflated grid (supercover, no corner cut)."""
        c0, r0 = self.world_to_cell(a[0], a[1])
        c1, r1 = self.world_to_cell(b[0], b[1])
        prev = None
        for c, r in bresenham(c0, r0, c1, r1):
            if self.grid[r, c]:
                return False
            if prev is not None:
                dc = c - prev[0]
                dr = r - prev[1]
                if dc != 0 and dr != 0:
                    if self.grid[prev[1], c] or self.grid[r, prev[0]]:
                        return False
            prev = (c, r)
        return True

    def _simplify(self, pts: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Greedy line-of-sight shortcutting of a dense grid path."""
        if len(pts) <= 2:
            return list(pts)
        out = [pts[0]]
        i = 0
        n = len(pts)
        while i < n - 1:
            j = n - 1
            while j > i + 1 and not self._line_free(pts[i], pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    # --------------------------------------------------------------- metrics
    @staticmethod
    def path_length(path: Sequence[Sequence[float]]) -> float:
        total = 0.0
        for a, b in zip(path[:-1], path[1:]):
            total += math.hypot(b[0] - a[0], b[1] - a[1])
        return float(total)


# ------------------------------------------------------------------ helpers
def polyline_length(path: Sequence[Sequence[float]]) -> float:
    """Length of a polyline (m)."""
    return GridPlanner.path_length(path)


def densify_path(path: Sequence[Sequence[float]],
                 step: float = 0.02) -> List[Tuple[float, float]]:
    """Resample a (simplified) polyline at roughly ``step`` metres."""
    if path is None or len(path) == 0:
        return []
    out: List[Tuple[float, float]] = [(float(path[0][0]), float(path[0][1]))]
    for a, b in zip(path[:-1], path[1:]):
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        dist = math.hypot(bx - ax, by - ay)
        n = max(1, int(math.ceil(dist / max(step, 1e-4))))
        for k in range(1, n + 1):
            t = k / float(n)
            out.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return out


def lookahead_point(path: Sequence[Sequence[float]], x: float, y: float,
                    lookahead: float) -> Tuple[float, float]:
    """First point on the polyline at ``lookahead`` m beyond the projection.

    Falls back to the last point when the remaining path is shorter.
    """
    if path is None or len(path) == 0:
        return float(x), float(y)
    pts = path
    n = len(pts)
    if n == 1:
        return float(pts[0][0]), float(pts[0][1])
    best = (math.inf, 0, 0.0)
    for j in range(n - 1):
        ax, ay = float(pts[j][0]), float(pts[j][1])
        bx, by = float(pts[j + 1][0]), float(pts[j + 1][1])
        abx, aby = bx - ax, by - ay
        seg2 = abx * abx + aby * aby
        t = 0.0 if seg2 <= 1e-12 else \
            max(0.0, min(1.0, ((x - ax) * abx + (y - ay) * aby) / seg2))
        px, py = ax + t * abx, ay + t * aby
        d = math.hypot(px - x, py - y)
        if d < best[0]:
            best = (d, j, t)
    _, j0, t0 = best
    ax, ay = float(pts[j0][0]), float(pts[j0][1])
    bx, by = float(pts[j0 + 1][0]), float(pts[j0 + 1][1])
    seg_len = math.hypot(bx - ax, by - ay)
    if seg_len <= 1e-9:
        return bx, by
    remain = (1.0 - t0) * seg_len
    if remain >= lookahead:
        ratio = lookahead / seg_len
        return ax + (t0 + ratio) * (bx - ax), ay + (t0 + ratio) * (by - ay)
    for j in range(j0 + 1, n - 1):
        sx, sy = float(pts[j][0]), float(pts[j][1])
        nx, ny = float(pts[j + 1][0]), float(pts[j + 1][1])
        seg_len = math.hypot(nx - sx, ny - sy)
        if remain + seg_len >= lookahead:
            ratio = (lookahead - remain) / max(seg_len, 1e-9)
            return sx + ratio * (nx - sx), sy + ratio * (ny - sy)
        remain += seg_len
    return float(pts[-1][0]), float(pts[-1][1])


def plan_sequence(start_xy: Sequence[float],
                  targets: Sequence[Sequence[float]],
                  planner: GridPlanner) -> List[List[Tuple[float, float]]]:
    """Plan one A* segment per consecutive target (``draw`` scene).

    Returns a list of paths; each path starts at the end of the previous one.
    Segments that cannot be planned are returned as straight lines so the
    mission still has a reference to follow.
    """
    paths: List[List[Tuple[float, float]]] = []
    current = (float(start_xy[0]), float(start_xy[1]))
    for target in targets:
        goal = (float(target[0]), float(target[1]))
        path = planner.plan(current, goal)
        if path is None:
            path = [current, goal]
        path = densify_path(path)
        paths.append(path)
        current = goal
    return paths
