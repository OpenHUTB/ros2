"""A* path planning, line-of-sight simplification and Catmull-Rom smoothing."""

from __future__ import annotations

import heapq
import math
from typing import List, Sequence, Tuple

import numpy as np

Cell = Tuple[int, int]


class Grid:
    def __init__(self, occupied: np.ndarray, resolution: float = 0.1,
                 origin: Tuple[float, float] = (0.0, 0.0)) -> None:
        self.occupied = occupied.astype(bool)
        self.resolution = resolution
        self.origin = origin
        self.height, self.width = self.occupied.shape

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and not self.occupied[cell[1], cell[0]]

    def to_world(self, cell: Cell) -> Tuple[float, float]:
        return (self.origin[0] + (cell[0] + 0.5) * self.resolution,
                self.origin[1] + (cell[1] + 0.5) * self.resolution)

    def to_cell(self, point: Tuple[float, float]) -> Cell:
        return (math.floor((point[0] - self.origin[0]) / self.resolution),
                math.floor((point[1] - self.origin[1]) / self.resolution))


NEIGHBOURS = [(1, 0), (-1, 0), (0, 1), (0, -1),
              (1, 1), (1, -1), (-1, 1), (-1, -1)]


def astar(grid: Grid, start: Cell, goal: Cell) -> List[Cell]:
    if not grid.free(start) or not grid.free(goal):
        return []
    open_heap = [(0.0, 0.0, start)]
    came_from = {}
    gscore = {start: 0.0}
    closed = set()
    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        for dx, dy in NEIGHBOURS:
            nxt = (current[0] + dx, current[1] + dy)
            if not grid.free(nxt):
                continue
            if dx and dy and (not grid.free((current[0] + dx, current[1])) or
                              not grid.free((current[0], current[1] + dy))):
                continue
            step = math.hypot(dx, dy)
            tentative = g + step
            if tentative < gscore.get(nxt, float("inf")):
                gscore[nxt] = tentative
                h = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(open_heap, (tentative + h, tentative, nxt))
                came_from[nxt] = current
    return []


def segment_free(grid: Grid, a: Sequence[float], b: Sequence[float]) -> bool:
    """Check every cell crossed by segment a-b (0.25-cell sampling)."""
    distance = math.hypot(b[0] - a[0], b[1] - a[1])
    steps = max(1, int(math.ceil(distance / (grid.resolution * 0.25))))
    previous = None
    for i in range(steps + 1):
        t = i / steps
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        cell = grid.to_cell((x, y))
        if not grid.free(cell):
            return False
        if previous is not None and previous != cell:
            # diagonal move must not cut a blocked corner
            if previous[0] != cell[0] and previous[1] != cell[1]:
                if not grid.free((cell[0], previous[1])) or \
                        not grid.free((previous[0], cell[1])):
                    return False
        previous = cell
    return True


def path_free(grid: Grid, points: Sequence[Sequence[float]]) -> bool:
    if len(points) == 0:
        return False
    if not all(grid.free(grid.to_cell(p)) for p in points):
        return False
    return all(segment_free(grid, points[i], points[i + 1])
               for i in range(len(points) - 1))


def line_of_sight(grid: Grid, a: Cell, b: Cell) -> bool:
    return segment_free(grid, grid.to_world(a), grid.to_world(b))


def simplify(grid: Grid, path: Sequence[Cell]) -> List[Cell]:
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


def catmull_rom(points: Sequence[Sequence[float]], samples: int = 12) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return pts.copy()
    padded = np.vstack([pts[0], pts, pts[-1]])
    out = []
    for i in range(len(pts) - 1):
        p0, p1, p2, p3 = padded[i], padded[i + 1], padded[i + 2], padded[i + 3]
        for s in range(samples):
            t = s / samples
            t2, t3 = t * t, t * t * t
            q = 0.5 * ((2 * p1) + (-p0 + p2) * t +
                       (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 +
                       (-p0 + 3 * p1 - 3 * p2 + p3) * t3)
            out.append(q)
    out.append(pts[-1])
    return np.asarray(out)


def curvature(points: Sequence[Sequence[float]]) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if len(pts) < 3:
        return np.zeros(max(0, len(pts)))
    k = []
    for i in range(1, len(pts) - 1):
        a = pts[i] - pts[i - 1]
        b = pts[i + 1] - pts[i]
        cross = abs(a[0] * b[1] - a[1] * b[0])
        ds = 0.5 * (np.linalg.norm(a) + np.linalg.norm(b))
        k.append(cross / max(ds ** 3, 1e-9))
    return np.asarray(k)


def path_length(points: Sequence[Sequence[float]]) -> float:
    pts = np.asarray(points, dtype=float)
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))) if len(pts) > 1 else 0.0


def clearance(grid: Grid, points: Sequence[Sequence[float]]) -> float:
    """Exact distance from points to the nearest occupied cell *area*."""
    occupied = np.argwhere(grid.occupied)
    if occupied.size == 0:
        return float("inf")
    res = grid.resolution
    cx = grid.origin[0] + (occupied[:, 1] + 0.5) * res
    cy = grid.origin[1] + (occupied[:, 0] + 0.5) * res
    best = float("inf")
    for px, py in points:
        dx = np.maximum(np.abs(px - cx) - res / 2.0, 0.0)
        dy = np.maximum(np.abs(py - cy) - res / 2.0, 0.0)
        best = min(best, float(np.min(np.hypot(dx, dy))))
    return best


def _cumulative(points: np.ndarray) -> np.ndarray:
    if len(points) < 2:
        return np.zeros(len(points))
    return np.concatenate([[0.0], np.cumsum(
        np.linalg.norm(np.diff(points, axis=0), axis=1))])


def resample_by_arclength(points: np.ndarray, count: int) -> np.ndarray:
    """Resample a polyline to ``count`` points uniformly by arc length."""
    if len(points) < 2 or count < 2:
        return points.copy()
    s = _cumulative(points)
    if s[-1] < 1e-9:
        return np.repeat(points[:1], count, axis=0)
    targets = np.linspace(0.0, s[-1], count)
    return np.column_stack([np.interp(targets, s, points[:, 0]),
                            np.interp(targets, s, points[:, 1])])


def smooth_path(grid: Grid, path: Sequence[Cell],
                min_clearance: float = 0.1) -> Tuple[np.ndarray, bool]:
    """Return (path, meets_clearance).

    The returned path is always collision-free; the flag additionally says
    whether it meets ``min_clearance``.
    """
    raw_world = np.asarray([grid.to_world(c) for c in path])
    raw_ok = path_free(grid, raw_world)
    if not raw_ok:
        return raw_world, False

    simplified_cells = simplify(grid, path)
    simple_world = np.asarray([grid.to_world(c) for c in simplified_cells])
    simple_ok = path_free(grid, simple_world)
    if not simple_ok:
        return raw_world, False
    simple_safe = clearance(grid, simple_world) >= min_clearance
    candidates = [(simple_world, simple_safe)]
    if len(simple_world) >= 3:
        control = resample_by_arclength(
            simple_world, max(3, int(path_length(simple_world) / 0.5) + 1))
        for alpha in (1.0, 0.75, 0.5, 0.25):
            smooth = catmull_rom(control, samples=12)
            raw_resampled = resample_by_arclength(raw_world, len(smooth))
            blended = alpha * smooth + (1.0 - alpha) * raw_resampled
            if path_free(grid, blended):
                candidates.append((blended, clearance(grid, blended) >= min_clearance))

    best_path, best_safe = None, False
    best_k = float("inf")
    for candidate, meets in candidates:
        k = float(curvature(candidate).max()) if len(candidate) > 2 else 0.0
        if (meets, -k) > (best_safe, -best_k):
            best_path, best_safe, best_k = candidate, meets, k
    if best_path is None:
        return raw_world, False
    return best_path, best_safe


