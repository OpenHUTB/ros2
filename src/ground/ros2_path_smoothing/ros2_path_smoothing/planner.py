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
        return (int((point[0] - self.origin[0]) / self.resolution),
                int((point[1] - self.origin[1]) / self.resolution))


NEIGHBOURS = [(1, 0), (-1, 0), (0, 1), (0, -1),
              (1, 1), (1, -1), (-1, 1), (-1, -1)]


def astar(grid: Grid, start: Cell, goal: Cell) -> List[Cell]:
    if not grid.free(start) or not grid.free(goal):
        return []
    open_heap = [(0.0, 0.0, start)]
    came_from = {}
    gscore = {start: 0.0}
    while open_heap:
        _, g, current = heapq.heappop(open_heap)
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


def line_of_sight(grid: Grid, a: Cell, b: Cell) -> bool:
    x0, y0 = a
    x1, y1 = b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        if not grid.free((x0, y0)):
            return False
        if x0 == x1 and y0 == y1:
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


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
    occupied = np.argwhere(grid.occupied)
    if occupied.size == 0:
        return float("inf")
    best = float("inf")
    for point in points:
        cell = grid.to_cell((point[0], point[1]))
        d = np.hypot(occupied[:, 1] - cell[0], occupied[:, 0] - cell[1]).min()
        best = min(best, float(d) * grid.resolution)
    return best


def resample_uniform(points: np.ndarray, spacing: float = 0.5) -> np.ndarray:
    if len(points) < 2:
        return points
    diffs = np.diff(points, axis=0)
    seg = np.linalg.norm(diffs, axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(seg)])
    total = cumulative[-1]
    targets = np.arange(0.0, total, spacing)
    targets = np.append(targets, total)
    xs = np.interp(targets, cumulative, points[:, 0])
    ys = np.interp(targets, cumulative, points[:, 1])
    return np.column_stack([xs, ys])


def smooth_path(grid: Grid, path: Sequence[Cell],
                min_clearance: float = 0.1) -> Tuple[np.ndarray, bool]:
    """Return (smoothed world points, collision_free)."""
    simplified = simplify(grid, path)
    world = np.asarray([grid.to_world(c) for c in simplified])
    if len(world) < 3:
        return world, True
    world = resample_uniform(world, 0.5)
    raw_world = np.asarray([grid.to_world(c) for c in path])
    for alpha in (1.0, 0.75, 0.5, 0.25):
        smooth = catmull_rom(world, samples=12)
        blended = alpha * smooth + (1.0 - alpha) * _resample(raw_world, len(smooth))
        if all(grid.free(grid.to_cell(p)) for p in blended) and \
                clearance(grid, blended) >= min_clearance:
            return blended, True
    return world, False


def _resample(points: np.ndarray, count: int) -> np.ndarray:
    if len(points) == count:
        return points
    src = np.linspace(0.0, 1.0, len(points))
    dst = np.linspace(0.0, 1.0, count)
    return np.column_stack([np.interp(dst, src, points[:, 0]),
                            np.interp(dst, src, points[:, 1])])




