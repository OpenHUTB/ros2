"""Small 2-D geometry helpers (poses, ray casting, grid conversion)."""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np


def wrap_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (float(a) + math.pi) % (2.0 * math.pi) - math.pi


def rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def body_to_world(pose: Sequence[float], v: Sequence[float]) -> np.ndarray:
    """Rotate a body-frame vector into the world frame."""
    return rot2(pose[2]) @ np.asarray(v, dtype=np.float64)


def world_to_body(pose: Sequence[float], v: Sequence[float]) -> np.ndarray:
    return rot2(-pose[2]) @ np.asarray(v, dtype=np.float64)


def ray_rect(origin: np.ndarray, direction: np.ndarray, rect: Sequence[float]) -> float:
    """Ray / axis-aligned rectangle intersection (slab method).

    ``rect`` = (cx, cy, half_w, half_h).  Returns distance or ``inf``.
    """
    cx, cy, hw, hh = rect
    ox, oy = origin[0] - cx, origin[1] - cy
    dx, dy = direction[0], direction[1]
    tmin, tmax = -1e9, 1e9
    for o, d, h in ((ox, dx, hw), (oy, dy, hh)):
        if abs(d) < 1e-9:
            if abs(o) > h:
                return math.inf
        else:
            t1 = (-h - o) / d
            t2 = (h - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return math.inf
    if tmax < 0.0:
        return math.inf
    return max(tmin, 0.0)


def ray_circle(origin: np.ndarray, direction: np.ndarray,
               circle: Sequence[float]) -> float:
    """Ray / circle intersection.  ``circle`` = (cx, cy, r)."""
    cx, cy, r = circle
    ox, oy = origin[0] - cx, origin[1] - cy
    b = ox * direction[0] + oy * direction[1]
    c = ox * ox + oy * oy - r * r
    disc = b * b - c
    if disc < 0.0:
        return math.inf
    sq = math.sqrt(disc)
    t0, t1 = -b - sq, -b + sq
    if t1 < 0.0:
        return math.inf
    return max(t0, 0.0)


def raycast_obstacles(origin: np.ndarray, directions: np.ndarray,
                      rects: Iterable[Sequence[float]],
                      circles: Iterable[Sequence[float]],
                      max_range: float) -> np.ndarray:
    """Cast a fan of rays against rect/circle obstacles.

    ``directions`` has shape (N, 2) and unit length.
    Returns an array of ranges with shape (N,).
    """
    n = len(directions)
    out = np.full(n, max_range, dtype=np.float64)
    rects = list(rects)
    circles = list(circles)
    for i in range(n):
        d = directions[i]
        best = max_range
        for rect in rects:
            t = ray_rect(origin, d, rect)
            if t < best:
                best = t
        for circle in circles:
            t = ray_circle(origin, d, circle)
            if t < best:
                best = t
        out[i] = best
    return out


def world_to_grid(x: float, y: float, resolution: float,
                  width: int, height: int) -> Tuple[int, int]:
    """World (m) -> grid (col, row).  Row 0 is the minimum-y edge."""
    c = int(math.floor(x / resolution))
    r = int(math.floor(y / resolution))
    return max(0, min(width - 1, c)), max(0, min(height - 1, r))


def grid_to_world(c: int, r: int, resolution: float) -> Tuple[float, float]:
    return (c + 0.5) * resolution, (r + 0.5) * resolution


def downsample_bool(grid: np.ndarray, factor: int) -> np.ndarray:
    """OR-downsample a boolean grid by an integer factor."""
    h, w = grid.shape
    h2, w2 = h // factor, w // factor
    if h2 == 0 or w2 == 0:
        return grid.copy()
    trimmed = grid[: h2 * factor, : w2 * factor]
    return trimmed.reshape(h2, factor, w2, factor).any(axis=(1, 3))


def inflate_bool(grid: np.ndarray, cells: int) -> np.ndarray:
    """Dilate a boolean grid with a square structuring element."""
    if cells <= 0:
        return grid.copy()
    padded = np.pad(grid, cells, mode="constant", constant_values=False)
    out = np.zeros_like(grid)
    h, w = grid.shape
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            out |= padded[cells + dy: cells + dy + h, cells + dx: cells + dx + w]
    return out


def bresenham(c0: int, r0: int, c1: int, r1: int) -> List[Tuple[int, int]]:
    """Integer line between two grid cells (inclusive)."""
    points: List[Tuple[int, int]] = []
    dc, dr = abs(c1 - c0), abs(r1 - r0)
    sc = 1 if c0 < c1 else -1
    sr = 1 if r0 < r1 else -1
    err = dc - dr
    c, r = c0, r0
    while True:
        points.append((c, r))
        if c == c1 and r == r1:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr
    return points


def polyline_length(points: Sequence[Sequence[float]]) -> float:
    total = 0.0
    for a, b in zip(points[:-1], points[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total
