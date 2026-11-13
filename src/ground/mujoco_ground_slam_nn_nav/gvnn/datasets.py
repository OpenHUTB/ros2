"""Dataset helpers: free-space sampling, local maps and sensor corruption."""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

from .config import LIDAR_RANGE_MAX, MAP_RESOLUTION
from .geometry import inflate

# 8 compass directions, counter-clockwise from +x (robot forward).
DIRECTION_BINS = 8
DIRECTION_ANGLES = np.arange(DIRECTION_BINS) * (2.0 * math.pi / DIRECTION_BINS)
DIRECTION_ANGLES = (DIRECTION_ANGLES + math.pi) % (2.0 * math.pi) - math.pi

_FREE_CACHE = {}


def _free_cells(occupancy: np.ndarray, clearance: float) -> np.ndarray:
    """Cached inflated free-space mask (ground-truth maps are static)."""
    key = (id(occupancy), occupancy.shape, float(clearance))
    cached = _FREE_CACHE.get(key)
    if cached is None or cached[0] is not occupancy:
        cells = int(round(clearance / MAP_RESOLUTION))
        cached = (occupancy, ~inflate(occupancy, cells))
        _FREE_CACHE[key] = cached
    return cached[1]


def sample_free_pose(scene, occupancy: np.ndarray, rng: np.random.Generator,
                     clearance: float = 0.65,
                     avoid: Optional[Sequence[float]] = None,
                     min_avoid_dist: float = 4.0) -> Tuple[float, float, float]:
    """Sample a collision-free pose with ``clearance`` metres from obstacles."""
    free = _free_cells(occupancy, clearance)
    ys, xs = np.nonzero(free)
    order = rng.permutation(len(xs))
    for k in order:
        x = (xs[k] + 0.5) * MAP_RESOLUTION
        y = (ys[k] + 0.5) * MAP_RESOLUTION
        if x < 0.8 or y < 0.8 or x > scene.width - 0.8 or y > scene.height - 0.8:
            continue
        if avoid is not None and math.hypot(x - avoid[0], y - avoid[1]) < min_avoid_dist:
            continue
        return x, y, float(rng.uniform(-math.pi, math.pi))
    # Fallback to the scene start if the map is unexpectedly full.
    return scene.start[0], scene.start[1], scene.start_yaw


def random_goal(scene, occupancy: np.ndarray, rng: np.random.Generator,
                start: Sequence[float], min_distance: float = 7.0,
                clearance: float = 0.75) -> Tuple[float, float]:
    free = _free_cells(occupancy, clearance)
    ys, xs = np.nonzero(free)
    for _ in range(400):
        k = int(rng.integers(len(xs)))
        x = (xs[k] + 0.5) * MAP_RESOLUTION
        y = (ys[k] + 0.5) * MAP_RESOLUTION
        if x < 1.0 or y < 1.0 or x > scene.width - 1.0 or y > scene.height - 1.0:
            continue
        if math.hypot(x - start[0], y - start[1]) >= min_distance:
            return x, y
    return scene.goal


def local_patch(occupied: np.ndarray, unknown: np.ndarray,
                pose: Sequence[float], size: int = 21,
                resolution: float = 0.2) -> np.ndarray:
    """Robot-aligned local map patch (forward = up), values 0/0.5/1."""
    n = int(size)
    ys, xs = np.mgrid[0:n, 0:n]
    lx = (xs - (n - 1) / 2.0) * resolution
    ly = ((n - 1) / 2.0 - ys) * resolution
    c, s = math.cos(pose[2]), math.sin(pose[2])
    wx = c * lx - s * ly + pose[0]
    wy = s * lx + c * ly + pose[1]
    gx = np.clip((wx / MAP_RESOLUTION).astype(np.int32), 0, occupied.shape[1] - 1)
    gy = np.clip((wy / MAP_RESOLUTION).astype(np.int32), 0, occupied.shape[0] - 1)
    patch = np.zeros((n, n), dtype=np.float32)
    patch[unknown[gy, gx]] = 0.5
    patch[occupied[gy, gx]] = 1.0
    return patch


def goal_polar(pose: Sequence[float], goal: Sequence[float],
               scale: float = 30.0) -> np.ndarray:
    dx, dy = goal[0] - pose[0], goal[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    d = math.hypot(dx, dy)
    return np.array([lx / max(1e-6, d), ly / max(1e-6, d), d / scale], dtype=np.float32)


def point_along_path(path: Sequence[Sequence[float]], pose: Sequence[float],
                     distance: float = 2.5) -> Tuple[float, float]:
    pts = np.asarray(path, dtype=np.float64)
    if len(pts) < 2:
        return float(pts[-1, 0]), float(pts[-1, 1])
    d = np.sum((pts - np.array(pose[:2], dtype=np.float64)) ** 2, axis=1)
    i = int(np.argmin(d))
    remain = float(distance)
    for j in range(i, len(pts) - 1):
        seg = float(np.linalg.norm(pts[j + 1] - pts[j]))
        if seg < 1e-9:
            continue
        if remain <= seg:
            t = remain / seg
            p = pts[j] + t * (pts[j + 1] - pts[j])
            return float(p[0]), float(p[1])
        remain -= seg
    return float(pts[-1, 0]), float(pts[-1, 1])


def direction_label(pose: Sequence[float], path: Sequence[Sequence[float]],
                    distance: float = 2.5) -> int:
    px, py = point_along_path(path, pose, distance)
    dx, dy = px - pose[0], py - pose[1]
    a = math.atan2(dy, dx) - pose[2]
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    idx = int(round(a / (2.0 * math.pi / DIRECTION_BINS))) % DIRECTION_BINS
    return idx


def corrupt_scan(clean: Sequence[float], rng: np.random.Generator,
                 severity: float = 1.0) -> np.ndarray:
    r = np.asarray(clean, dtype=np.float64).copy()
    n = len(r)
    r += rng.normal(0.0, (0.006 + 0.0025 * r) * severity)
    drop = rng.random(n) < 0.012 * severity
    r[drop] = LIDAR_RANGE_MAX
    outlier = rng.random(n) < 0.006 * severity
    r[outlier] = rng.uniform(0.15, LIDAR_RANGE_MAX, int(outlier.sum()))
    # Occasional blocked beam (dust / transparent surface).
    blocked = rng.random(n) < 0.004 * severity
    r[blocked] = rng.uniform(0.10, 0.60, int(blocked.sum()))
    return np.clip(r, 0.08, LIDAR_RANGE_MAX).astype(np.float32)
