"""Geometry helpers shared by simulation, SLAM, planning and rendering."""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]


def wrap_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    out = (a + math.pi) % (2.0 * math.pi) - math.pi
    if out <= -math.pi + 1e-12:
        out += 2.0 * math.pi
    return out


def rot2(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def transform_points(points: np.ndarray, pose: Sequence[float]) -> np.ndarray:
    """Transform Nx2 points from a local frame into the frame of ``pose``."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    c, s = math.cos(yaw), math.sin(yaw)
    out = np.empty_like(pts)
    out[:, 0] = c * pts[:, 0] - s * pts[:, 1] + x
    out[:, 1] = s * pts[:, 0] + c * pts[:, 1] + y
    return out


def inverse_transform_points(points: np.ndarray, pose: Sequence[float]) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    c, s = math.cos(yaw), math.sin(yaw)
    dx = pts[:, 0] - x
    dy = pts[:, 1] - y
    out = np.empty_like(pts)
    out[:, 0] = c * dx + s * dy
    out[:, 1] = -s * dx + c * dy
    return out


def pose_to_matrix(pose: Sequence[float]) -> np.ndarray:
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def matrix_to_pose(mat: np.ndarray) -> np.ndarray:
    return np.array(
        [mat[0, 2], mat[1, 2], math.atan2(mat[1, 0], mat[0, 0])],
        dtype=np.float64,
    )


def relative_pose(base: Sequence[float], target: Sequence[float]) -> np.ndarray:
    """Pose of ``target`` expressed in the local frame of ``base``."""
    return matrix_to_pose(np.linalg.inv(pose_to_matrix(base)) @ pose_to_matrix(target))


def yaw_to_quat(yaw: float) -> Tuple[float, float]:
    return math.cos(yaw / 2.0), math.sin(yaw / 2.0)


def bresenham(x0: int, y0: int, x1: int, y1: int) -> Iterable[Tuple[int, int]]:
    """Integer Bresenham line, inclusive of both endpoints."""
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = int(x0), int(y0)
    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


def distance_point_segment(px: float, py: float,
                           x1: float, y1: float,
                           x2: float, y2: float) -> float:
    vx, vy = x2 - x1, y2 - y1
    wx, wy = px - x1, py - y1
    denom = vx * vx + vy * vy
    if denom < 1e-12:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
    return math.hypot(px - (x1 + t * vx), py - (y1 + t * vy))


def line_of_sight(occupied: np.ndarray, p0: Point, p1: Point,
                  resolution: float, clearance_cells: int = 0) -> bool:
    """True when the straight segment p0-p1 does not cross occupied cells."""
    h, w = occupied.shape
    x0, y0 = int(p0[0] / resolution), int(p0[1] / resolution)
    x1, y1 = int(p1[0] / resolution), int(p1[1] / resolution)
    if not (0 <= x0 < w and 0 <= y0 < h and 0 <= x1 < w and 0 <= y1 < h):
        return False
    for cx, cy in bresenham(x0, y0, x1, y1):
        for oy in range(-clearance_cells, clearance_cells + 1):
            for ox in range(-clearance_cells, clearance_cells + 1):
                nx, ny = cx + ox, cy + oy
                if 0 <= nx < w and 0 <= ny < h and occupied[ny, nx]:
                    return False
    return True


def inflate(occupied: np.ndarray, radius_cells: int) -> np.ndarray:
    """Binary dilation with a square structuring element (numpy only)."""
    if radius_cells <= 0:
        return occupied.copy()
    out = occupied.copy()
    for _ in range(radius_cells):
        padded = np.pad(out, 1, mode="constant", constant_values=False)
        neigh = np.zeros_like(out)
        for oy in (-1, 0, 1):
            for ox in (-1, 0, 1):
                if ox == 0 and oy == 0:
                    continue
                neigh |= padded[1 + oy:1 + oy + out.shape[0],
                                1 + ox:1 + ox + out.shape[1]]
        out = out | neigh
    return out


def downsample_bool(grid: np.ndarray, factor: int) -> np.ndarray:
    h, w = grid.shape
    h2, w2 = h // factor, w // factor
    trimmed = grid[:h2 * factor, :w2 * factor]
    return trimmed.reshape(h2, factor, w2, factor).any(axis=(1, 3))


def path_length(points: Sequence[Sequence[float]]) -> float:
    if len(points) < 2:
        return 0.0
    return float(sum(math.hypot(points[i + 1][0] - points[i][0],
                                points[i + 1][1] - points[i][1])
                     for i in range(len(points) - 1)))


def resample_polyline(points: Sequence[Sequence[float]], step: float) -> List[Point]:
    """Resample a polyline at approximately ``step`` metres."""
    if len(points) < 2:
        return [(float(points[0][0]), float(points[0][1]))] if points else []
    out: List[Point] = [(float(points[0][0]), float(points[0][1]))]
    carry = 0.0
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-9:
            continue
        d = step - carry
        while d <= seg:
            t = d / seg
            out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
            d += step
        carry = seg - (d - step)
    if out[-1] != (float(points[-1][0]), float(points[-1][1])):
        out.append((float(points[-1][0]), float(points[-1][1])))
    return out
