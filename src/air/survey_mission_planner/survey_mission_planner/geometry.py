"""Polygon, rotation and coverage helpers for survey planning."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np


def polygon_area(polygon: np.ndarray) -> float:
    total = 0.0
    for index, current in enumerate(polygon):
        nxt = polygon[(index + 1) % len(polygon)]
        total += current[0] * nxt[1] - nxt[0] * current[1]
    return abs(total) * 0.5


def point_in_polygon(point: Sequence[float], polygon: np.ndarray) -> bool:
    x, y = float(point[0]), float(point[1])
    inside = False
    for index, current in enumerate(polygon):
        nxt = polygon[(index + 1) % len(polygon)]
        if _point_on_segment(x, y, current[0], current[1], nxt[0], nxt[1]):
            return True
        if (current[1] > y) != (nxt[1] > y):
            crossing = (nxt[0] - current[0]) * (y - current[1]) / (nxt[1] - current[1]) + current[0]
            if x < crossing:
                inside = not inside
    return inside


def _point_on_segment(x: float, y: float, x1: float, y1: float, x2: float, y2: float) -> bool:
    cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
    if abs(cross) > 1e-7:
        return False
    return (
        min(x1, x2) - 1e-7 <= x <= max(x1, x2) + 1e-7
        and min(y1, y2) - 1e-7 <= y <= max(y1, y2) + 1e-7
    )


def rotate_points(points: np.ndarray, angle_rad: float) -> np.ndarray:
    rotation = np.array(
        [[math.cos(angle_rad), -math.sin(angle_rad)], [math.sin(angle_rad), math.cos(angle_rad)]]
    )
    return np.asarray(points, dtype=float) @ rotation.T


def principal_angle(polygon: np.ndarray) -> float:
    """Angle of the dominant axis of the polygon vertices."""
    centered = polygon - polygon.mean(axis=0)
    covariance = centered.T @ centered
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    direction = eigenvectors[:, int(np.argmax(eigenvalues))]
    return float(math.atan2(direction[1], direction[0]))


def line_intervals(y: float, polygon: np.ndarray) -> list[tuple[float, float]]:
    """Intersect a horizontal line with a polygon and return x intervals."""
    intersections: list[float] = []
    for index, current in enumerate(polygon):
        nxt = polygon[(index + 1) % len(polygon)]
        if (current[1] > y) == (nxt[1] > y):
            continue
        x = current[0] + (y - current[1]) * (nxt[0] - current[0]) / (nxt[1] - current[1])
        intersections.append(float(x))
    intersections.sort()
    intervals = []
    for index in range(0, len(intersections) - 1, 2):
        start, end = intersections[index], intersections[index + 1]
        if end - start > 1e-6:
            intervals.append((start, end))
    return intervals


def sample_coverage(
    polygon: np.ndarray,
    line_ys: Sequence[float],
    half_width_m: float,
    resolution_m: float = 2.0,
) -> float:
    minimum = polygon.min(axis=0)
    maximum = polygon.max(axis=0)
    xs = np.arange(minimum[0], maximum[0] + resolution_m, resolution_m)
    ys = np.arange(minimum[1], maximum[1] + resolution_m, resolution_m)
    inside = 0
    covered = 0
    for y in ys:
        for x in xs:
            if not point_in_polygon((x, y), polygon):
                continue
            inside += 1
            if min(abs(float(y) - float(line_y)) for line_y in line_ys) <= half_width_m:
                covered += 1
    return covered / inside if inside else 0.0


def polyline_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
