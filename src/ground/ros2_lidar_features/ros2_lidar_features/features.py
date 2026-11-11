"""Split-and-merge line extraction and door-gap detection from 2-D scans."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np


@dataclass
class Line:
    p1: Tuple[float, float]
    p2: Tuple[float, float]
    length: float
    angle: float
    residual: float


def points_from_scan(ranges: Sequence[float], angle_min: float,
                     angle_increment: float, max_range: float = 6.0,
                     range_min: float = 0.05) -> np.ndarray:
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(r.size) * angle_increment
    valid = np.isfinite(r) & (r > range_min) & (r < max_range)
    return np.column_stack([r[valid] * np.cos(angles[valid]),
                            r[valid] * np.sin(angles[valid])])


def _fit(points: np.ndarray) -> Tuple[float, float, float, float, float]:
    """Total-least-squares line fit, endpoints projected onto the line."""
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    direction = vt[0]
    projection = centered @ direction
    p1 = centroid + direction * projection.min()
    p2 = centroid + direction * projection.max()
    normal = np.array([-direction[1], direction[0]])
    residual = float(np.mean(np.abs(centered @ normal)))
    return float(p1[0]), float(p1[1]), float(p2[0]), float(p2[1]), residual


def _split(points: np.ndarray, threshold: float, min_points: int,
           out: List[np.ndarray], depth: int = 0, max_depth: int = 12) -> None:
    if len(points) < min_points or depth >= max_depth:
        if len(points) >= min_points:
            out.append(points)
        return
    p1, p2 = points[0], points[-1]
    direction = p2 - p1
    norm = np.linalg.norm(direction)
    if norm < 1e-9:
        out.append(points)
        return
    normal = np.array([-direction[1], direction[0]]) / norm
    distances = np.abs((points - p1) @ normal)
    index = int(np.argmax(distances))
    if distances[index] <= threshold:
        out.append(points)
        return
    _split(points[:index + 1], threshold, min_points, out, depth + 1, max_depth)
    _split(points[index:], threshold, min_points, out, depth + 1, max_depth)


def extract_lines(points: np.ndarray, threshold: float = 0.08,
                  min_points: int = 4, max_gap: float = 0.5,
                  angle_increment: float = 0.0) -> List[Line]:
    if len(points) < min_points:
        return []
    runs = []
    current = [0]
    for i in range(1, len(points)):
        radius = float(np.linalg.norm(points[i]))
        limit = max_gap
        if angle_increment > 0.0:
            limit = max(max_gap, 3.0 * radius * angle_increment)
        if float(np.linalg.norm(points[i] - points[i - 1])) > limit:
            runs.append(points[current])
            current = [i]
        else:
            current.append(i)
    runs.append(points[current])
    segments: List[np.ndarray] = []
    for run in runs:
        _split(run, threshold, min_points, segments)
    lines = []
    for seg in segments:
        if len(seg) < min_points:
            continue
        x1, y1, x2, y2, residual = _fit(seg)
        length = math.hypot(x2 - x1, y2 - y1)
        if length < 0.25:
            continue
        lines.append(Line((x1, y1), (x2, y2), length,
                          math.atan2(y2 - y1, x2 - x1), residual))
    return lines


def detect_doors(lines: Sequence[Line], min_width: float = 0.6,
                 max_width: float = 1.4, angle_tol: float = 0.12,
                 lateral_tol: float = 0.15,
                 min_side_length: float = 0.5) -> List[Tuple[float, float, float, float]]:
    """Return pairs of real endpoints spanning a door gap."""
    doors = []
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            a, b = lines[i], lines[j]
            if a.length < min_side_length or b.length < min_side_length:
                continue
            if abs(math.atan2(math.sin(a.angle - b.angle),
                              math.cos(a.angle - b.angle))) > angle_tol:
                continue
            direction = np.array([math.cos(a.angle), math.sin(a.angle)])
            normal = np.array([-direction[1], direction[0]])
            offset_b1 = abs(float(np.dot(np.array(b.p1) - np.array(a.p1), normal)))
            offset_b2 = abs(float(np.dot(np.array(b.p2) - np.array(a.p1), normal)))
            if max(offset_b1, offset_b2) > lateral_tol:
                continue
            best = None
            for pa in (a.p1, a.p2):
                for pb in (b.p1, b.p2):
                    delta = np.array(pb) - np.array(pa)
                    lateral = abs(float(np.dot(delta, normal)))
                    along = abs(float(np.dot(delta, direction)))
                    if lateral <= lateral_tol and min_width <= along <= max_width:
                        if best is None or along < best[0]:
                            best = (along, pa, pb)
            if best is not None:
                _, p1, p2 = best
                doors.append((float(p1[0]), float(p1[1]),
                              float(p2[0]), float(p2[1])))
    return doors
