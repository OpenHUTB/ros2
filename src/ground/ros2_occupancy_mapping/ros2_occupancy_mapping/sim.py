"""2-D simulation and metrics for the occupancy-grid mapping module."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np

try:
    from .mapping import LogOddsGrid
except ImportError:
    from mapping import LogOddsGrid


@dataclass
class Circle:
    x: float
    y: float
    r: float


@dataclass
class Segment:
    x1: float
    y1: float
    x2: float
    y2: float


def default_world() -> Tuple[List[Circle], List[Segment]]:
    circles = [Circle(2.0, 1.5, 0.45), Circle(-2.0, -1.5, 0.45),
               Circle(0.0, 2.6, 0.35)]
    segments = [
        Segment(-6, -4, 6, -4), Segment(-6, 4, 6, 4),
        Segment(-6, -4, -6, 4), Segment(6, -4, 6, 4),
    ]
    return circles, segments


def raycast(pose: Tuple[float, float, float], circles: Sequence[Circle],
            segments: Sequence[Segment], n_beams: int = 180,
            max_range: float = 8.0) -> np.ndarray:
    x, y, yaw = pose
    angles = np.linspace(-math.pi, math.pi, n_beams, endpoint=False) + yaw
    dirs = np.column_stack([np.cos(angles), np.sin(angles)])
    ranges = np.full(n_beams, np.inf, dtype=float)

    for c in circles:
        ocx, ocy = c.x - x, c.y - y
        proj = dirs[:, 0] * ocx + dirs[:, 1] * ocy
        perp2 = ocx * ocx + ocy * ocy - proj * proj
        disc = c.r * c.r - perp2
        hit = (disc >= 0.0) & (proj > 0.0)
        t = np.where(hit, proj - np.sqrt(np.maximum(disc, 0.0)), np.inf)
        ranges = np.minimum(ranges, t)

    for s in segments:
        sx, sy = s.x2 - s.x1, s.y2 - s.y1
        qx, qy = s.x1 - x, s.y1 - y
        denom = dirs[:, 0] * sy - dirs[:, 1] * sx
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (qx * sy - qy * sx) / denom
            u = (qx * dirs[:, 1] - qy * dirs[:, 0]) / denom
        valid = (np.abs(denom) > 1e-12) & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
        ranges = np.minimum(ranges, np.where(valid, t, np.inf))

    return np.where(np.isfinite(ranges), np.minimum(ranges, max_range), max_range)


def true_grid(grid: LogOddsGrid, circles: Sequence[Circle],
              segments: Sequence[Segment]) -> np.ndarray:
    truth = np.zeros((grid.height, grid.width), dtype=bool)
    for gy in range(grid.height):
        for gx in range(grid.width):
            x, y = grid.grid_to_world(gx, gy)
            occupied = any(math.hypot(x - c.x, y - c.y) <= c.r for c in circles)
            if not occupied:
                for s in segments:
                    vx, vy = s.x2 - s.x1, s.y2 - s.y1
                    length2 = vx * vx + vy * vy
                    t = 0.0 if length2 < 1e-9 else max(
                        0.0, min(1.0, ((x - s.x1) * vx + (y - s.y1) * vy) / length2))
                    if math.hypot(x - (s.x1 + t * vx), y - (s.y1 + t * vy)) <= grid.resolution:
                        occupied = True
                        break
            truth[gy, gx] = occupied
    return truth


def evaluate(grid: LogOddsGrid, truth: np.ndarray) -> Dict[str, float]:
    known = grid.known_mask()
    pred = grid.occupied_mask()
    tp = int(np.sum(known & pred & truth))
    fp = int(np.sum(known & pred & ~truth))
    fn = int(np.sum(known & ~pred & truth))
    iou = tp / (tp + fp + fn) if (tp + fp + fn) else 0.0
    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    return {
        "coverage": round(float(known.mean()), 3),
        "occupied_iou": round(iou, 3),
        "occupied_precision": round(precision, 3),
        "occupied_recall": round(recall, 3),
        "free_cells": int(np.sum(known & (grid.as_occupancy() == 0))),
        "occupied_cells": int(np.sum(pred)),
    }


def run_mapping(poses: int = 60, n_beams: int = 180) -> Tuple[LogOddsGrid, Dict[str, float]]:
    circles, segments = default_world()
    grid = LogOddsGrid(-6.5, -4.5, 6.5, 4.5, resolution=0.05)
    for i in range(poses):
        t = 2.0 * math.pi * i / poses
        x, y = 3.0 * math.cos(t), 3.0 * math.sin(t)
        yaw = t + math.pi / 2.0
        ranges = raycast((x, y, yaw), circles, segments, n_beams)
        grid.update_scan((x, y, yaw), ranges, -math.pi,
                         2.0 * math.pi / n_beams, max_range=8.0)
    truth = true_grid(grid, circles, segments)
    return grid, evaluate(grid, truth)


def main() -> None:
    grid, metrics = run_mapping()
    print("map", grid.width, "x", grid.height, "cells")
    for key, value in metrics.items():
        print(f"  {key}: {value}")


if __name__ == "__main__":
    main()
