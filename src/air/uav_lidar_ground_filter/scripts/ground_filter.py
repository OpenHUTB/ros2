"""Robust ground-plane segmentation for world-frame LiDAR clouds."""
from __future__ import annotations

import numpy as np


def fit_plane(points, distance_threshold=0.2, iterations=120, seed=0):
    """RANSAC plane fit; returns (normal, d) with normal·p + d = 0."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < 3:
        raise ValueError("points must be (N>=3, 3)")
    rng = np.random.default_rng(seed)
    best = None
    best_count = -1
    for _ in range(int(iterations)):
        idx = rng.choice(len(pts), 3, replace=False)
        a, b, c = pts[idx]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        d = -float(normal @ a)
        count = int(np.sum(np.abs(pts @ normal + d) < distance_threshold))
        if count > best_count:
            best_count = count
            best = (normal, d)
    if best is None:
        raise ValueError("cannot fit plane")
    return best


def split_cloud(points, normal, d, distance_threshold=0.2):
    """Return (ground_mask, obstacle_mask) as boolean arrays."""
    pts = np.asarray(points)
    dist = np.abs(pts @ np.asarray(normal) + float(d))
    ground = dist < float(distance_threshold)
    return ground, ~ground


def plane_metrics(points, ground_mask):
    pts = np.asarray(points)
    n = len(pts)
    if n == 0:
        return {"points": 0, "ground": 0, "obstacles": 0, "ground_ratio": 0.0}
    ground = int(np.sum(ground_mask))
    return {
        "points": n,
        "ground": ground,
        "obstacles": n - ground,
        "ground_ratio": ground / float(n),
    }
