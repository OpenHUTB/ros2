"""Obstacle inflation for 2-D occupancy grids (Nav2-style cost layer)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np


@dataclass
class InflationResult:
    costs: np.ndarray      # int8, -1 unknown, 0..99 inflated, 100 lethal
    distance: np.ndarray   # metres to nearest lethal cell
    lethal: np.ndarray


def _dt1d(f: np.ndarray) -> np.ndarray:
    """Exact 1-D squared distance transform (Felzenszwalb & Huttenlocher)."""
    n = len(f)
    d = np.empty(n, dtype=float)
    v = np.zeros(n, dtype=int)
    z = np.empty(n + 1, dtype=float)
    k = 0
    v[0] = 0
    z[0] = -np.inf
    z[1] = np.inf
    for q in range(1, n):
        s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
        while s <= z[k]:
            k -= 1
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = np.inf
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        d[q] = (q - v[k]) ** 2 + f[v[k]]
    return d


def distance_transform(lethal: np.ndarray, resolution: float) -> np.ndarray:
    """Exact Euclidean distance (metres) to the nearest lethal cell."""
    if not np.any(lethal):
        return np.full(lethal.shape, np.inf)
    inf = 1e12
    f = np.where(lethal, 0.0, inf)
    for y in range(f.shape[0]):
        f[y, :] = _dt1d(f[y, :])
    for x in range(f.shape[1]):
        f[:, x] = _dt1d(f[:, x])
    return np.sqrt(np.maximum(f, 0.0)) * resolution


def inflate(occupancy: np.ndarray, resolution: float = 0.05,
            inflation_radius: float = 0.5, inscribed_radius: float = 0.2,
            cost_scaling: float = 3.0) -> InflationResult:
    occ = np.asarray(occupancy)
    lethal = occ >= 50
    unknown = occ < 0
    dist = distance_transform(lethal, resolution)

    costs = np.zeros(occ.shape, dtype=np.int16)
    costs[unknown] = -1
    inside = (~lethal) & (~unknown) & (dist <= inflation_radius)
    if inscribed_radius > 0:
        hard = inside & (dist <= inscribed_radius)
        costs[hard] = 99
    soft = inside & ~hard if inscribed_radius > 0 else inside
    scale = 98.0 * np.exp(-cost_scaling * np.maximum(
        dist - inscribed_radius, 0.0) / max(inflation_radius - inscribed_radius, 1e-6))
    costs[soft] = np.clip(scale[soft], 0, 98).astype(np.int16)
    costs[lethal] = 100
    costs = np.clip(costs, -1, 100).astype(np.int8)
    return InflationResult(costs, dist, lethal)


def analytic_distance(lethal: np.ndarray, resolution: float) -> np.ndarray:
    ys, xs = np.nonzero(lethal)
    out = np.full(lethal.shape, np.inf)
    for y in range(lethal.shape[0]):
        for x in range(lethal.shape[1]):
            if len(xs) == 0:
                out[y, x] = 0.0
                continue
            out[y, x] = np.min(np.hypot(xs - x, ys - y)) * resolution
    return out


def summary(result: InflationResult, resolution: float = 0.05) -> Dict[str, float]:
    exact = analytic_distance(result.lethal, resolution)
    known = ~result.lethal & np.isfinite(exact)
    if not np.any(known):
        return {
            "max_distance_error_m": 0.0,
            "mean_distance_error_m": 0.0,
            "lethal_cells": int(result.lethal.sum()),
            "inflated_cells": int(((result.costs > 0) & (result.costs < 100)).sum()),
            "free_cells": int((result.costs == 0).sum()),
        }
    err = np.abs(result.distance - exact)
    return {
        "max_distance_error_m": round(float(err[known].max()), 4),
        "mean_distance_error_m": round(float(err[known].mean()), 4),
        "lethal_cells": int(result.lethal.sum()),
        "inflated_cells": int(((result.costs > 0) & (result.costs < 100)).sum()),
        "free_cells": int((result.costs == 0).sum()),
    }


