"""Curvature-aware speed profiling along a 2-D path."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np


def curvature(points: Sequence[Sequence[float]]) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if len(pts) < 3:
        return np.zeros(len(pts))
    k = np.zeros(len(pts))
    for i in range(1, len(pts) - 1):
        a = pts[i] - pts[i - 1]
        b = pts[i + 1] - pts[i]
        cross = abs(a[0] * b[1] - a[1] * b[0])
        ds = 0.5 * (np.linalg.norm(a) + np.linalg.norm(b))
        k[i] = cross / max(ds ** 3, 1e-9)
    k[0], k[-1] = k[1], k[-2]
    return k


def arc_lengths(points: Sequence[Sequence[float]]) -> np.ndarray:
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return np.zeros(len(pts))
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])


@dataclass
class Profile:
    points: np.ndarray
    speed: np.ndarray
    time: np.ndarray
    curvature: np.ndarray
    max_lateral_accel: float
    max_accel: float
    duration: float


def speed_profile(points: Sequence[Sequence[float]], v_max: float = 1.0,
                  a_max: float = 0.8, a_lat_max: float = 0.5,
                  v_start: float = 0.0, v_end: float = 0.0,
                  min_speed: float = 0.12) -> Profile:
    pts = np.asarray(points, dtype=float)
    s = arc_lengths(pts)
    k = curvature(pts)
    ds = np.diff(s)
    ds[ds < 1e-6] = 1e-6

    v_curve = np.sqrt(a_lat_max / np.maximum(k, 1e-6))
    v = np.minimum(v_max, np.maximum(v_curve, min_speed))
    v[0] = max(v_start, min_speed)
    v[-1] = max(v_end, min_speed)

    for i in range(1, len(v)):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * a_max * ds[i - 1]))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * a_max * ds[i]))
    v = np.maximum(v, min_speed)

    dt = ds / np.maximum(0.5 * (v[:-1] + v[1:]), 1e-6)
    t = np.concatenate([[0.0], np.cumsum(dt)])
    accel = np.abs(np.diff(v) / np.maximum(dt, 1e-6))
    return Profile(
        points=pts, speed=v, time=t, curvature=k,
        max_lateral_accel=float(np.max(k * v * v)),
        max_accel=float(np.max(accel)) if accel.size else 0.0,
        duration=float(t[-1]),
    )


def constant_profile(points: Sequence[Sequence[float]], v: float = 1.0) -> Profile:
    pts = np.asarray(points, dtype=float)
    s = arc_lengths(pts)
    speed = np.full(len(pts), v)
    t = s / v
    k = curvature(pts)
    return Profile(pts, speed, t, k, float(np.max(k * v * v)), 0.0, float(t[-1]))
