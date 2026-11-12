"""Curvature-aware speed profiling along a 2-D path."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np


def curvature(points: Sequence[Sequence[float]]) -> np.ndarray:
    """Signed 3-point circumcircle curvature (1/m).

    The sign follows the cross product: left turn positive, right turn negative.
    A near-180-degree reversal is treated as a very large curvature instead of
    the zero produced by the cross product.
    """
    pts = np.asarray(points, dtype=float)
    if len(pts) < 3:
        return np.zeros(len(pts))
    k = np.zeros(len(pts))
    for i in range(1, len(pts) - 1):
        a = pts[i] - pts[i - 1]
        b = pts[i + 1] - pts[i]
        na = float(np.linalg.norm(a))
        nb = float(np.linalg.norm(b))
        if na < 1e-9 or nb < 1e-9:
            k[i] = 0.0
            continue
        cross = float(a[0] * b[1] - a[1] * b[0])
        denom = na * nb * float(np.linalg.norm(a + b))
        if denom < 1e-9:
            if float(np.dot(a, b)) < 0.0:
                k[i] = math.pi / max(0.5 * (na + nb), 1e-9)
            else:
                k[i] = 0.0
        else:
            k[i] = 2.0 * cross / denom
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


def _validate(points: Sequence[Sequence[float]], v_max: float, a_max: float,
              a_lat_max: float, min_speed: float, v_start: float,
              v_end: float) -> None:
    if len(points) < 2:
        raise ValueError("path needs at least 2 points")
    if v_max <= 0.0 or a_max <= 0.0 or a_lat_max <= 0.0:
        raise ValueError("v_max, a_max and a_lat_max must be positive")
    if not 0.0 <= min_speed <= v_max:
        raise ValueError("min_speed must be in [0, v_max]")
    if not 0.0 <= v_start <= v_max or not 0.0 <= v_end <= v_max:
        raise ValueError("v_start/v_end must be in [0, v_max]")


def speed_profile(points: Sequence[Sequence[float]], v_max: float = 1.0,
                  a_max: float = 0.8, a_lat_max: float = 0.5,
                  v_start: float = 0.0, v_end: float = 0.0,
                  min_speed: float = 0.12) -> Profile:
    _validate(points, v_max, a_max, a_lat_max, min_speed, v_start, v_end)
    pts = np.asarray(points, dtype=float)
    s = arc_lengths(pts)
    k = curvature(pts)
    ds = np.diff(s)
    ds[ds < 1e-9] = 1e-9

    v_curve = np.sqrt(0.9 * a_lat_max / np.maximum(np.abs(k), 1e-9))
    v = np.minimum(v_max, np.maximum(v_curve, min_speed))
    v[0] = v_start
    v[-1] = v_end

    for i in range(1, len(v)):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * a_max * ds[i - 1]))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * a_max * ds[i]))

    dt = ds / np.maximum(0.5 * (v[:-1] + v[1:]), 1e-6)
    t = np.concatenate([[0.0], np.cumsum(dt)])
    accel = np.abs(np.diff(v) / np.maximum(dt, 1e-6))

    heading = np.unwrap(np.arctan2(np.diff(pts[:, 1]), np.diff(pts[:, 0])))
    if len(heading) >= 2:
        yaw_rate = np.diff(heading) / np.maximum(dt[:-1], 1e-6)
        v_mid = 0.5 * (v[1:-1] + v[2:])[:len(yaw_rate)]
        lat = np.abs(yaw_rate * v_mid)
        max_lat = float(lat.max()) if lat.size else 0.0
    else:
        max_lat = 0.0

    return Profile(
        points=pts, speed=v, time=t, curvature=k,
        max_lateral_accel=max_lat,
        max_accel=float(accel.max()) if accel.size else 0.0,
        duration=float(t[-1]),
    )


def constant_profile(points: Sequence[Sequence[float]], v: float = 1.0) -> Profile:
    _validate(points, v, 1.0, 1.0, 0.0, 0.0, 0.0)
    pts = np.asarray(points, dtype=float)
    s = arc_lengths(pts)
    speed = np.full(len(pts), v)
    t = s / v
    k = curvature(pts)
    heading = np.unwrap(np.arctan2(np.diff(pts[:, 1]), np.diff(pts[:, 0])))
    if len(heading) >= 2 and len(t) >= 3:
        yaw_rate = np.diff(heading) / np.maximum(np.diff(t)[:-1], 1e-6)
        v_mid = np.full(len(yaw_rate), v)
        lat = np.abs(yaw_rate * v_mid)
        max_lat = float(lat.max()) if lat.size else 0.0
    else:
        max_lat = 0.0
    return Profile(pts, speed, t, k, max_lat, 0.0, float(t[-1]))

