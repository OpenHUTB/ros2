"""Reactive safety layer shared by all controllers (classical and neural)."""

from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np

from ..config import LIDAR_BEAMS, LIDAR_RANGE_MAX


def front_clearance(scan: Sequence[float], half_angle_deg: float = 35.0) -> float:
    r = np.asarray(scan, dtype=np.float64)
    n = len(r)
    if n == 0:
        return LIDAR_RANGE_MAX
    idx = np.arange(n)
    angles = idx * (2.0 * math.pi / n)
    rel = (angles + math.pi) % (2.0 * math.pi) - math.pi
    mask = np.abs(rel) < math.radians(half_angle_deg)
    if not np.any(mask):
        return float(r.min())
    return float(r[mask].min())


def side_clearances(scan: Sequence[float]) -> Tuple[float, float]:
    r = np.asarray(scan, dtype=np.float64)
    n = len(r)
    angles = np.arange(n) * (2.0 * math.pi / n)
    rel = (angles + math.pi) % (2.0 * math.pi) - math.pi
    left_mask = (rel > math.radians(35)) & (rel < math.radians(100))
    right_mask = (rel < -math.radians(35)) & (rel > -math.radians(100))
    lv = float(r[left_mask].min()) if np.any(left_mask) else LIDAR_RANGE_MAX
    rv = float(r[right_mask].min()) if np.any(right_mask) else LIDAR_RANGE_MAX
    return lv, rv


def safety_filter(scan: Sequence[float], v: float, w: float,
                  hard_stop: float = 0.55, slow: float = 1.10) -> Tuple[float, float, dict]:
    """Override a controller when the lidar sees an imminent collision."""
    fc = front_clearance(scan)
    lv, rv = side_clearances(scan)
    info = {"front_clearance": fc, "left_clearance": lv, "right_clearance": rv,
            "intervened": False}
    if fc < hard_stop:
        # Stop and rotate towards the side with more free space.
        direction = 1.0 if rv > lv else -1.0
        return 0.0, 1.25 * direction, {**info, "intervened": True}
    if fc < slow:
        return float(v * max(0.15, (fc - hard_stop) / (slow - hard_stop))), float(w), info
    return float(v), float(w), info
