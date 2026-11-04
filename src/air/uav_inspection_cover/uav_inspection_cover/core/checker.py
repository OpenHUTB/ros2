#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Coverage check: a viewpoint counts as covered when the route enters its sphere.

覆盖判定：航路折线（三维）与观察点距离 ≤ 覆盖半径即视为该观察角度被
拍摄；输出覆盖率与漏检清单。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Sequence

import numpy as np

from .geometry import ObservePoint, Waypoint


@dataclass
class CoverReport:
    rate: float
    covered: List[bool] = field(default_factory=list)
    missed: List[str] = field(default_factory=list)


def _point_seg_dist3d(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    v = b - a
    l2 = float(v @ v)
    if l2 <= 1e-12:
        return float(np.linalg.norm(p - a))
    u = min(1.0, max(0.0, float((p - a) @ v) / l2))
    return float(np.linalg.norm(p - (a + u * v)))


def cover_report(viewpoints: Sequence[ObservePoint],
                 waypoints: Sequence[Waypoint],
                 radius_m: float) -> CoverReport:
    """逐观察点判定覆盖：取航路所有航段上的最小三维距离。"""
    if not waypoints:
        return CoverReport(rate=0.0,
                           missed=[vp.tag for vp in viewpoints])
    wps = np.asarray(waypoints, dtype=float)
    covered: List[bool] = []
    missed: List[str] = []
    for vp in viewpoints:
        p = np.asarray(vp.xyz(), dtype=float)
        d = min(_point_seg_dist3d(p, a, b) for a, b in zip(wps[:-1], wps[1:]))
        ok = d <= radius_m
        covered.append(ok)
        if not ok:
            missed.append(vp.tag)
    n = len(viewpoints)
    return CoverReport(rate=(sum(covered) / n if n else 1.0),
                       covered=covered, missed=missed)
