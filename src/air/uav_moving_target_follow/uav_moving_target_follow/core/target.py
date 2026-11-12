#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Moving ground target: analytic waypoint path with constant speed.

Single-line summary: piecewise-linear ground path; pos/vel at any time t
is analytic, so the follower's reference and the closed-loop metrics need
no ODE integration for the target itself.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class GroundTarget:
    """地面目标：折线路径 + 恒速。路径走完后停在终点（vel = 0）。"""

    name: str
    waypoints: Tuple[Tuple[float, float], ...]
    speed: float                     # m/s

    def __post_init__(self):
        if self.speed <= 0.0:
            raise ValueError("target speed must be positive")
        if len(self.waypoints) < 2:
            raise ValueError("target path needs >= 2 waypoints")

    # ------------------------------------------------------------------ #
    @property
    def segs(self) -> List[Tuple[np.ndarray, np.ndarray, float, np.ndarray]]:
        """[(a, b, length, unit_dir), ...] 折线段表。"""
        out = []
        for i in range(len(self.waypoints) - 1):
            a = np.asarray(self.waypoints[i], dtype=float)
            b = np.asarray(self.waypoints[i + 1], dtype=float)
            d = b - a
            n = float(np.linalg.norm(d))
            out.append((a, b, n, d / max(n, 1e-9)))
        return out

    @property
    def total_len_m(self) -> float:
        return sum(s[2] for s in self.segs)

    @property
    def duration_s(self) -> float:
        """目标走完全程的时间（之后停在终点）。"""
        return self.total_len_m / self.speed

    def pos_vel(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """t 时刻目标位置与速度（世界系，走完停在终点）。"""
        if t < 0.0:
            t = 0.0
        s = t * self.speed
        for a, b, n, u in self.segs:
            if s <= n + 1e-9:
                return a + u * s, u * self.speed
            s -= n
        end = np.asarray(self.waypoints[-1], dtype=float)
        return end.copy(), np.zeros(2)


# --------------------------------------------------------------------------- #
# 场景
# --------------------------------------------------------------------------- #
def CASE_ROAD() -> GroundTarget:
    """郊外直路：300 m 直线匀速，考察跟车段稳态误差。"""
    return GroundTarget("road",
                        ((20.0, 0.0), (320.0, 0.0)), 6.0)


def CASE_TOWN() -> GroundTarget:
    """城镇街区：两个 90° 直角转弯，考察转角瞬态。"""
    return GroundTarget("town",
                        ((20.0, 0.0), (170.0, 0.0),
                         (170.0, 120.0), (320.0, 120.0)), 6.0)


def CASE_S_CURVE() -> GroundTarget:
    """S 形连续弯：3 个 60° 折弯，考察连续机动下的误差包络。"""
    return GroundTarget("s_curve",
                        ((20.0, 0.0), (120.0, 0.0), (170.0, 86.6),
                         (270.0, 86.6), (320.0, 173.2)), 5.0)


CASES = ("road", "town", "s_curve")


def make_case(name: str) -> GroundTarget:
    return {"road": CASE_ROAD, "town": CASE_TOWN,
            "s_curve": CASE_S_CURVE}[name]()
