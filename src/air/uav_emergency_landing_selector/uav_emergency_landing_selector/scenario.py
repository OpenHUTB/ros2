"""Deterministic emergency landing scenario shared by ROS and offline demos."""

from __future__ import annotations

import math

from .core import LandingSite, UAVState


def scenario_at(time_s: float) -> tuple[UAVState, list[LandingSite]]:
    phase = max(0.0, time_s)
    uav = UAVState(
        x=8.0 * math.sin(0.12 * phase),
        y=20.0 + 0.18 * phase,
        altitude_m=35.0,
        battery_percent=max(24.0, 58.0 - 0.12 * phase),
        wind_speed_mps=4.0 + 1.5 * math.sin(0.2 * phase),
    )
    sites = [
        LandingSite("A", 18.0, 28.0, 4.0, 0.12, 0.18, 0.05, 0.94),
        LandingSite("B", -6.0, 42.0, 8.0, 0.22, 0.28, 0.08, 0.86),
        LandingSite(
            "C",
            12.0,
            55.0,
            22.0,
            0.18,
            0.22,
            0.04,
            0.90,
        ),
        LandingSite(
            "D",
            2.0,
            18.0,
            5.0,
            0.16,
            0.32,
            0.78,
            0.88,
        ),
        LandingSite(
            "E",
            28.0,
            34.0,
            7.0,
            0.20,
            0.35,
            0.22,
            0.55 + 0.30 * math.sin(0.15 * phase) ** 2,
        ),
    ]
    return uav, sites
