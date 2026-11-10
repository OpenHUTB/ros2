"""Crossing-path scenario with independently moving temporary no-fly zones."""

from __future__ import annotations

from .core import DynamicZone, UAVMotion


def scenario_at(time_s: float) -> tuple[UAVMotion, list[DynamicZone]]:
    t = max(0.0, time_s)
    uav = UAVMotion(x=-25.0 + 3.0 * t, y=0.0, vx=3.0, vy=0.0)
    curve_time = max(0.0, t - 6.0)
    curve_acceleration = 0.25
    zones = [
        DynamicZone(
            "construction",
            center_x=12.0,
            center_y=-16.0
            + 2.6 * t
            - curve_acceleration * curve_time * curve_time,
            radius_m=8.0,
            vx=0.0,
            vy=2.6 - 2.0 * curve_acceleration * curve_time,
            active=t < 18.0,
        ),
        DynamicZone(
            "event-corridor",
            center_x=-4.0 + 0.4 * t,
            center_y=17.0,
            radius_m=5.5,
            vx=0.4,
            vy=0.0,
            active=t < 18.0,
        ),
    ]
    return uav, zones
