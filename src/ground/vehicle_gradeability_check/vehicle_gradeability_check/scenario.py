"""Deterministic slope and speed scenario used by the ROS 2 and offline demos."""

from __future__ import annotations


def slope_at(elapsed_s: float) -> float:
    """Return road slope in degrees for the fixed demo timeline."""
    if elapsed_s < 2.0:
        return 0.0
    if elapsed_s < 4.0:
        return 8.0
    if elapsed_s < 6.0:
        return 14.0
    if elapsed_s < 8.0:
        return 24.0
    return 10.0


def speed_at(elapsed_s: float) -> float:
    """Return ego speed in m/s for the fixed demo timeline."""
    if elapsed_s < 2.0:
        return 8.0
    if elapsed_s < 4.0:
        return 7.0
    if elapsed_s < 6.0:
        return 6.0
    if elapsed_s < 8.0:
        return 5.0
    return 6.0
