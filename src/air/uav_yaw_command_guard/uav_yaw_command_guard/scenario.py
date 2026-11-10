"""Deterministic yaw target scenario used by the ROS 2 and offline demos."""

from __future__ import annotations

import math


def target_yaw_at(elapsed_s: float) -> float:
    if elapsed_s < 1.0:
        return 0.0
    if elapsed_s < 3.0:
        return math.radians(170.0)
    if elapsed_s < 5.0:
        return math.radians(-170.0)
    if elapsed_s < 7.0:
        return math.radians(45.0)
    return 0.0
