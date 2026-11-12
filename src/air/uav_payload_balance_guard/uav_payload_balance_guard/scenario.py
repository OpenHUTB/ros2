"""Deterministic payload shift and recovery scenario."""

from __future__ import annotations

import math

from .core import PayloadSample


def scenario_at(time_s: float) -> PayloadSample:
    t = max(0.0, time_s)
    base_thrust = 22.0
    if t < 5.0:
        imbalance, roll, sway, payload = 0.03, 1.0, 0.05, 4.2
    elif t < 11.0:
        u = (t - 5.0) / 6.0
        imbalance = 0.03 + 0.57 * u
        roll = 1.0 + 12.0 * u
        sway = 0.05 + 0.75 * u
        payload = 4.2 + 2.8 * u
    elif t < 15.0:
        u = (t - 11.0) / 4.0
        imbalance = 0.60 + 0.28 * u
        roll = 13.0 + 8.0 * u
        sway = 0.80 + 0.70 * u
        payload = 7.0 + 0.4 * u
    else:
        u = min(1.0, (t - 15.0) / 5.0)
        imbalance = 0.88 * (1.0 - u) + 0.05 * u
        roll = 21.0 * (1.0 - u) + 1.5 * u
        sway = 1.50 * (1.0 - u) + 0.08 * u
        payload = 7.4 - 2.7 * u
    left = base_thrust * (1.0 + imbalance / 2.0) + math.sin(t) * 0.10
    right = base_thrust * (1.0 - imbalance / 2.0) - math.sin(t) * 0.10
    return PayloadSample(t, left, right, roll, sway, payload, 8.0)
