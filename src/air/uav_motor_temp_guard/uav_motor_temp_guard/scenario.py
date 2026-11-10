"""Deterministic scenario: cruise -> hard pull -> cooldown."""

from __future__ import annotations

from .core import MotorSample


def scenario_at(t: float) -> MotorSample:
    if t < 3.0:
        return MotorSample(t, 12.0, 25.0, 4000.0)
    if t < 8.0:
        u = (t - 3.0) / 5.0
        return MotorSample(t, 12.0 + 20.0 * u, 25.0, 4000.0 + 2000.0 * u)
    if t < 12.0:
        return MotorSample(t, 38.0, 25.0, 7000.0)
    return MotorSample(t, 10.0, 25.0, 3500.0)
