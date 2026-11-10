"""Deterministic ESC scenario: cruise -> voltage sag -> stale frame -> recovery."""

from __future__ import annotations

from .core import ESCSample


def scenario_at(t: float) -> ESCSample:
    if t < 3.0:
        return ESCSample(t, 22.5, 12.0, 5000.0, 45.0, 20.0)
    if t < 7.0:
        u = (t - 3.0) / 4.0
        return ESCSample(t, 22.5 - 3.0 * u, 12.0 + 25.0 * u, 5000.0, 45.0 + 10.0 * u, 30.0)
    if t < 10.0:
        return ESCSample(t, 18.5, 40.0, 6000.0, 75.0, 50.0)
    if t < 12.0:
        # ESC telemetry frame drops out
        return ESCSample(t, 18.0, 42.0, 6000.0, 78.0, 250.0)
    u = min(1.0, (t - 12.0) / 3.0)
    return ESCSample(t, 20.0 + 2.0 * u, 15.0 - 8.0 * u, 5000.0, 60.0 - 20.0 * u, 30.0)
