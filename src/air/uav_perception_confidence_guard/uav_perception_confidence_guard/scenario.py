"""Deterministic perception degradation and recovery scenario."""

from __future__ import annotations

from .core import PerceptionSample


def scenario_at(time_s: float) -> PerceptionSample:
    t = max(0.0, time_s)
    frame_id = int(t * 10)
    if t < 5.0:
        confidence, entropy, latency, age, lux, count = 0.91, 0.24, 42.0, 55.0, 420.0, 2
    elif t < 10.0:
        u = (t - 5.0) / 5.0
        confidence = 0.91 - 0.66 * u
        entropy = 0.24 + 1.36 * u
        latency = 42.0 + 408.0 * u
        age = 55.0 + 945.0 * u
        lux = 420.0 - 340.0 * u
        count = 2 if u < 0.72 else 1
    elif t < 15.0:
        u = (t - 10.0) / 5.0
        confidence = 0.44 - 0.10 * u
        entropy = 1.10 + 0.25 * u
        latency = 260.0 + 260.0 * u
        age = 390.0 + 1100.0 * u
        lux = 95.0 - 35.0 * u
        count = 0
    else:
        u = min(1.0, (t - 15.0) / 5.0)
        confidence = 0.34 + 0.57 * u
        entropy = 1.35 - 1.11 * u
        latency = 520.0 - 478.0 * u
        age = 1490.0 - 1435.0 * u
        lux = 60.0 + 360.0 * u
        count = 1 if u < 0.45 else 2
    return PerceptionSample(
        t,
        frame_id,
        True,
        count,
        confidence,
        entropy,
        latency,
        age,
        lux,
    )
