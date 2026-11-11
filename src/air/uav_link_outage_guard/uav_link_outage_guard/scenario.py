"""Deterministic radio-link scenario with degradation, outage and recovery."""

from __future__ import annotations

import math

from .core import LinkSample


def scenario_at(time_s: float) -> LinkSample:
    t = max(0.0, time_s)
    phase = t % 18.0
    if phase < 4.0:
        rssi, latency, loss, age = -58.0, 38.0, 0.01, 45.0
    elif phase < 8.0:
        u = (phase - 4.0) / 4.0
        rssi = -58.0 - 19.0 * u
        latency = 45.0 + 135.0 * u
        loss = 0.02 + 0.23 * u
        age = 55.0 + 245.0 * u
    elif phase < 11.0:
        rssi = -79.0 + 1.6 * math.sin(2.0 * phase)
        latency = 210.0 + 35.0 * math.sin(1.3 * phase)
        loss = 0.31 + 0.05 * math.sin(phase)
        age = 390.0
    elif phase < 14.0:
        rssi = -91.0
        latency = 620.0
        loss = 0.88
        age = 1800.0
    else:
        u = min(1.0, (phase - 14.0) / 3.0)
        rssi = -88.0 + 30.0 * u
        latency = 520.0 - 470.0 * u
        loss = 0.70 - 0.68 * u
        age = 1450.0 - 1370.0 * u
    return LinkSample(t, rssi, latency, max(0.0, loss), age)
