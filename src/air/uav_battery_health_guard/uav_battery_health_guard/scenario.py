"""Deterministic battery imbalance, heating and recovery scenario."""

from __future__ import annotations

from .core import BatterySample


def scenario_at(time_s: float) -> BatterySample:
    t = max(0.0, time_s)
    if t < 5.0:
        imbalance, temperature, resistance, current = 0.015, 31.0, 14.0, 24.0
    elif t < 11.0:
        u = (t - 5.0) / 6.0
        imbalance = 0.015 + 0.205 * u
        temperature = 31.0 + 30.0 * u
        resistance = 14.0 + 27.0 * u
        current = 24.0 + 35.0 * u
    elif t < 15.0:
        u = (t - 11.0) / 4.0
        imbalance = 0.22 - 0.05 * u
        temperature = 61.0 + 4.0 * u
        resistance = 41.0 - 7.0 * u
        current = 40.0 - 8.0 * u
    else:
        u = min(1.0, (t - 15.0) / 5.0)
        imbalance = 0.17 * (1.0 - u) + 0.02 * u
        temperature = 65.0 - 30.0 * u
        resistance = 34.0 - 18.0 * u
        current = 32.0 - 8.0 * u
    mean_cell = 3.82
    cells = (
        mean_cell + imbalance / 2.0,
        mean_cell - imbalance / 2.0,
        mean_cell + 0.003,
        mean_cell - 0.004,
    )
    return BatterySample(t, cells, current, temperature, resistance)
