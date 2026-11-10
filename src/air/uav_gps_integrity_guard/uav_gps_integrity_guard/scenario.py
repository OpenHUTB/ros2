"""Deterministic GPS spoofing and recovery scenario."""

from __future__ import annotations

from .core import GpsSample


def scenario_at(time_s: float) -> GpsSample:
    t = max(0.0, time_s)
    true_speed = 3.0
    truth_x = 3.0 * t
    truth_y = 0.0
    if t < 5.0:
        x, y = truth_x, truth_y
        reported, accuracy, imu = true_speed, 0.9, true_speed
    elif t < 9.0:
        u = t - 5.0
        x, y = truth_x + 1.8 * u, truth_y + 0.5 * u
        reported, accuracy, imu = 1.2, 4.8 + 0.4 * u, true_speed
    elif t < 13.0:
        u = t - 9.0
        x, y = truth_x + 9.0 - 2.0 * u, truth_y + 2.0 - 1.1 * u
        reported, accuracy, imu = 1.6, 7.5, true_speed
    else:
        u = min(1.0, (t - 13.0) / 3.0)
        x = truth_x + (1.0 - u) * 1.5
        y = truth_y
        reported = 1.6 + (true_speed - 1.6) * u
        accuracy = 7.5 - 6.6 * u
        imu = reported + (1.0 - u) * 0.7
    return GpsSample(t, x, y, 25.0, reported, accuracy, imu)
