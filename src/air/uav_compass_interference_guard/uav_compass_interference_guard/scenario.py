"""Deterministic magnetometer-interference scenario with approach and recovery."""

from __future__ import annotations

import math

from .core import CompassSample


def scenario_at(time_s: float) -> CompassSample:
    t = max(0.0, time_s)
    phase = t % 18.0

    # Reference truth heading slowly drifts (aircraft turning gently).
    truth_heading = 30.0 + 2.0 * math.sin(0.3 * t)

    if phase < 4.0:
        # Clean cruise away from ferrous structures.
        field = 50.0
        bias = 0.0
        current = 12.0
        speed = 8.0
        alt = 30.0
    elif phase < 8.0:
        u = (phase - 4.0) / 4.0
        field = 50.0 + 26.0 * u
        bias = 4.0 + 18.0 * u
        current = 12.0 + 3.0 * u
        speed = 8.0
        alt = 30.0 - 22.0 * u  # descending toward rooftop / vehicle
    elif phase < 11.0:
        field = 82.0 + 6.0 * math.sin(1.7 * phase)
        bias = 30.0 + 3.0 * math.sin(phase)
        current = 21.0
        speed = 6.0
        alt = 6.0
    elif phase < 14.0:
        # Hard EMI / near a power line: field saturates, heading wildly off.
        field = 132.0
        bias = 80.0
        current = 26.0
        speed = 4.5
        alt = 4.0
    else:
        u = min(1.0, (phase - 14.0) / 3.0)
        field = 120.0 - 68.0 * u
        bias = 60.0 - 58.0 * u
        current = 24.0 - 12.0 * u
        speed = 7.0 + 1.0 * u
        alt = 5.0 + 20.0 * u

    mag_heading = (truth_heading + bias) % 360.0
    gps_course = (truth_heading + 0.5) % 360.0  # GPS course ~ truth
    vio_heading = (truth_heading - 0.5) % 360.0  # VIO heading ~ truth

    return CompassSample(
        timestamp_s=t,
        mag_heading_deg=mag_heading,
        gps_course_deg=gps_course,
        vio_heading_deg=vio_heading,
        mag_field_ut=field,
        motor_current_a=current,
        ground_speed_mps=speed,
        altitude_m=alt,
    )
