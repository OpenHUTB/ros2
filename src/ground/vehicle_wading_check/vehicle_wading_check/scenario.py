"""Deterministic wading scenario: water depth and vehicle speed over time."""

from __future__ import annotations


DEPTH_BREAKPOINTS = (
    (0.0, 0.00),
    (6.0, 0.42),
    (12.0, 0.42),
    (18.0, 0.05),
    (24.0, 0.05),
)

SPEED_BREAKPOINTS = (
    (0.0, 8.00),
    (4.0, 2.50),
    (12.0, 2.50),
    (18.0, 0.80),
    (24.0, 6.00),
)


def _interp(breakpoints, time_s: float) -> float:
    if time_s <= breakpoints[0][0]:
        return breakpoints[0][1]
    for (t0, v0), (t1, v1) in zip(breakpoints, breakpoints[1:]):
        if time_s <= t1:
            span = t1 - t0
            if span <= 0.0:
                return v1
            return v0 + (v1 - v0) * (time_s - t0) / span
    return breakpoints[-1][1]


def depth_at(time_s: float) -> float:
    """Water depth profile in metres (0 .. 0.42 m, risen and drained)."""
    return _interp(DEPTH_BREAKPOINTS, time_s)


def speed_at(time_s: float) -> float:
    """Vehicle speed profile in m/s (approach, wade slowly, leave)."""
    return _interp(SPEED_BREAKPOINTS, time_s)
