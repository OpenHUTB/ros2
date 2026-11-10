"""Deterministic loss-of-authority and recovery scenario."""

from __future__ import annotations

from .core import ControlSample


def scenario_at(time_s: float) -> ControlSample:
    t = max(0.0, time_s)
    if t < 5.0:
        cmd_r, act_r, cmd_p, act_p = 22.0, 21.0, 8.0, 7.5
        saturation, integrator, voltage = 0.10, 0.08, 3.85
    elif t < 9.0:
        u = (t - 5.0) / 4.0
        cmd_r, act_r = 38.0, 38.0 - 9.0 * u
        cmd_p, act_p = 27.0, 26.0 - 11.0 * u
        saturation = 0.45 + 0.45 * u
        integrator = 0.20 + 0.38 * u
        voltage = 3.68 - 0.10 * u
    elif t < 14.0:
        u = (t - 9.0) / 5.0
        cmd_r, act_r = 48.0, 17.0 + 3.0 * u
        cmd_p, act_p = 38.0, 12.0 + 2.0 * u
        saturation = 0.96
        integrator = 0.72 + 0.18 * u
        voltage = 3.55 - 0.14 * u
    else:
        u = min(1.0, (t - 14.0) / 4.0)
        cmd_r, act_r = 20.0, 12.0 + 7.0 * u
        cmd_p, act_p = 6.0, 3.0 + 2.6 * u
        saturation = 0.92 * (1.0 - u) + 0.12 * u
        integrator = 0.86 * (1.0 - u) + 0.10 * u
        voltage = 3.30 + 0.55 * u
    return ControlSample(
        t,
        cmd_r,
        act_r,
        cmd_p,
        act_p,
        saturation,
        integrator,
        voltage,
    )
