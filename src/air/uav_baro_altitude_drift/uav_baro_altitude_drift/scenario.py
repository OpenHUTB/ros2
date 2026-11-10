"""Deterministic baro drift scenario: clean -> washdown -> recovery."""

from __future__ import annotations

import math

from .core import AltSample


def scenario_at(time_s: float) -> AltSample:
    t = max(0.0, time_s)
    phase = t % 16.0
    truth_alt = 20.0 + 1.0 * math.sin(0.2 * t)

    if phase < 3.0:
        baro = truth_alt
        gps = truth_alt
        radar = truth_alt
        vs = 0.0
        pres = 1005.0
        temp = 22.0
    elif phase < 7.0:
        u = (phase - 3.0) / 4.0
        baro = truth_alt + 2.0 + 10.0 * u
        gps = truth_alt
        radar = truth_alt
        vs = 0.0
        pres = 1005.0 - 3.0 * u
        temp = 22.0
    elif phase < 10.0:
        baro = truth_alt + 14.0 + 2.0 * math.sin(2 * phase)
        gps = truth_alt
        radar = truth_alt
        vs = 0.5
        pres = 1001.0 + 4.0 * math.sin(1.5 * phase)
        temp = 23.0
    elif phase < 12.0:
        # hard downwash spike: pressure drops fast, baro reads wrong
        baro = truth_alt + 30.0
        gps = truth_alt
        radar = truth_alt
        vs = 0.0
        pres = 995.0
        temp = 24.0
    else:
        u = min(1.0, (phase - 12.0) / 3.0)
        baro = truth_alt + 15.0 - 13.0 * u
        gps = truth_alt
        radar = truth_alt
        vs = 0.0
        pres = 998.0 + 6.0 * u
        temp = 23.0
    return AltSample(t, baro, gps, radar, vs, pres, temp)
