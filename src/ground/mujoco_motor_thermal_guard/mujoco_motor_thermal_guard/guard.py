"""Hysteretic temperature guard core independent of any simulator."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GuardDecision:
    predicted_temperature_c: float
    speed_scale: float
    active: bool


class MotorTemperatureGuard:
    def __init__(self, limit_c: float = 65.0, release_c: float = 60.0, horizon_s: float = 5.0) -> None:
        if release_c >= limit_c or horizon_s <= 0:
            raise ValueError("release_c must be below limit_c and horizon_s must be positive")
        self.limit_c = float(limit_c)
        self.release_c = float(release_c)
        self.horizon_s = float(horizon_s)
        self._previous: tuple[float, float] | None = None
        self.active = False

    def update(self, time_s: float, temperature_c: float) -> GuardDecision:
        if not math.isfinite(time_s) or not math.isfinite(temperature_c):
            raise ValueError("time and temperature must be finite")
        predicted = float(temperature_c)
        if self._previous is not None:
            previous_time, previous_temperature = self._previous
            delta = time_s - previous_time
            if delta <= 0:
                raise ValueError("time_s must increase between readings")
            slope = (temperature_c - previous_temperature) / delta
            predicted += max(0.0, slope) * self.horizon_s
        self._previous = (float(time_s), float(temperature_c))
        if self.active:
            self.active = predicted > self.release_c
        else:
            self.active = predicted >= self.limit_c
        scale = 1.0
        if self.active:
            scale = max(0.25, min(1.0, 1.0 - 0.03 * (predicted - self.release_c)))
        return GuardDecision(predicted, scale, self.active)


def update_guard_sample(
    guard: MotorTemperatureGuard, time_s: float, temperature_c: float
) -> tuple[GuardDecision | None, str | None]:
    try:
        return guard.update(time_s, temperature_c), None
    except ValueError as error:
        return None, str(error)
