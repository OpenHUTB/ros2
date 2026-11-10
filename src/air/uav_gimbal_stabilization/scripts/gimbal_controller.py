"""Two-axis gimbal PID controller with rate and deadband limits."""

from __future__ import annotations

from dataclasses import dataclass, field
import math


def _clip(value, limit):
    return max(-float(limit), min(float(limit), float(value)))


@dataclass
class AxisPID:
    """Stateful PID for one gimbal axis; angles are expressed in degrees."""

    kp: float = 2.2
    ki: float = 0.15
    kd: float = 0.08
    max_rate: float = 60.0
    deadband: float = 0.15
    integral_limit: float = 15.0
    _integral: float = field(default=0.0, init=False)
    _previous_error: float | None = field(default=None, init=False)
    saturation_ratio: float = field(default=0.0, init=False)

    def reset(self):
        self._integral = 0.0
        self._previous_error = None
        self.saturation_ratio = 0.0

    def update(self, error, dt):
        error = float(error)
        dt = max(float(dt), 1e-3)
        if abs(error) <= self.deadband:
            self._integral = 0.0
            self._previous_error = error
            self.saturation_ratio = 0.0
            return 0.0

        self._integral = _clip(
            self._integral + error * dt,
            self.integral_limit,
        )
        derivative = (
            0.0
            if self._previous_error is None
            else (error - self._previous_error) / dt
        )
        self._previous_error = error
        raw = self.kp * error + self.ki * self._integral + self.kd * derivative
        rate = _clip(raw, self.max_rate)
        self.saturation_ratio = 0.0 if abs(raw) < 1e-9 else abs(raw - rate) / abs(raw)
        return rate


@dataclass(frozen=True)
class GimbalCommand:
    pan_rate: float
    tilt_rate: float
    pan_error: float
    tilt_error: float
    pan_saturated: bool
    tilt_saturated: bool


class GimbalController:
    """Independent pan/tilt PID controller."""

    def __init__(self, pan=None, tilt=None):
        self.pan = pan or AxisPID()
        self.tilt = tilt or AxisPID()

    def reset(self):
        self.pan.reset()
        self.tilt.reset()

    def command(self, target_pan, target_tilt, current_pan, current_tilt, dt):
        pan_error = float(target_pan) - float(current_pan)
        tilt_error = float(target_tilt) - float(current_tilt)
        pan_rate = self.pan.update(pan_error, dt)
        tilt_rate = self.tilt.update(tilt_error, dt)
        return GimbalCommand(
            pan_rate=pan_rate,
            tilt_rate=tilt_rate,
            pan_error=pan_error,
            tilt_error=tilt_error,
            pan_saturated=self.pan.saturation_ratio > 0.0,
            tilt_saturated=self.tilt.saturation_ratio > 0.0,
        )


def angular_error(target_pan, target_tilt, current_pan, current_tilt):
    pan_error = float(target_pan) - float(current_pan)
    tilt_error = float(target_tilt) - float(current_tilt)
    return math.sqrt(pan_error * pan_error + tilt_error * tilt_error)
