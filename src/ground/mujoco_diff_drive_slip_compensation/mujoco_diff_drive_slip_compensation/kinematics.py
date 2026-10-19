"""Differential-drive kinematics and command safety helpers."""

from dataclasses import dataclass
import math
import time
from typing import Callable, Tuple


@dataclass(frozen=True)
class DriveGeometry:
    """Physical dimensions and actuator limits for a differential drive."""

    wheel_radius: float
    track_width: float
    max_wheel_speed: float

    def __post_init__(self) -> None:
        for name, value in (
            ('wheel_radius', self.wheel_radius),
            ('track_width', self.track_width),
            ('max_wheel_speed', self.max_wheel_speed),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be a finite positive value')

    def wheel_speeds(self, linear: float, angular: float) -> Tuple[float, float]:
        """Convert chassis velocity to clipped left and right wheel rad/s."""
        if not math.isfinite(linear) or not math.isfinite(angular):
            raise ValueError('linear and angular commands must be finite')

        left = (linear - 0.5 * self.track_width * angular) / self.wheel_radius
        right = (linear + 0.5 * self.track_width * angular) / self.wheel_radius
        peak = max(abs(left), abs(right))
        if peak > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak
            left *= scale
            right *= scale
        return left, right


class CommandWatchdog:
    """Return zero commands after a bounded period without input."""

    def __init__(
        self,
        timeout_seconds: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not math.isfinite(timeout_seconds) or timeout_seconds <= 0.0:
            raise ValueError('timeout_seconds must be a finite positive value')
        self._timeout_seconds = timeout_seconds
        self._clock = clock
        self._linear = 0.0
        self._angular = 0.0
        self._last_update: float | None = None

    def update(self, linear: float, angular: float) -> None:
        if not math.isfinite(linear) or not math.isfinite(angular):
            raise ValueError('linear and angular commands must be finite')
        self._linear = linear
        self._angular = angular
        self._last_update = self._clock()

    def current(self) -> Tuple[float, float]:
        if self._last_update is None:
            return 0.0, 0.0
        if self._clock() - self._last_update > self._timeout_seconds:
            return 0.0, 0.0
        return self._linear, self._angular
