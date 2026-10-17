"""Online slip inference and bounded velocity compensation policy."""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .slip_model import IMU_FIELDS, WHEEL_VELOCITY_FIELDS, TemporalSlipMlp


@dataclass(frozen=True)
class CompensationResult:
    """One control output and the slip estimate that produced it."""

    linear: float
    angular: float
    slip: float
    ready: bool
    reason: str


class SlipCompensationPolicy:
    """Validate synchronized sensors, infer slip, and bound control changes."""

    def __init__(
        self,
        model: TemporalSlipMlp,
        sensor_timeout_seconds: float = 0.2,
        max_linear_scale: float = 1.5,
        max_linear_acceleration: float = 1.5,
        max_angular_acceleration: float = 3.0,
    ) -> None:
        for name, value in (
            ('sensor_timeout_seconds', sensor_timeout_seconds),
            ('max_linear_scale', max_linear_scale),
            ('max_linear_acceleration', max_linear_acceleration),
            ('max_angular_acceleration', max_angular_acceleration),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        self.model = model
        self.sensor_timeout_ns = int(sensor_timeout_seconds * 1e9)
        self.max_linear_scale = max_linear_scale
        self.max_linear_acceleration = max_linear_acceleration
        self.max_angular_acceleration = max_angular_acceleration
        self._window: deque[np.ndarray] = deque(maxlen=model.window_size)
        self._last_stamp_ns: int | None = None
        self._last_linear = 0.0
        self._last_angular = 0.0

    def _safe_stop(self, stamp_ns: int, reason: str) -> CompensationResult:
        self._window.clear()
        self._last_stamp_ns = stamp_ns
        self._last_linear = 0.0
        self._last_angular = 0.0
        return CompensationResult(0.0, 0.0, 0.0, False, reason)

    def update(
        self,
        stamp_ns: int,
        imu_stamp_ns: int,
        odometry_stamp_ns: int,
        command_stamp_ns: int,
        wheel_velocities: tuple[float, float, float, float],
        acceleration: tuple[float, float, float],
        angular_velocity: tuple[float, float, float],
        command_linear: float,
        command_angular: float,
    ) -> CompensationResult:
        """Consume one encoder sample and return a fail-safe command."""
        stamps = (stamp_ns, imu_stamp_ns, odometry_stamp_ns, command_stamp_ns)
        if stamp_ns <= 0 or any(value <= 0 for value in stamps):
            return self._safe_stop(stamp_ns, 'missing timestamp')
        if self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
            return self._safe_stop(stamp_ns, 'non-monotonic encoder timestamp')
        if any(stamp_ns - value < 0 or stamp_ns - value > self.sensor_timeout_ns
               for value in stamps[1:]):
            return self._safe_stop(stamp_ns, 'stale sensor or command')

        values = (*wheel_velocities, *acceleration, *angular_velocity,
                  command_linear, command_angular)
        if not all(math.isfinite(value) for value in values):
            return self._safe_stop(stamp_ns, 'non-finite input')
        lf, lr, rf, rr = wheel_velocities
        left = 0.5 * (lf + lr)
        right = 0.5 * (rf + rr)
        features = np.asarray((
            left,
            right,
            0.25 * (lf + lr + rf + rr),
            right - left,
            command_linear,
            command_angular,
            *acceleration,
            *angular_velocity,
        ), dtype=np.float64)
        if len(features) != len(WHEEL_VELOCITY_FIELDS) + len(IMU_FIELDS) + 2:
            raise RuntimeError('online feature contract does not match model')
        self._window.append(features)
        previous_stamp_ns = self._last_stamp_ns
        self._last_stamp_ns = stamp_ns
        if len(self._window) < self.model.window_size:
            self._last_linear = 0.0
            self._last_angular = 0.0
            return CompensationResult(0.0, 0.0, 0.0, False, 'warming window')

        slip = float(np.clip(self.model.predict(np.stack(self._window)), -0.5, 0.5))
        scale = min(1.0 / max(1.0 - slip, 0.5), self.max_linear_scale)
        target_linear = command_linear * scale
        target_angular = command_angular
        dt = (
            0.0 if previous_stamp_ns is None
            else (stamp_ns - previous_stamp_ns) / 1e9
        )
        if dt <= 0.0:
            return self._safe_stop(stamp_ns, 'invalid control interval')
        linear_delta = self.max_linear_acceleration * dt
        angular_delta = self.max_angular_acceleration * dt
        linear = self._last_linear + float(np.clip(
            target_linear - self._last_linear, -linear_delta, linear_delta
        ))
        angular = self._last_angular + float(np.clip(
            target_angular - self._last_angular, -angular_delta, angular_delta
        ))
        self._last_linear = linear
        self._last_angular = angular
        return CompensationResult(linear, angular, slip, True, 'ok')
