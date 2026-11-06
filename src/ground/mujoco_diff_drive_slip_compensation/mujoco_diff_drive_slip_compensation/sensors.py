"""Reproducible encoder and IMU noise models."""

from dataclasses import dataclass
import math
from typing import Iterable, Tuple

import numpy as np


Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class SensorNoiseConfig:
    """Standard deviations for simulated sensor measurements."""

    encoder_position_std: float = 0.0005
    encoder_velocity_std: float = 0.005
    accelerometer_std: float = 0.02
    gyroscope_std: float = 0.002

    def __post_init__(self) -> None:
        values = (
            self.encoder_position_std,
            self.encoder_velocity_std,
            self.accelerometer_std,
            self.gyroscope_std,
        )
        if any(not math.isfinite(value) or value < 0.0 for value in values):
            raise ValueError('sensor noise values must be finite and non-negative')


class SensorNoise:
    """Apply Gaussian noise from a local deterministic random generator."""

    def __init__(self, config: SensorNoiseConfig, seed: int) -> None:
        self.config = config
        self._generator = np.random.default_rng(seed)

    def encoders(
        self,
        positions: Iterable[float],
        velocities: Iterable[float],
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """Return noisy encoder positions and velocities."""
        position_array = np.asarray(tuple(positions), dtype=float)
        velocity_array = np.asarray(tuple(velocities), dtype=float)
        if position_array.shape != velocity_array.shape:
            raise ValueError('encoder position and velocity shapes must match')
        noisy_position = position_array + self._generator.normal(
            0.0, self.config.encoder_position_std, position_array.shape
        )
        noisy_velocity = velocity_array + self._generator.normal(
            0.0, self.config.encoder_velocity_std, velocity_array.shape
        )
        return tuple(noisy_position), tuple(noisy_velocity)

    def imu(
        self,
        acceleration: Iterable[float],
        angular_velocity: Iterable[float],
    ) -> tuple[Vector3, Vector3]:
        """Return noisy three-axis accelerometer and gyroscope values."""
        acceleration_array = np.asarray(tuple(acceleration), dtype=float)
        angular_array = np.asarray(tuple(angular_velocity), dtype=float)
        if acceleration_array.shape != (3,) or angular_array.shape != (3,):
            raise ValueError('IMU vectors must contain exactly three values')
        noisy_acceleration = acceleration_array + self._generator.normal(
            0.0, self.config.accelerometer_std, 3
        )
        noisy_angular = angular_array + self._generator.normal(
            0.0, self.config.gyroscope_std, 3
        )
        return tuple(noisy_acceleration), tuple(noisy_angular)
