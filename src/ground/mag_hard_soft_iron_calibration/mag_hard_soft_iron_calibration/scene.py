"""Deterministic magnetometer data: a calibration sweep and a heading run."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def rotation_matrix(yaw, pitch, roll):
    """ZYX rotation matrix, vectorised over the sample axis."""
    yaw = np.atleast_1d(yaw)
    pitch = np.atleast_1d(pitch)
    roll = np.atleast_1d(roll)
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    matrix = np.empty((yaw.size, 3, 3))
    matrix[:, 0, 0] = cy * cp
    matrix[:, 0, 1] = cy * sp * sr - sy * cr
    matrix[:, 0, 2] = cy * sp * cr + sy * sr
    matrix[:, 1, 0] = sy * cp
    matrix[:, 1, 1] = sy * sp * sr + cy * cr
    matrix[:, 1, 2] = sy * sp * cr - cy * sr
    matrix[:, 2, 0] = -sp
    matrix[:, 2, 1] = cp * sr
    matrix[:, 2, 2] = cp * cr
    return matrix


@dataclass(frozen=True)
class MagConfig:
    """Field, distortion and noise of the synthetic magnetometer."""

    field_norm_ut: float = 48.0
    inclination_deg: float = 60.0
    hard_iron_ut: tuple = (20.0, -15.0, 8.0)
    soft_diagonal: tuple = (1.28, 0.84, 1.12)
    soft_cross: float = 0.10
    noise_ut: float = 0.5
    calibration_samples: int = 1200
    heading_rate_hz: float = 50.0
    heading_duration_s: float = 120.0
    seed: int = 0

    def validate(self) -> None:
        if self.field_norm_ut <= 0.0:
            raise ValueError("field_norm_ut must be positive")
        if self.noise_ut < 0.0:
            raise ValueError("noise_ut must be non-negative")
        if self.calibration_samples < 20:
            raise ValueError("need at least 20 calibration samples")

    @property
    def soft_matrix(self) -> np.ndarray:
        cross = self.soft_cross
        return np.array(
            [
                [self.soft_diagonal[0], cross, 0.5 * cross],
                [cross, self.soft_diagonal[1], cross],
                [0.5 * cross, cross, self.soft_diagonal[2]],
            ]
        )

    @property
    def hard_iron(self) -> np.ndarray:
        return np.asarray(self.hard_iron_ut, dtype=float)

    def field_enu(self) -> np.ndarray:
        """Earth field in the local ENU frame (north along +x)."""
        inclination = np.deg2rad(self.inclination_deg)
        return self.field_norm_ut * np.array(
            [np.cos(inclination), 0.0, -np.sin(inclination)]
        )


def simulate_calibration(config: MagConfig) -> dict:
    """Rotate the sensor over a sphere and record distorted measurements."""
    config.validate()
    rng = np.random.default_rng(config.seed)
    count = config.calibration_samples
    yaw = rng.uniform(-np.pi, np.pi, count)
    pitch = rng.uniform(-np.pi / 3.0, np.pi / 3.0, count)
    roll = rng.uniform(-np.pi / 4.0, np.pi / 4.0, count)
    rotations = rotation_matrix(yaw, pitch, roll)
    field_enu = config.field_enu()
    true_field = np.einsum("nji,j->ni", rotations, field_enu)
    measured = true_field @ config.soft_matrix.T + config.hard_iron
    measured = measured + rng.normal(0.0, config.noise_ut, measured.shape)
    return {
        "true_field_ut": true_field,
        "measured_ut": measured,
        "yaw_rad": yaw,
        "pitch_rad": pitch,
        "roll_rad": roll,
        "soft_matrix": config.soft_matrix,
        "hard_iron": config.hard_iron,
    }


def simulate_heading(config: MagConfig) -> dict:
    """Slow yaw sweep with gyro, accelerometer and magnetometer streams."""
    config.validate()
    rng = np.random.default_rng(config.seed + 1)
    count = int(round(config.heading_rate_hz * config.heading_duration_s))
    dt = 1.0 / config.heading_rate_hz
    time_s = np.arange(count) * dt
    amplitude = np.deg2rad(90.0)
    rate_amplitude = 2.0 * np.pi * 0.01
    yaw = amplitude * np.sin(rate_amplitude * time_s)
    yaw_rate = amplitude * rate_amplitude * np.cos(rate_amplitude * time_s)
    pitch = np.deg2rad(4.0) * np.sin(2.0 * np.pi * 0.05 * time_s)
    roll = np.deg2rad(2.0) * np.sin(2.0 * np.pi * 0.07 * time_s + 0.4)

    rotations = rotation_matrix(yaw, pitch, roll)
    field_enu = config.field_enu()
    true_field = np.einsum("nji,j->ni", rotations, field_enu)
    measured = true_field @ config.soft_matrix.T + config.hard_iron
    measured = measured + rng.normal(0.0, config.noise_ut, measured.shape)

    gravity_sensor = np.einsum("nji,j->ni", rotations, np.array([0.0, 0.0, 9.81]))
    gravity_sensor = gravity_sensor + rng.normal(0.0, 0.05, gravity_sensor.shape)
    gyro = yaw_rate + np.deg2rad(0.4) + rng.normal(0.0, np.deg2rad(0.3), count)
    return {
        "time_s": time_s,
        "dt": dt,
        "yaw_rad": yaw,
        "gyro_rad_s": gyro,
        "acceleration": gravity_sensor,
        "measured_ut": measured,
        "true_field_ut": true_field,
        "soft_matrix": config.soft_matrix,
        "hard_iron": config.hard_iron,
    }
