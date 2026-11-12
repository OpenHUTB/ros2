"""A compact 6-state loosely coupled navigation EKF.

State: position (NED), velocity (NED).
The EKF fuses IMU specific force with GPS position and barometric altitude.
Attitude is supplied by :class:`ComplementaryAttitude`.

Accelerometer bias is intentionally not part of the state: with only GPS
position and barometric altitude, the vertical bias is not observable and
tends to absorb gravity/attitude errors. Keep bias compensation in the IMU
driver, or add a full 15-state INS/GNSS filter with attitude and gyro bias.
"""

from __future__ import annotations

import numpy as np

from .attitude import GRAVITY_NED


class NavigationEKF:
    def __init__(
        self,
        initial_position: np.ndarray,
        accel_noise: float = 0.10,
    ) -> None:
        self.x = np.zeros(6)
        self.x[0:3] = np.asarray(initial_position, dtype=float)
        self.accel_noise = float(accel_noise)
        self.P = np.diag([4.0, 4.0, 4.0, 1.0, 1.0, 1.0]).astype(float)

    @property
    def position(self) -> np.ndarray:
        return self.x[0:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[3:6].copy()

    def predict(self, specific_force_body: np.ndarray, rotation: np.ndarray, dt: float) -> None:
        force = np.asarray(specific_force_body, dtype=float)
        accel_ned = rotation @ force + GRAVITY_NED

        self.x[0:3] += self.x[3:6] * dt + 0.5 * accel_ned * dt * dt
        self.x[3:6] += accel_ned * dt

        transition = np.eye(6)
        transition[0:3, 3:6] = np.eye(3) * dt

        noise_map = np.zeros((6, 3))
        noise_map[0:3, :] = 0.5 * dt * dt * rotation
        noise_map[3:6, :] = dt * rotation
        process = noise_map @ (self.accel_noise**2 * np.eye(3)) @ noise_map.T

        self.P = transition @ self.P @ transition.T + process
        self.P = 0.5 * (self.P + self.P.T)

    def update_position(self, position: np.ndarray, sigma_m: float) -> None:
        measurement = np.asarray(position, dtype=float).reshape(-1)
        if measurement.size not in (2, 3):
            raise ValueError("position must contain two or three coordinates")
        observation = np.zeros((measurement.size, 6))
        observation[:, 0 : measurement.size] = np.eye(measurement.size)
        self._update(
            measurement,
            observation,
            np.eye(measurement.size) * sigma_m**2,
        )

    def update_altitude(self, altitude: float, sigma_m: float) -> None:
        observation = np.zeros((1, 6))
        observation[0, 2] = 1.0
        self._update(
            np.array([float(altitude)]),
            observation,
            np.array([[sigma_m**2]]),
        )

    def _update(self, measurement: np.ndarray, observation: np.ndarray, noise: np.ndarray) -> None:
        innovation = measurement - observation @ self.x
        innovation_cov = observation @ self.P @ observation.T + noise
        gain = self.P @ observation.T @ np.linalg.inv(innovation_cov)
        self.x += gain @ innovation

        identity = np.eye(6)
        correction = identity - gain @ observation
        self.P = correction @ self.P @ correction.T + gain @ noise @ gain.T
        self.P = 0.5 * (self.P + self.P.T)
