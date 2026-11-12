"""Encoder quantisation and four velocity estimators.

A quadrature encoder reports position in steps of ``2*pi/CPR``.  Differencing that
quantised position at the control rate multiplies the quantisation error by the
loop rate, which is why a 1024 CPR encoder can show a velocity noise of several
rad/s.  This module compares the raw difference with a moving average, an
alpha-beta tracker and a two-state Kalman filter, on the same trapezoidal profile.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

EPS = 1e-12


@dataclass(frozen=True)
class EncoderSpec:
    """Resolution and sampling of a quadrature encoder."""

    counts_per_revolution: int = 4096
    rate_hz: float = 1000.0
    noise_rad: float = 0.0

    def validate(self) -> None:
        if self.counts_per_revolution < 16:
            raise ValueError("resolution too low")
        if self.rate_hz <= 0.0:
            raise ValueError("rate must be positive")
        if self.noise_rad < 0.0:
            raise ValueError("noise must be non-negative")

    @property
    def step_rad(self) -> float:
        return 2.0 * np.pi / self.counts_per_revolution

    @property
    def dt(self) -> float:
        return 1.0 / self.rate_hz

    def quantise(self, position_rad) -> np.ndarray:
        position_rad = np.asarray(position_rad, dtype=float)
        return np.round(position_rad / self.step_rad) * self.step_rad


def raw_difference(position_rad, dt: float) -> np.ndarray:
    """Backward difference of the quantised position."""
    position_rad = np.asarray(position_rad, dtype=float)
    velocity = np.zeros_like(position_rad)
    velocity[1:] = np.diff(position_rad) / dt
    return velocity


def moving_average(signal, window: int) -> np.ndarray:
    signal = np.asarray(signal, dtype=float)
    if window < 1:
        raise ValueError("window must be positive")
    kernel = np.ones(window) / window
    return np.convolve(signal, kernel, mode="full")[: signal.size] / np.maximum(
        np.convolve(np.ones_like(signal), kernel, mode="full")[: signal.size], EPS
    )


def alpha_beta(position_rad, dt: float, alpha: float = 0.3, beta: float = 0.05) -> np.ndarray:
    """Alpha-beta tracker on the quantised position."""
    position_rad = np.asarray(position_rad, dtype=float)
    if not 0.0 < alpha <= 1.0 or not 0.0 < beta <= 1.0:
        raise ValueError("gains must be in (0, 1]")
    estimate_position = float(position_rad[0])
    estimate_velocity = 0.0
    velocity = np.empty_like(position_rad)
    for index, measured in enumerate(position_rad):
        predicted = estimate_position + estimate_velocity * dt
        residual = float(measured) - predicted
        estimate_position = predicted + alpha * residual
        estimate_velocity = estimate_velocity + beta * residual / dt
        velocity[index] = estimate_velocity
    return velocity


@dataclass
class VelocityFilter:
    """Two-state (velocity, acceleration) Kalman filter on encoder position."""

    state: np.ndarray
    covariance: np.ndarray
    process_sigma: float
    measurement_sigma: float

    @classmethod
    def create(cls, process_sigma: float = 20.0, measurement_sigma: float = 0.01) -> "VelocityFilter":
        if process_sigma <= 0.0 or measurement_sigma <= 0.0:
            raise ValueError("sigmas must be positive")
        return cls(
            state=np.zeros(2),
            covariance=np.eye(2),
            process_sigma=float(process_sigma),
            measurement_sigma=float(measurement_sigma),
        )

    def step(self, measurement: float, dt: float) -> float:
        transition = np.array([[1.0, dt], [0.0, 1.0]])
        control = np.array([[0.5 * dt**2], [dt]])
        self.state = transition @ self.state
        process = control @ control.T * self.process_sigma**2
        self.covariance = transition @ self.covariance @ transition.T + process
        innovation = measurement - self.state[0]
        variance = self.covariance[0, 0] + self.measurement_sigma**2
        gain = self.covariance[:, 0] / variance
        self.state = self.state + gain * innovation
        self.covariance = self.covariance - np.outer(gain, self.covariance[0, :])
        return float(self.state[1])


def kalman_velocity(position_rad, dt: float, process_sigma: float = 20.0,
                    measurement_sigma: float = 0.01) -> np.ndarray:
    filter_ = VelocityFilter.create(process_sigma, measurement_sigma)
    velocity = np.empty_like(np.asarray(position_rad, dtype=float))
    for index, measured in enumerate(np.asarray(position_rad, dtype=float)):
        velocity[index] = filter_.step(float(measured), dt)
    return velocity
