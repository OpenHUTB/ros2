"""Differential-drive kinematics and a wheel-odometry error model.

The simulator integrates the *true* unicycle motion with perfect ``(v, omega)``.
Odometry is reconstructed from the two wheel angular rates, using slightly wrong
wheel radius, wheelbase and a left/right radius mismatch, which is exactly the
systematic error a real differential-drive robot accumulates.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

EPS = 1e-9


@dataclass(frozen=True)
class WheelParameters:
    """Geometry and calibration of the differential drive."""

    radius_true: float = 0.065
    wheelbase_true: float = 0.320
    radius_est: float = 0.0663
    wheelbase_est: float = 0.3150
    asymmetry: float = 0.009
    encoder_sigma_v: float = 0.004
    encoder_sigma_w: float = 0.006

    def validate(self) -> None:
        values = (
            self.radius_true,
            self.wheelbase_true,
            self.radius_est,
            self.wheelbase_est,
        )
        if not all(np.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("wheel geometry must be positive and finite")
        if not 0.0 <= self.asymmetry < 0.5:
            raise ValueError("asymmetry must be in [0, 0.5)")


def wheel_rates(v: float, omega: float, params: WheelParameters) -> tuple[float, float]:
    """Wheel surface speeds ``(v_left, v_right)`` for a body twist."""
    left = v - omega * params.wheelbase_true / 2.0
    right = v + omega * params.wheelbase_true / 2.0
    return left, right


def wheel_angular_rates(v: float, omega: float, params: WheelParameters) -> tuple[float, float]:
    """Encoder-observed wheel angular rates with the left/right mismatch."""
    left, right = wheel_rates(v, omega, params)
    radius_left = params.radius_true * (1.0 - params.asymmetry / 2.0)
    radius_right = params.radius_true * (1.0 + params.asymmetry / 2.0)
    return left / radius_left, right / radius_right


def reconstruct(rate_left: float, rate_right: float, params: WheelParameters) -> tuple[float, float]:
    """Twist recovered by odometry from the measured wheel angular rates."""
    v = params.radius_est * (rate_left + rate_right) / 2.0
    omega = params.radius_est * (rate_right - rate_left) / params.wheelbase_est
    return v, omega


def odometry_of(v: float, omega: float, params: WheelParameters) -> tuple[float, float]:
    """Noise-free odometry reading of a true body twist."""
    return reconstruct(*wheel_angular_rates(v, omega, params), params)


def measure_odometry(
    v: float,
    omega: float,
    params: WheelParameters,
    rng: np.random.Generator,
    noise: bool = True,
) -> tuple[float, float]:
    """Return the noisy odometry reading ``(v_odom, omega_odom)``.

    The encoder noise is split into a *common* part, which moves both wheels the
    same way and therefore perturbs only the linear velocity, and a
    *differential* part, which perturbs only the yaw rate.  With the standard
    differential-drive relations the two sigmas map to
    ``sigma_v`` (m/s) and ``sigma_w`` (rad/s) exactly.
    """
    rate_left, rate_right = wheel_angular_rates(v, omega, params)
    if noise:
        common = rng.normal(0.0, params.encoder_sigma_v) / params.radius_true
        differential = (
            rng.normal(0.0, params.encoder_sigma_w)
            * params.wheelbase_true
            / (2.0 * params.radius_true)
        )
        rate_left += common + differential
        rate_right += common - differential
    return reconstruct(rate_left, rate_right, params)


def unicycle_step(state, v: float, omega: float, dt: float) -> tuple[float, float, float]:
    """Exact midpoint unicycle integration of ``(x, y, theta)``."""
    x, y, theta = state
    theta_mid = theta + 0.5 * omega * dt
    return (
        x + v * dt * np.cos(theta_mid),
        y + v * dt * np.sin(theta_mid),
        theta + omega * dt,
    )


def wrap_angle(angle: float) -> float:
    """Wrap an angle to ``[-pi, pi)``."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def dead_reckon(
    odometry_commands, dt: float, initial=(0.0, 0.0, 0.0)
) -> np.ndarray:
    """Integrate ``(v, omega)`` measurements into a pose trajectory."""
    states = np.empty((len(odometry_commands), 3), dtype=float)
    state = (float(initial[0]), float(initial[1]), float(initial[2]))
    for index, (v, omega) in enumerate(odometry_commands):
        state = unicycle_step(state, v, omega, dt)
        states[index] = state
    return states
