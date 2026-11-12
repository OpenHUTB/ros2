"""Synthetic joint: velocity sweeps and a closed-loop sine tracking test."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .core import FrictionModel


@dataclass(frozen=True)
class JointConfig:
    """Inertia, controller gains and sweep layout."""

    inertia: float = 0.05
    damping: float = 0.01
    control_hz: float = 500.0
    sweep_speeds: tuple = (0.03, 0.05, 0.08, 0.12, 0.18, 0.25, 0.4, 0.6, 0.9, 1.2, 1.8, 2.4)
    sweep_time_s: float = 1.5
    torque_noise: float = 0.01
    sine_amplitude_rad: float = 0.5
    sine_frequency_hz: float = 0.5
    sine_duration_s: float = 6.0
    seed: int = 0

    def validate(self) -> None:
        if self.inertia <= 0.0:
            raise ValueError("inertia must be positive")
        if self.control_hz <= 0.0:
            raise ValueError("control_hz must be positive")
        if self.torque_noise < 0.0:
            raise ValueError("torque_noise must be non-negative")

    @property
    def dt(self) -> float:
        return 1.0 / self.control_hz


def simulate_sweeps(config: JointConfig, truth: FrictionModel) -> dict:
    """Steady-state torque at each sweep speed (inertia term is zero)."""
    config.validate()
    truth.validate()
    rng = np.random.default_rng(config.seed)
    speeds = np.asarray(config.sweep_speeds, dtype=float)
    torques = truth.torque(speeds) + config.damping * speeds
    torques = torques + rng.normal(0.0, config.torque_noise, speeds.size)
    return {"speeds": speeds, "torques": torques}


def _pd(state, target, kp, kd) -> float:
    position, velocity = state
    return float(kp * (target - position) - kd * velocity)


def simulate_tracking(config: JointConfig, truth: FrictionModel,
                      model: FrictionModel | None = None,
                      kp: float = 60.0, kd: float = 4.0) -> dict:
    """Track a sine with PD plus optional friction feed-forward."""
    config.validate()
    truth.validate()
    dt = config.dt
    steps = int(round(config.sine_duration_s * config.control_hz))
    time_s = np.arange(steps) * dt
    omega = 2.0 * np.pi * config.sine_frequency_hz
    target = config.sine_amplitude_rad * np.sin(omega * time_s)
    target_velocity = config.sine_amplitude_rad * omega * np.cos(omega * time_s)
    position = float(target[0])
    velocity = 0.0
    positions = np.empty(steps)
    torques = np.empty(steps)
    for index in range(steps):
        command = _pd((position, velocity), float(target[index]), kp, kd)
        command += kd * float(target_velocity[index])
        if model is not None:
            command += float(model.torque(velocity))
        friction = float(truth.torque(velocity))
        acceleration = (command - friction - config.damping * velocity) / config.inertia
        velocity += acceleration * dt
        position += velocity * dt
        positions[index] = position
        torques[index] = command
    error = positions - target
    return {
        "time_s": time_s,
        "target": target,
        "error": error,
        "position": positions,
        "torque": torques,
        "rms_error_rad": float(np.sqrt(np.mean(error**2))),
        "max_error_rad": float(np.max(np.abs(error))),
    }
