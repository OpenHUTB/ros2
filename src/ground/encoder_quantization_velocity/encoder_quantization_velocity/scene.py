"""Deterministic trapezoidal motion profile for the encoder experiments."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ProfileConfig:
    """Trapezoid: accelerate, cruise, decelerate, hold."""

    rate_hz: float = 1000.0
    peak_velocity: float = 6.0
    acceleration: float = 40.0
    duration_s: float = 1.2
    seed: int = 0

    def validate(self) -> None:
        if self.rate_hz <= 0.0 or self.duration_s <= 0.0:
            raise ValueError("rate and duration must be positive")
        if self.peak_velocity <= 0.0 or self.acceleration <= 0.0:
            raise ValueError("velocity and acceleration must be positive")

    @property
    def dt(self) -> float:
        return 1.0 / self.rate_hz


def trapezoid(config: ProfileConfig) -> dict:
    """Velocity and position of a trapezoidal profile."""
    config.validate()
    dt = config.dt
    steps = int(round(config.duration_s * config.rate_hz))
    time_s = np.arange(steps) * dt
    ramp_time = config.peak_velocity / config.acceleration
    velocity = np.zeros(steps)
    for index, time in enumerate(time_s):
        if time < ramp_time:
            velocity[index] = config.acceleration * time
        elif time < ramp_time + 0.4:
            velocity[index] = config.peak_velocity
        elif time < 2.0 * ramp_time + 0.4:
            velocity[index] = config.peak_velocity - config.acceleration * (
                time - ramp_time - 0.4
            )
        else:
            velocity[index] = 0.0
    position = np.cumsum(velocity) * dt
    return {"time_s": time_s, "velocity_rad_s": velocity, "position_rad": position, "dt": dt}


def measure(profile: dict, counts_per_revolution: int, noise_rad: float = 0.0,
            seed: int = 0) -> dict:
    """Quantised encoder position of a profile."""
    from .core import EncoderSpec

    rng = np.random.default_rng(seed)
    spec = EncoderSpec(counts_per_revolution=counts_per_revolution,
                       rate_hz=1.0 / profile["dt"], noise_rad=noise_rad)
    true_position = np.asarray(profile["position_rad"], dtype=float)
    noisy = true_position + rng.normal(0.0, noise_rad, true_position.size)
    quantised = spec.quantise(noisy)
    return {
        "position_rad": quantised,
        "true_position_rad": true_position,
        "step_rad": spec.step_rad,
        "dt": profile["dt"],
    }
