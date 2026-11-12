"""Deterministic visual-inertial signals with a known time offset."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SignalConfig:
    """Sampling, excitation and noise of the synthetic pair."""

    rate_hz: float = 200.0
    duration_s: float = 8.0
    offset_s: float = 0.025
    visual_gain: float = 0.95
    gyro_bias: float = 0.02
    gyro_noise: float = 0.01
    visual_noise: float = 0.05
    seed: int = 0

    def validate(self) -> None:
        if self.rate_hz <= 0.0 or self.duration_s <= 0.0:
            raise ValueError("rate and duration must be positive")
        if abs(self.offset_s) > 0.1:
            raise ValueError("offset must stay within +-100 ms")

    @property
    def dt(self) -> float:
        return 1.0 / self.rate_hz

    @property
    def samples(self) -> int:
        return int(round(self.duration_s * self.rate_hz))


def true_rate(time_s, seed: int = 0) -> np.ndarray:
    """Excitation: two incommensurate sinusoids plus a chirp."""
    rng = np.random.default_rng(seed + 7)
    phases = rng.uniform(0.0, 2.0 * np.pi, 3)
    signal = (
        0.9 * np.sin(2.0 * np.pi * 0.7 * time_s + phases[0])
        + 0.5 * np.sin(2.0 * np.pi * 1.9 * time_s + phases[1])
        + 0.3 * np.sin(2.0 * np.pi * (0.4 * time_s + 0.35 * time_s**2) + phases[2])
    )
    return signal


def simulate_pair(config: SignalConfig) -> dict:
    """Gyro and visual-rate streams; the visual one is delayed by ``offset_s``."""
    config.validate()
    rng = np.random.default_rng(config.seed)
    dt = config.dt
    time_s = np.arange(config.samples) * dt
    rate = true_rate(time_s, config.seed)

    gyro = rate + config.gyro_bias + rng.normal(0.0, config.gyro_noise, rate.size)
    shift = int(round(config.offset_s / dt))
    delayed = np.zeros_like(rate)
    if shift >= 0:
        delayed[shift:] = rate[: rate.size - shift] if shift else rate
    else:
        delayed[: rate.size + shift] = rate[-shift:]
    visual = config.visual_gain * delayed + rng.normal(
        0.0, config.visual_noise, rate.size
    )
    return {
        "time_s": time_s,
        "dt": dt,
        "gyro": gyro,
        "visual": visual,
        "true_rate": rate,
        "true_offset_s": float(config.offset_s),
        "visual_gain": float(config.visual_gain),
    }
