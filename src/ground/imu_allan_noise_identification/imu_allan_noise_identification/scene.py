"""Synthetic IMU data: a static log for identification and a tilt sweep for the filter.

The generator reproduces the two noise types the model keeps: white rate noise
(ARW) and a 1/f flicker component whose Allan floor is calibrated to
``0.664 * B`` (bias instability).  A constant bias offset is added to the sweep
so that the filter has something to estimate.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .core import BIAS_INSTABILITY_FACTOR, NoiseModel, cluster_means


@dataclass(frozen=True)
class ImuConfig:
    """Sampling layout of the synthetic IMU."""

    rate_hz: float = 100.0
    static_duration_s: float = 3600.0
    sweep_duration_s: float = 300.0
    sweep_amplitude_deg: float = 10.0
    sweep_frequency_hz: float = 0.02
    accel_noise_deg: float = 0.35
    constant_bias_deg_s: float = 0.02
    noise: NoiseModel = NoiseModel()
    seed: int = 0

    def validate(self) -> None:
        if self.rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        if self.static_duration_s <= 0.0 or self.sweep_duration_s <= 0.0:
            raise ValueError("durations must be positive")
        self.noise.validate()


def _pink_series(rng: np.random.Generator, count: int, dt: float) -> np.ndarray:
    """Unit-amplitude 1/f series built in the frequency domain."""
    frequencies = np.fft.rfftfreq(count, dt)
    weights = np.zeros_like(frequencies)
    weights[1:] = 1.0 / np.sqrt(frequencies[1:])
    spectrum = weights * (
        rng.normal(size=frequencies.size) + 1j * rng.normal(size=frequencies.size)
    )
    series = np.fft.irfft(spectrum, n=count)
    return series - series.mean()


def _calibrated_flicker(
    rng: np.random.Generator, count: int, dt: float, bias_instability: float
) -> np.ndarray:
    """1/f component scaled so its Allan floor equals ``0.664 * B``."""
    if bias_instability <= 0.0:
        return np.zeros(count)
    series = _pink_series(rng, count, dt)
    size = max(2, min(int(round(100.0 / dt)), count // 10))
    if size * 2 >= count:
        return np.zeros(count)
    means = cluster_means(series, size)
    difference = means[size:] - means[:-size]
    if difference.size == 0:
        return np.zeros(count)
    floor = float(np.sqrt(np.sum(difference**2) / (2.0 * difference.size)))
    target = BIAS_INSTABILITY_FACTOR * bias_instability / 3600.0
    if floor <= 0.0:
        return np.zeros(count)
    return series * (target / floor)


def _white_and_flicker(config: ImuConfig, rng: np.random.Generator, count: int):
    dt = 1.0 / config.rate_hz
    rate_variance, _ = config.noise.sample_noise(dt)
    white = rng.normal(0.0, np.sqrt(rate_variance), count)
    flicker = _calibrated_flicker(rng, count, dt, config.noise.bias_instability)
    return white, flicker, dt


def simulate_static(config: ImuConfig) -> dict:
    """Stationary gyro data (default one hour at 100 Hz)."""
    config.validate()
    rng = np.random.default_rng(config.seed)
    count = int(round(config.static_duration_s * config.rate_hz))
    white, flicker, dt = _white_and_flicker(config, rng, count)
    time_s = np.arange(count) * dt
    return {
        "time_s": time_s,
        "dt": dt,
        "rate_deg_s": white + flicker,
        "flicker_deg_s": flicker,
        "truth": config.noise,
    }


def simulate_tilt_sweep(config: ImuConfig) -> dict:
    """A slow roll sweep: gyro rate plus an accelerometer tilt measurement."""
    config.validate()
    rng = np.random.default_rng(config.seed + 1)
    count = int(round(config.sweep_duration_s * config.rate_hz))
    white, flicker, dt = _white_and_flicker(config, rng, count)
    time_s = np.arange(count) * dt
    amplitude = np.deg2rad(config.sweep_amplitude_deg)
    omega = 2.0 * np.pi * config.sweep_frequency_hz
    tilt_deg = np.rad2deg(amplitude * np.sin(omega * time_s))
    tilt_rate_deg_s = np.rad2deg(amplitude * omega * np.cos(omega * time_s))
    gyro = tilt_rate_deg_s + config.constant_bias_deg_s + flicker + white
    accel_tilt = tilt_deg + rng.normal(0.0, config.accel_noise_deg, count)
    return {
        "time_s": time_s,
        "dt": dt,
        "tilt_deg": tilt_deg,
        "gyro_deg_s": gyro,
        "accel_tilt_deg": accel_tilt,
        "accel_sigma_deg": config.accel_noise_deg,
        "constant_bias_deg_s": config.constant_bias_deg_s,
        "truth": config.noise,
    }
