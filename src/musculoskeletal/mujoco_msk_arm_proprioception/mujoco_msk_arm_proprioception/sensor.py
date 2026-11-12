"""Noise and transport-delay pipeline for proprioceptive signals."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import isfinite

import numpy as np

from .model import MUSCLE_NAMES, ProprioceptiveSample


@dataclass(frozen=True)
class SensedProprioception:
    """Delayed and noisy proprioceptive signals."""

    spindle_length_m: np.ndarray
    spindle_velocity_m_s: np.ndarray
    golgi_force_n: np.ndarray
    tendon_extension_m: np.ndarray

    def validate(self) -> None:
        expected = len(MUSCLE_NAMES)
        for name in (
            'spindle_length_m',
            'spindle_velocity_m_s',
            'golgi_force_n',
            'tendon_extension_m',
        ):
            values = np.asarray(getattr(self, name), dtype=float)
            if values.shape != (expected,):
                raise ValueError('%s must contain %d values' % (name, expected))
            if not np.all(np.isfinite(values)):
                raise ValueError('%s values must be finite' % name)


class NoisyDelayedProprioception:
    """Apply independent Gaussian noise and a fixed transport delay."""

    def __init__(
        self,
        sample_period_s: float = 0.01,
        delay_s: float = 0.03,
        length_noise_std_m: float = 0.0005,
        velocity_noise_std_m_s: float = 0.005,
        force_noise_std_n: float = 0.2,
        tendon_extension_noise_std_m: float = 0.0001,
        seed: int = 0,
    ) -> None:
        values = (
            sample_period_s,
            delay_s,
            length_noise_std_m,
            velocity_noise_std_m_s,
            force_noise_std_n,
            tendon_extension_noise_std_m,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError('sensor configuration values must be finite')
        if sample_period_s <= 0.0:
            raise ValueError('sample period must be positive')
        if delay_s < 0.0:
            raise ValueError('delay must be non-negative')
        if min(
            length_noise_std_m,
            velocity_noise_std_m_s,
            force_noise_std_n,
            tendon_extension_noise_std_m,
        ) < 0.0:
            raise ValueError('noise standard deviations must be non-negative')

        self.sample_period_s = float(sample_period_s)
        self.delay_s = float(delay_s)
        self.delay_samples = int(round(self.delay_s / self.sample_period_s))
        self.noise_std = np.array(
            [
                length_noise_std_m,
                velocity_noise_std_m_s,
                force_noise_std_n,
                tendon_extension_noise_std_m,
            ],
            dtype=float,
        )
        self._random = np.random.default_rng(seed)
        self._history: deque[np.ndarray] = deque()

    @property
    def ready(self) -> bool:
        return len(self._history) > self.delay_samples

    def measure(self, sample: ProprioceptiveSample) -> SensedProprioception:
        sample.validate()
        true_signal = np.vstack(
            (
                sample.spindle_length_m,
                sample.spindle_velocity_m_s,
                sample.golgi_force_n,
                sample.tendon_extension_m,
            )
        )
        noisy_signal = true_signal + self._random.normal(
            loc=0.0,
            scale=self.noise_std[:, None],
            size=true_signal.shape,
        )
        noisy_signal[0] = np.maximum(noisy_signal[0], 0.0)
        noisy_signal[2] = np.maximum(noisy_signal[2], 0.0)
        noisy_signal[3] = np.maximum(noisy_signal[3], 0.0)
        self._history.append(noisy_signal)
        while len(self._history) > self.delay_samples + 1:
            self._history.popleft()

        if len(self._history) <= self.delay_samples:
            delayed = np.zeros_like(noisy_signal)
        else:
            delayed = self._history[0]

        result = SensedProprioception(
            spindle_length_m=delayed[0].copy(),
            spindle_velocity_m_s=delayed[1].copy(),
            golgi_force_n=delayed[2].copy(),
            tendon_extension_m=delayed[3].copy(),
        )
        result.validate()
        return result

    def reset(self) -> None:
        self._history.clear()
