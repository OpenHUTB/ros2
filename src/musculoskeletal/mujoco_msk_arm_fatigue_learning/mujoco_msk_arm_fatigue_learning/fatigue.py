"""Muscle fatigue dynamics for the two-joint arm."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Iterable

import numpy as np


MUSCLE_NAMES = (
    'shoulder_flexor',
    'shoulder_extensor',
    'elbow_flexor',
    'elbow_extensor',
    'biarticular_flexor',
    'biarticular_extensor',
)

DEFAULT_FATIGUE_TIME_S = np.array(
    [7.0, 7.0, 6.0, 6.0, 8.0, 8.0],
    dtype=float,
)

DEFAULT_RECOVERY_TIME_S = np.array(
    [12.0, 12.0, 10.0, 10.0, 14.0, 14.0],
    dtype=float,
)

DEFAULT_STRENGTH_LOSS = np.array(
    [0.32, 0.32, 0.36, 0.36, 0.30, 0.30],
    dtype=float,
)


def _vector(
    name: str,
    values: Iterable[float],
    size: int,
) -> np.ndarray:
    vector = np.asarray(tuple(values), dtype=float)
    if vector.shape != (size,):
        raise ValueError('%s must contain %d values' % (name, size))
    if not np.all(np.isfinite(vector)):
        raise ValueError('%s values must be finite' % name)
    return vector


@dataclass(frozen=True)
class FatigueState:
    fatigue: np.ndarray
    strength_retention: np.ndarray
    effective_activation: np.ndarray

    def validate(self) -> None:
        size = len(MUSCLE_NAMES)
        for name in ('fatigue', 'strength_retention', 'effective_activation'):
            values = np.asarray(getattr(self, name), dtype=float)
            if values.shape != (size,):
                raise ValueError('%s must contain %d values' % (name, size))
            if not np.all(np.isfinite(values)):
                raise ValueError('%s values must be finite' % name)


class MuscleFatigueModel:
    """First-order fatigue accumulation and recovery for six muscles."""

    def __init__(
        self,
        fatigue_time_s: Iterable[float] = DEFAULT_FATIGUE_TIME_S,
        recovery_time_s: Iterable[float] = DEFAULT_RECOVERY_TIME_S,
        strength_loss: Iterable[float] = DEFAULT_STRENGTH_LOSS,
        activation_threshold: float = 0.05,
        maximum_fatigue: float = 0.95,
    ) -> None:
        size = len(MUSCLE_NAMES)
        self.fatigue_time_s = _vector(
            'fatigue_time_s',
            fatigue_time_s,
            size,
        )
        self.recovery_time_s = _vector(
            'recovery_time_s',
            recovery_time_s,
            size,
        )
        self.strength_loss = _vector(
            'strength_loss',
            strength_loss,
            size,
        )
        if np.any(self.fatigue_time_s <= 0.0):
            raise ValueError('fatigue time constants must be positive')
        if np.any(self.recovery_time_s <= 0.0):
            raise ValueError('recovery time constants must be positive')
        if np.any(self.strength_loss < 0.0) or np.any(
            self.strength_loss >= 1.0
        ):
            raise ValueError('strength loss must be within [0, 1)')
        if not isfinite(activation_threshold) or not (
            0.0 <= activation_threshold < 1.0
        ):
            raise ValueError('activation threshold must be within [0, 1)')
        if not isfinite(maximum_fatigue) or not (
            0.0 < maximum_fatigue <= 1.0
        ):
            raise ValueError('maximum fatigue must be within (0, 1]')

        self.activation_threshold = float(activation_threshold)
        self.maximum_fatigue = float(maximum_fatigue)
        self.fatigue = np.zeros(size)

    def update(
        self,
        activation: Iterable[float],
        dt_s: float,
    ) -> FatigueState:
        activation_values = _vector(
            'activation',
            activation,
            len(MUSCLE_NAMES),
        )
        activation_values = np.clip(activation_values, 0.0, 1.0)
        if not isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError('timestep must be positive and finite')

        active_drive = np.maximum(
            activation_values - self.activation_threshold,
            0.0,
        )
        fatigue_rate = (
            active_drive * (1.0 - self.fatigue) / self.fatigue_time_s
            - (1.0 - active_drive) * self.fatigue / self.recovery_time_s
        )
        self.fatigue = np.clip(
            self.fatigue + float(dt_s) * fatigue_rate,
            0.0,
            self.maximum_fatigue,
        )
        retention = 1.0 - self.strength_loss * self.fatigue
        effective_activation = activation_values * retention
        state = FatigueState(
            fatigue=self.fatigue.copy(),
            strength_retention=retention,
            effective_activation=effective_activation,
        )
        state.validate()
        return state

    def reset(self) -> None:
        self.fatigue = np.zeros(len(MUSCLE_NAMES))
