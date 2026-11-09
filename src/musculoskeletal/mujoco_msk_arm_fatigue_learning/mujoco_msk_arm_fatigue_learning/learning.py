"""Online gain learning for fatigue compensation."""

from __future__ import annotations

from math import isfinite
from typing import Iterable

import numpy as np

from .fatigue import MUSCLE_NAMES


DEFAULT_MOMENT_ARMS = np.array(
    [
        [-0.060, 0.000],
        [0.060, 0.000],
        [0.000, -0.050],
        [0.000, 0.050],
        [-0.030, -0.040],
        [0.030, 0.040],
    ],
    dtype=float,
)


class AdaptiveFatigueCompensator:
    """Learn per-muscle gains from persistent directional tracking error."""

    def __init__(
        self,
        moment_arms: np.ndarray = DEFAULT_MOMENT_ARMS,
        learning_rate: float = 3.0,
        gain_decay_rate: float = 0.20,
        maximum_gain: float = 2.0,
        activation_deadband: float = 0.02,
        error_deadband_rad: float = 0.005,
        error_ceiling_rad: float = 0.30,
    ) -> None:
        moment_arms = np.asarray(moment_arms, dtype=float)
        if moment_arms.shape != (len(MUSCLE_NAMES), 2):
            raise ValueError('moment_arms must have shape (6, 2)')
        if not np.all(np.isfinite(moment_arms)):
            raise ValueError('moment_arms values must be finite')
        for name, value in (
            ('learning_rate', learning_rate),
            ('gain_decay_rate', gain_decay_rate),
            ('maximum_gain', maximum_gain),
            ('activation_deadband', activation_deadband),
            ('error_deadband_rad', error_deadband_rad),
            ('error_ceiling_rad', error_ceiling_rad),
        ):
            if not isfinite(value) or value < 0.0:
                raise ValueError('%s must be finite and non-negative' % name)
        if maximum_gain < 1.0:
            raise ValueError('maximum gain must be at least 1')
        if error_ceiling_rad <= error_deadband_rad:
            raise ValueError('error ceiling must exceed error deadband')

        self.moment_arms = moment_arms
        self.learning_rate = float(learning_rate)
        self.gain_decay_rate = float(gain_decay_rate)
        self.maximum_gain = float(maximum_gain)
        self.activation_deadband = float(activation_deadband)
        self.error_deadband_rad = float(error_deadband_rad)
        self.error_ceiling_rad = float(error_ceiling_rad)
        self.gain = np.ones(len(MUSCLE_NAMES))

    def update(
        self,
        base_activation: Iterable[float],
        joint_error_rad: Iterable[float],
        dt_s: float,
    ) -> np.ndarray:
        activation = np.asarray(tuple(base_activation), dtype=float)
        joint_error = np.asarray(tuple(joint_error_rad), dtype=float)
        if activation.shape != (len(MUSCLE_NAMES),):
            raise ValueError('base activation must contain 6 values')
        if joint_error.shape != (2,):
            raise ValueError('joint error must contain 2 values')
        if not np.all(np.isfinite(activation)):
            raise ValueError('base activation values must be finite')
        if not np.all(np.isfinite(joint_error)):
            raise ValueError('joint error values must be finite')
        if not isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError('timestep must be positive and finite')

        activation = np.clip(activation, 0.0, 1.0)
        error_magnitude = float(np.linalg.norm(joint_error))
        useful = (
            error_magnitude > self.error_deadband_rad
        ) & (
            error_magnitude <= self.error_ceiling_rad
        ) & (
            activation > self.activation_deadband
        )
        self.gain = np.where(
            useful,
            self.gain
            + self.learning_rate
            * error_magnitude
            * activation
            * float(dt_s),
            self.gain
            - self.gain_decay_rate * (self.gain - 1.0) * float(dt_s),
        )
        self.gain = np.clip(self.gain, 1.0, self.maximum_gain)
        return self.gain.copy()

    def compensate(
        self,
        base_activation: Iterable[float],
        strength_retention: Iterable[float],
    ) -> np.ndarray:
        activation = np.asarray(tuple(base_activation), dtype=float)
        retention = np.asarray(tuple(strength_retention), dtype=float)
        if activation.shape != (len(MUSCLE_NAMES),):
            raise ValueError('base activation must contain 6 values')
        if retention.shape != (len(MUSCLE_NAMES),):
            raise ValueError('strength retention must contain 6 values')
        if not np.all(np.isfinite(activation)):
            raise ValueError('base activation values must be finite')
        if not np.all(np.isfinite(retention)):
            raise ValueError('strength retention values must be finite')
        if np.any(retention <= 0.0):
            raise ValueError('strength retention must be positive')
        compensated = activation * retention * self.gain
        return np.clip(compensated, 0.0, 1.0)

    def reset(self) -> None:
        self.gain = np.ones(len(MUSCLE_NAMES))
