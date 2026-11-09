"""Lumped muscle-tendon and proprioception model."""

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

DEFAULT_REST_LENGTHS_M = np.array(
    [0.200, 0.200, 0.230, 0.230, 0.300, 0.300],
    dtype=float,
)

DEFAULT_TENDON_STIFFNESS_N_M = np.array(
    [3500.0, 3500.0, 3000.0, 3000.0, 2500.0, 2500.0],
    dtype=float,
)

DEFAULT_MAX_FORCES_N = np.array(
    [220.0, 220.0, 180.0, 180.0, 150.0, 150.0],
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
class ProprioceptiveSample:
    """True proprioceptive state before noise and delay."""

    musculotendon_length_m: np.ndarray
    tendon_extension_m: np.ndarray
    spindle_length_m: np.ndarray
    spindle_velocity_m_s: np.ndarray
    golgi_force_n: np.ndarray
    activation: np.ndarray

    def validate(self) -> None:
        expected = len(MUSCLE_NAMES)
        for name in (
            'musculotendon_length_m',
            'tendon_extension_m',
            'spindle_length_m',
            'spindle_velocity_m_s',
            'golgi_force_n',
            'activation',
        ):
            values = np.asarray(getattr(self, name), dtype=float)
            if values.shape != (expected,):
                raise ValueError('%s must contain %d values' % (name, expected))
            if not np.all(np.isfinite(values)):
                raise ValueError('%s values must be finite' % name)


class ProprioceptionModel:
    """Estimate spindle and Golgi signals from joint and muscle state."""

    def __init__(
        self,
        muscle_names: Iterable[str] = MUSCLE_NAMES,
        moment_arms: np.ndarray = DEFAULT_MOMENT_ARMS,
        resting_lengths_m: np.ndarray = DEFAULT_REST_LENGTHS_M,
        tendon_stiffness_n_m: np.ndarray = DEFAULT_TENDON_STIFFNESS_N_M,
        max_forces_n: np.ndarray = DEFAULT_MAX_FORCES_N,
        velocity_filter_tau_s: float = 0.02,
    ) -> None:
        muscle_names = tuple(muscle_names)
        if muscle_names != MUSCLE_NAMES:
            raise ValueError(
                'muscle order must match the documented six-muscle order'
            )
        moment_arms = np.asarray(moment_arms, dtype=float)
        if moment_arms.shape != (len(MUSCLE_NAMES), 2):
            raise ValueError('moment_arms must have shape (6, 2)')
        if not np.all(np.isfinite(moment_arms)):
            raise ValueError('moment_arms values must be finite')

        resting_lengths_m = _vector(
            'resting_lengths_m',
            resting_lengths_m,
            len(MUSCLE_NAMES),
        )
        tendon_stiffness_n_m = _vector(
            'tendon_stiffness_n_m',
            tendon_stiffness_n_m,
            len(MUSCLE_NAMES),
        )
        max_forces_n = _vector(
            'max_forces_n',
            max_forces_n,
            len(MUSCLE_NAMES),
        )
        if np.any(resting_lengths_m <= 0.0):
            raise ValueError('resting lengths must be positive')
        if np.any(tendon_stiffness_n_m <= 0.0):
            raise ValueError('tendon stiffness must be positive')
        if np.any(max_forces_n <= 0.0):
            raise ValueError('maximum forces must be positive')
        if not isfinite(velocity_filter_tau_s) or velocity_filter_tau_s <= 0.0:
            raise ValueError('velocity filter time constant must be positive')

        self.muscle_names = muscle_names
        self.moment_arms = moment_arms
        self.resting_lengths_m = resting_lengths_m
        self.tendon_stiffness_n_m = tendon_stiffness_n_m
        self.max_forces_n = max_forces_n
        self.velocity_filter_tau_s = float(velocity_filter_tau_s)
        self._previous_force_n: np.ndarray | None = None
        self._filtered_spindle_velocity: np.ndarray | None = None

    def estimate(
        self,
        shoulder_rad: float,
        elbow_rad: float,
        shoulder_velocity_rad_s: float,
        elbow_velocity_rad_s: float,
        activations: Iterable[float],
        muscle_forces_n: Iterable[float],
        dt_s: float,
    ) -> ProprioceptiveSample:
        joint_values = np.array(
            [
                shoulder_rad,
                elbow_rad,
                shoulder_velocity_rad_s,
                elbow_velocity_rad_s,
                dt_s,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(joint_values)):
            raise ValueError('joint state and timestep must be finite')
        if dt_s <= 0.0:
            raise ValueError('timestep must be positive')

        activation = _vector(
            'activations',
            activations,
            len(MUSCLE_NAMES),
        )
        activation = np.clip(activation, 0.0, 1.0)
        force = _vector(
            'muscle_forces_n',
            muscle_forces_n,
            len(MUSCLE_NAMES),
        )
        force = np.maximum(force, 0.0)

        joint_position = np.array([shoulder_rad, elbow_rad], dtype=float)
        joint_velocity = np.array(
            [shoulder_velocity_rad_s, elbow_velocity_rad_s],
            dtype=float,
        )
        musculotendon_length = (
            self.resting_lengths_m + self.moment_arms @ joint_position
        )
        musculotendon_length = np.maximum(musculotendon_length, 0.01)
        tendon_extension = force / self.tendon_stiffness_n_m
        spindle_length = np.maximum(
            musculotendon_length - tendon_extension,
            0.005,
        )

        if self._previous_force_n is None:
            force_rate = np.zeros(len(MUSCLE_NAMES))
            raw_spindle_velocity = self.moment_arms @ joint_velocity
        else:
            force_rate = (
                force - self._previous_force_n
            ) / float(dt_s)
            tendon_velocity = force_rate / self.tendon_stiffness_n_m
            raw_spindle_velocity = (
                self.moment_arms @ joint_velocity - tendon_velocity
            )

        alpha = float(dt_s) / (
            self.velocity_filter_tau_s + float(dt_s)
        )
        if self._filtered_spindle_velocity is None:
            filtered_velocity = raw_spindle_velocity
        else:
            filtered_velocity = (
                self._filtered_spindle_velocity
                + alpha
                * (raw_spindle_velocity - self._filtered_spindle_velocity)
            )

        self._previous_force_n = force.copy()
        self._filtered_spindle_velocity = filtered_velocity.copy()
        sample = ProprioceptiveSample(
            musculotendon_length_m=musculotendon_length,
            tendon_extension_m=tendon_extension,
            spindle_length_m=spindle_length,
            spindle_velocity_m_s=filtered_velocity,
            golgi_force_n=force,
            activation=activation,
        )
        sample.validate()
        return sample

    def reset(self) -> None:
        self._previous_force_n = None
        self._filtered_spindle_velocity = None


def coactivation_joint_stiffness(
    coactivation: float,
    muscle_names: Iterable[str] = MUSCLE_NAMES,
    moment_arms: np.ndarray = DEFAULT_MOMENT_ARMS,
    max_forces_n: np.ndarray = DEFAULT_MAX_FORCES_N,
    resting_lengths_m: np.ndarray = DEFAULT_REST_LENGTHS_M,
    active_stiffness_gain: float = 4.0,
) -> np.ndarray:
    """Return joint stiffness from symmetric agonist-antagonist activation."""
    muscle_names = tuple(muscle_names)
    if muscle_names != MUSCLE_NAMES:
        raise ValueError(
            'muscle order must match the documented six-muscle order'
        )
    moment_arms = np.asarray(moment_arms, dtype=float)
    if moment_arms.shape != (len(MUSCLE_NAMES), 2):
        raise ValueError('moment_arms must have shape (6, 2)')
    if not np.all(np.isfinite(moment_arms)):
        raise ValueError('moment_arms values must be finite')
    if not isfinite(coactivation) or not 0.0 <= coactivation <= 1.0:
        raise ValueError('coactivation must be in [0, 1]')
    max_forces_n = _vector(
        'max_forces_n',
        max_forces_n,
        len(MUSCLE_NAMES),
    )
    resting_lengths_m = _vector(
        'resting_lengths_m',
        resting_lengths_m,
        len(MUSCLE_NAMES),
    )
    if np.any(resting_lengths_m <= 0.0):
        raise ValueError('resting lengths must be positive')
    if not isfinite(active_stiffness_gain) or active_stiffness_gain <= 0.0:
        raise ValueError('active stiffness gain must be positive')

    muscle_stiffness = (
        active_stiffness_gain * max_forces_n / resting_lengths_m
    )
    activation = np.full(len(MUSCLE_NAMES), float(coactivation))
    return (
        moment_arms.T
        @ np.diag(muscle_stiffness * activation)
        @ moment_arms
    )
