"""MuJoCo rollout dataset and episode-level splitting utilities.

The collector deliberately obtains every feature row from a rendered MuJoCo
frame.  It does not create synthetic image features, so the training set keeps
the same camera, actuator, gravity, and moving-platform distribution as the
landing demo.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, replace
from typing import Any

import numpy as np
from sklearn.model_selection import GroupShuffleSplit


@dataclass(frozen=True)
class MovingPadTrajectory:
    """Sinusoidal moving-pad parameters; frequencies are in rad/s."""

    amplitude_x: float = 0.34
    amplitude_y: float = 0.22
    frequency_x: float = 0.42
    frequency_y: float = 0.31
    phase_x: float = 0.0
    phase_y: float = 0.0

    @classmethod
    def from_value(
        cls, value: "MovingPadTrajectory | Mapping[str, float] | None"
    ) -> "MovingPadTrajectory":
        if value is None:
            return cls()
        if isinstance(value, cls):
            trajectory = value
        elif isinstance(value, Mapping):
            unknown = set(value) - set(cls.__dataclass_fields__)
            if unknown:
                raise ValueError(f"unknown trajectory fields: {sorted(unknown)}")
            trajectory = replace(cls(), **{key: float(item) for key, item in value.items()})
        else:
            raise TypeError("trajectory must be a MovingPadTrajectory or mapping")
        values = (
            trajectory.amplitude_x,
            trajectory.amplitude_y,
            trajectory.frequency_x,
            trajectory.frequency_y,
            trajectory.phase_x,
            trajectory.phase_y,
        )
        if not np.isfinite(values).all():
            raise ValueError("trajectory values must be finite")
        if trajectory.amplitude_x < 0 or trajectory.amplitude_y < 0:
            raise ValueError("trajectory amplitudes must be non-negative")
        if trajectory.frequency_x < 0 or trajectory.frequency_y < 0:
            raise ValueError("trajectory frequencies must be non-negative")
        return trajectory


def _trajectory_for_episode(
    rng: np.random.Generator,
    trajectory: MovingPadTrajectory | Mapping[str, float] | None,
) -> MovingPadTrajectory:
    """Return a fixed trajectory or a reproducible episode-specific variant."""
    if trajectory is not None:
        return MovingPadTrajectory.from_value(trajectory)
    # The ranges retain the scene's nominal motion while covering slower and
    # faster pads during training.  Phases are sampled independently per run.
    return MovingPadTrajectory(
        amplitude_x=float(rng.uniform(0.24, 0.34)),
        amplitude_y=float(rng.uniform(0.15, 0.22)),
        frequency_x=float(rng.uniform(0.30, 0.50)),
        frequency_y=float(rng.uniform(0.22, 0.38)),
        phase_x=float(rng.uniform(-np.pi, np.pi)),
        phase_y=float(rng.uniform(-np.pi, np.pi)),
    )


def _initial_pose_for_episode(
    rng: np.random.Generator,
    episode: int,
    initial_pose: Sequence[float] | Mapping[str, float] | None,
) -> tuple[float, float, float]:
    """Return a pose with explicit low-altitude coverage in every collection."""
    if initial_pose is not None:
        if isinstance(initial_pose, Mapping):
            try:
                values = (initial_pose["x"], initial_pose["y"], initial_pose["z"])
            except KeyError as exc:
                raise ValueError("initial_pose mapping requires x, y, and z") from exc
        else:
            if len(initial_pose) != 3:
                raise ValueError("initial_pose must contain x, y, and z")
            values = tuple(initial_pose)
        pose = tuple(float(value) for value in values)
    else:
        # Interleave low, medium, and high starts.  The first stratum starts
        # immediately above the platform, ensuring near-ground samples even
        # for short smoke-test episodes.
        strata = episode % 3
        z_range = ((0.235, 0.27), (0.42, 0.78), (0.82, 1.40))[strata]
        pose = (
            float(rng.uniform(-0.62, 0.62)),
            float(rng.uniform(-0.62, 0.62)),
            float(rng.uniform(*z_range)),
        )
    if not np.isfinite(pose).all():
        raise ValueError("initial_pose values must be finite")
    if not -1.3 <= pose[0] <= 1.3 or not -1.3 <= pose[1] <= 1.3:
        raise ValueError("initial x and y must be within the MuJoCo scene")
    if not 0.22 <= pose[2] <= 2.0:
        raise ValueError("initial z must be in the controllable altitude range")
    return pose


def _phase_override(
    trajectory: MovingPadTrajectory,
    phase: float | Sequence[float] | None,
) -> MovingPadTrajectory:
    """Apply an optional fixed x/y phase pair to the trajectory."""
    if phase is None:
        return trajectory
    if isinstance(phase, (int, float, np.number)):
        phase_x = phase_y = float(phase)
    else:
        if len(phase) != 2:
            raise ValueError("phase must be a scalar or an (x, y) pair")
        phase_x, phase_y = (float(value) for value in phase)
    if not np.isfinite((phase_x, phase_y)).all():
        raise ValueError("phase values must be finite")
    return replace(trajectory, phase_x=phase_x, phase_y=phase_y)


def _install_trajectory(simulation: Any, trajectory: MovingPadTrajectory) -> None:
    """Set public trajectory parameters used by ``LandingSimulation``."""
    simulation.pad_amplitude = np.array(
        [trajectory.amplitude_x, trajectory.amplitude_y], dtype=np.float64
    )
    simulation.pad_frequency = np.array(
        [trajectory.frequency_x, trajectory.frequency_y], dtype=np.float64
    )
    simulation.pad_phase = np.array(
        [trajectory.phase_x, trajectory.phase_y], dtype=np.float64
    )


def generate_grouped_samples(
    labeler: Callable[[np.ndarray], Sequence[float]] | None = None,
    *,
    episodes: int = 240,
    steps_per_episode: int = 20,
    seed: int = 7,
    samples: int | None = None,
    trajectory: MovingPadTrajectory | Mapping[str, float] | None = None,
    initial_pose: Sequence[float] | Mapping[str, float] | None = None,
    phase: float | Sequence[float] | None = None,
    render_size: tuple[int, int] = (160, 160),
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Collect feature rows and labels from complete MuJoCo rollouts.

    ``features`` are produced by ``vision_features(render(...))`` and labels
    come from ``labeler`` (``expert_command`` by default).  Rows carry an
    integer episode id so downstream splitting can keep whole trajectories
    together.  ``trajectory`` and ``initial_pose`` can be fixed explicitly;
    when omitted, the collector randomizes trajectory parameters, phase, and
    initial position/altitude from ``seed``.  ``phase`` overrides both pad axes
    with either one shared phase or an ``(x, y)`` pair.
    """
    if isinstance(steps_per_episode, bool) or steps_per_episode < 1:
        raise ValueError("steps_per_episode must be positive")
    if isinstance(episodes, bool) or episodes < 1:
        raise ValueError("episodes must be positive")
    if samples is not None:
        if isinstance(samples, bool) or samples < 2:
            raise ValueError("samples must be at least two")
        episodes = int(np.ceil(samples / steps_per_episode))
    if not isinstance(seed, (int, np.integer)):
        raise TypeError("seed must be an integer")
    if len(render_size) != 2 or any(isinstance(value, bool) or int(value) < 32 for value in render_size):
        raise ValueError("render_size must contain two dimensions of at least 32 pixels")
    width, height = (int(value) for value in render_size)

    # Import lazily: landing.py imports this module for its training entrypoint.
    from .landing import DT, LandingSimulation, expert_command, vision_features

    if labeler is None:
        labeler = expert_command
    rng = np.random.default_rng(int(seed))
    requested = samples if samples is not None else episodes * steps_per_episode
    rows: list[np.ndarray] = []
    labels: list[np.ndarray] = []
    groups: list[int] = []
    simulation = LandingSimulation(width=width, height=height)
    try:
        for episode in range(episodes):
            episode_count = min(steps_per_episode, requested - len(rows))
            if episode_count <= 0:
                break
            episode_trajectory = _phase_override(
                _trajectory_for_episode(rng, trajectory), phase
            )
            _install_trajectory(simulation, episode_trajectory)
            x, y, z = _initial_pose_for_episode(rng, episode, initial_pose)
            simulation.reset(x=x, y=y, z=z)
            for step in range(episode_count):
                time_s = step * DT
                image_rgb, pad_velocity = simulation.render(time_s)
                features = np.asarray(
                    vision_features(image_rgb, float(simulation.position[2]), pad_velocity),
                    dtype=np.float32,
                )
                if features.shape != (6,) or not np.isfinite(features).all():
                    raise RuntimeError("vision_features returned an invalid six-value row")
                command = np.asarray(labeler(features), dtype=np.float32)
                if command.shape != (3,) or not np.isfinite(command).all():
                    raise ValueError("labeler must return three finite command values")
                rows.append(features)
                labels.append(command)
                groups.append(episode)
                simulation.step(command, time_s)
    finally:
        simulation.close()

    if len(rows) != requested:
        raise RuntimeError(f"rollout produced {len(rows)} rows, expected {requested}")
    return np.stack(rows), np.stack(labels), np.asarray(groups, dtype=np.int64)


def episode_split_indices(
    groups: np.ndarray,
    *,
    test_size: float = 0.2,
    seed: int = 7,
) -> tuple[np.ndarray, np.ndarray]:
    """Return train/test row indices while keeping episode groups disjoint."""
    group_array = np.asarray(groups)
    if group_array.ndim != 1 or len(group_array) < 2:
        raise ValueError("groups must be a one-dimensional array with two rows")
    unique_groups = np.unique(group_array)
    if len(unique_groups) < 2:
        raise ValueError("at least two episode groups are required")
    splitter = GroupShuffleSplit(n_splits=1, test_size=test_size, random_state=seed)
    train_rows, test_rows = next(splitter.split(np.zeros((len(group_array), 1)), groups=group_array))
    if set(group_array[train_rows]) & set(group_array[test_rows]):
        raise RuntimeError("episode leakage detected in split")
    return train_rows, test_rows


def split_by_episode(
    features: np.ndarray,
    labels: np.ndarray,
    groups: np.ndarray,
    *,
    test_size: float = 0.2,
    seed: int = 7,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int, int]:
    """Split arrays and return train/test episode counts for auditability."""
    features = np.asarray(features)
    labels = np.asarray(labels)
    groups = np.asarray(groups)
    if len(features) != len(labels) or len(features) != len(groups):
        raise ValueError("features, labels, and groups must have equal rows")
    train_rows, test_rows = episode_split_indices(groups, test_size=test_size, seed=seed)
    train_groups = np.unique(groups[train_rows])
    test_groups = np.unique(groups[test_rows])
    return (
        features[train_rows],
        labels[train_rows],
        features[test_rows],
        labels[test_rows],
        int(len(train_groups)),
        int(len(test_groups)),
    )
