"""Reproducible trajectory samples and episode-level evaluation splits."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
from sklearn.model_selection import GroupShuffleSplit


def generate_grouped_samples(
    labeler: Callable[[np.ndarray], np.ndarray],
    episodes: int = 240,
    steps_per_episode: int = 20,
    seed: int = 7,
    samples: int | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Generate related observations under stable episode identifiers.

    When ``samples`` is set, the final episode is shortened to return that
    exact number of rows while preserving episode identifiers for splitting.
    """
    if steps_per_episode < 1:
        raise ValueError("steps_per_episode must be positive")
    if samples is not None:
        if samples < 2:
            raise ValueError("samples must be at least two")
        episodes = (samples + steps_per_episode - 1) // steps_per_episode
    if episodes < 2:
        raise ValueError("at least two episodes are required")

    rng = np.random.default_rng(seed)
    rows: list[np.ndarray] = []
    labels: list[np.ndarray] = []
    groups: list[int] = []

    for episode in range(episodes):
        episode_steps = steps_per_episode
        if samples is not None:
            episode_steps = min(steps_per_episode, samples - episode * steps_per_episode)
        error = rng.uniform(-0.42, 0.42, size=2)
        height = float(rng.uniform(0.35, 1.0))
        phase = rng.uniform(0.0, 2.0 * np.pi, size=2)
        frequency = rng.uniform(0.2, 0.55, size=2)
        amplitude = rng.uniform(0.04, 0.18, size=2)

        for step in range(episode_steps):
            time_s = step * 0.05
            velocity = amplitude * frequency * np.cos(frequency * time_s + phase)
            features = np.array(
                [error[0], error[1], height, velocity[0], velocity[1], 1.0],
                dtype=np.float32,
            )
            rows.append(features)
            labels.append(np.asarray(labeler(features), dtype=np.float32))
            groups.append(episode)

            command = labels[-1]
            error -= command[:2] * 0.025 + velocity * 0.025
            height = max(0.0, height + float(command[2]) * 0.025)

    return (
        np.stack(rows),
        np.stack(labels),
        np.asarray(groups, dtype=np.int32),
    )


def split_by_episode(
    features: np.ndarray,
    labels: np.ndarray,
    groups: np.ndarray,
    test_size: float = 0.2,
    seed: int = 7,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int, int]:
    """Split complete trajectories so adjacent observations cannot leak."""
    train_rows, test_rows = episode_split_indices(groups, test_size=test_size, seed=seed)
    train_groups = np.unique(groups[train_rows])
    test_groups = np.unique(groups[test_rows])
    if np.intersect1d(train_groups, test_groups).size:
        raise RuntimeError("episode leakage detected between train and test sets")
    return (
        features[train_rows],
        labels[train_rows],
        features[test_rows],
        labels[test_rows],
        int(train_groups.size),
        int(test_groups.size),
    )


def episode_split_indices(
    groups: np.ndarray,
    test_size: float = 0.2,
    seed: int = 7,
) -> tuple[np.ndarray, np.ndarray]:
    """Return sample indices partitioned by whole episode IDs."""
    splitter = GroupShuffleSplit(n_splits=1, test_size=test_size, random_state=seed)
    train_rows, test_rows = next(splitter.split(np.zeros(groups.shape[0]), groups=groups))
    return train_rows, test_rows
