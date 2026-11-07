import numpy as np

from mujoco_uav_moving_pad_landing.dataset import (
    episode_split_indices,
    generate_grouped_samples,
    split_by_episode,
)


def _label(features: np.ndarray) -> np.ndarray:
    return np.asarray(features[:3], dtype=np.float32)


def test_requested_sample_count_and_episode_groups_are_preserved():
    features, labels, groups = generate_grouped_samples(_label, samples=53, steps_per_episode=20)

    assert features.shape == (53, 6)
    assert labels.shape == (53, 3)
    assert np.unique(groups).size == 3
    assert sorted(np.bincount(groups).tolist()) == [13, 20, 20]


def test_episode_split_has_no_related_rows_on_both_sides():
    features, labels, groups = generate_grouped_samples(_label, episodes=40, steps_per_episode=5)
    x_train, y_train, x_test, y_test, train_episodes, test_episodes = split_by_episode(
        features, labels, groups
    )
    train_rows, test_rows = episode_split_indices(groups)

    assert train_episodes + test_episodes == 40
    assert x_train.shape[0] + x_test.shape[0] == groups.size
    assert y_train.shape[0] + y_test.shape[0] == groups.size
    assert np.intersect1d(groups[train_rows], groups[test_rows]).size == 0
