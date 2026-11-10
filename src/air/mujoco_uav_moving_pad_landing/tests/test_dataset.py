import numpy as np

from mujoco_uav_moving_pad_landing.dataset import (
    MovingPadTrajectory,
    episode_split_indices,
    generate_grouped_samples,
    split_by_episode,
)
from mujoco_uav_moving_pad_landing.landing import expert_command


def test_rollout_features_cover_low_altitude_and_match_expert_labels():
    features, labels, groups = generate_grouped_samples(
        expert_command, samples=30, steps_per_episode=10, seed=23, render_size=(96, 96)
    )

    assert features.shape == (30, 6)
    assert labels.shape == (30, 3)
    assert np.isfinite(features).all() and np.isfinite(labels).all()
    np.testing.assert_allclose(labels, np.vstack([expert_command(row) for row in features]))
    # The first episode starts near the platform and must contribute low-altitude
    # rows; this catches accidental replacement by a high-altitude synthetic sampler.
    assert float(features[:, 2].min()) <= 0.035
    assert set(np.unique(groups)) == {0, 1, 2}
    assert all(np.sum(groups == episode) == 10 for episode in range(3))


def test_fixed_trajectory_phase_is_reflected_in_rendered_velocity_features():
    trajectory = MovingPadTrajectory(
        amplitude_x=0.28,
        amplitude_y=0.17,
        frequency_x=0.40,
        frequency_y=0.25,
    )
    features, _, groups = generate_grouped_samples(
        expert_command,
        samples=8,
        steps_per_episode=8,
        seed=9,
        trajectory=trajectory,
        initial_pose=(0.1, -0.1, 0.6),
        phase=(0.35, -0.45),
        render_size=(96, 96),
    )
    expected_velocity = np.array(
        [
            trajectory.amplitude_x * trajectory.frequency_x * np.cos(0.35),
            trajectory.amplitude_y * trajectory.frequency_y * np.cos(-0.45),
        ],
        dtype=np.float32,
    )
    np.testing.assert_allclose(features[0, 3:5], expected_velocity, rtol=1e-5, atol=1e-5)
    assert np.all(groups == 0)


def test_episode_split_has_no_group_leakage():
    features, labels, groups = generate_grouped_samples(
        expert_command, samples=36, steps_per_episode=6, seed=31, render_size=(96, 96)
    )
    x_train, y_train, x_test, y_test, train_count, test_count = split_by_episode(
        features, labels, groups, test_size=0.25, seed=31
    )
    train_rows, test_rows = episode_split_indices(groups, test_size=0.25, seed=31)
    train_groups = set(groups[train_rows].tolist())
    test_groups = set(groups[test_rows].tolist())
    assert train_groups.isdisjoint(test_groups)
    assert len(x_train) + len(x_test) == len(features)
    assert len(y_train) + len(y_test) == len(labels)
    assert train_count + test_count == len(np.unique(groups))
