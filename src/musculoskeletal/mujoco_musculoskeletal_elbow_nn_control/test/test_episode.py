from pathlib import Path

import pytest

from mujoco_musculoskeletal_elbow_nn_control.episode import (
    EpisodeSpec,
    load_episode_config,
    split_episode_ids,
)
from mujoco_musculoskeletal_elbow_nn_control.trajectories import TrajectorySpec


def test_step_and_sine_trajectories_are_deterministic() -> None:
    step = TrajectorySpec(kind="step", duration_s=2.0, step_time_s=0.5)
    sine = TrajectorySpec(kind="sine", duration_s=2.0, offset_rad=0.7)
    assert step.sample(0.4) == step.initial_angle_rad
    assert step.sample(0.5) == step.target_angle_rad
    assert sine.sample(0.0) == pytest.approx(0.7)


def test_split_keeps_complete_episode_ids_disjoint() -> None:
    split = split_episode_ids([f"episode_{index}" for index in range(10)], seed=25012)
    sets = [set(split[name]) for name in ("train", "validation", "test")]
    assert all(sets)
    assert not (sets[0] & sets[1] or sets[0] & sets[2] or sets[1] & sets[2])
    assert len(set.union(*sets)) == 10


def test_episode_validation_rejects_unsafe_load() -> None:
    with pytest.raises(ValueError, match="load_kg"):
        EpisodeSpec("unsafe", TrajectorySpec("step", 1.0), 2.1)


def test_project_episode_configuration_loads() -> None:
    config = Path(__file__).parents[1] / "config" / "episodes.yaml"
    seed, output, episodes = load_episode_config(config)
    assert seed == 25012
    assert output.as_posix() == "data/episodes"
    assert {episode.trajectory.kind for episode in episodes} == {"step", "sine"}
    assert {episode.load_kg for episode in episodes} == {0.0, 0.5}
