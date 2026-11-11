import csv
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from mujoco_musculoskeletal_elbow_nn_control import collector as collector_module
from mujoco_musculoskeletal_elbow_nn_control.episode import (
    CSV_FIELDS,
    EpisodeCollector,
    EpisodeSpec,
    load_episode_config,
    split_episode_ids,
    write_episode_split,
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


def test_split_is_order_independent_and_keeps_small_partitions_non_empty() -> None:
    ids = ["step_load_0p0", "step_load_0p5", "sine_load_0p0"]
    first = split_episode_ids(ids, seed=12, ratios=(0.98, 0.01, 0.01))
    second = split_episode_ids(reversed(ids), seed=12, ratios=(0.98, 0.01, 0.01))

    assert first == second
    assert all(first[name] for name in ("train", "validation", "test"))


@pytest.mark.parametrize(
    ("content", "field"),
    [
        ("seed: null\nepisodes: []\n", "seed"),
        (
            "episodes:\n  - id: sample\n    duration_s: 1\n    load_kg: 0\n",
            "trajectory",
        ),
        (
            "episodes:\n  - trajectory: step\n    duration_s: 1\n    load_kg: 0\n",
            "id",
        ),
    ],
)
def test_invalid_episode_config_names_the_field(
    tmp_path: Path, content: str, field: str
) -> None:
    config = tmp_path / "episodes.yaml"
    config.write_text(content, encoding="utf-8")

    with pytest.raises(ValueError, match=field):
        load_episode_config(config)


def test_episode_validation_rejects_unsafe_load() -> None:
    with pytest.raises(ValueError, match="load_kg"):
        EpisodeSpec("unsafe", TrajectorySpec("step", 1.0), 2.1)


def test_trajectory_bounds_are_checked_against_the_joint() -> None:
    TrajectorySpec("sine", duration_s=2.0, offset_rad=0.8).validate_joint_range(
        0.05, 2.2
    )
    with pytest.raises(ValueError, match="outside joint range"):
        TrajectorySpec("sine", duration_s=2.0, offset_rad=2.0).validate_joint_range(
            0.05, 2.2
        )


def test_collector_stops_if_simulation_time_does_not_advance(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    observation = SimpleNamespace(
        time_s=0.0,
        angle_rad=0.35,
        angular_velocity_rad_s=0.0,
        flexor_force_n=0.0,
        extensor_force_n=0.0,
    )

    class FrozenSimulation:
        joint_range_rad = (0.05, 2.2)
        timestep_s = 0.5

        def __init__(self) -> None:
            self.step_count = 0
            self.stopped = False

        def reset(self, _angle: float):
            return observation

        def step(self, _activations):
            self.step_count += 1
            return observation

        def safe_stop(self) -> None:
            self.stopped = True

    simulation = FrozenSimulation()
    monkeypatch.setattr(
        "mujoco_musculoskeletal_elbow_nn_control.episode.ElbowSimulation",
        lambda *_args, **_kwargs: simulation,
    )
    spec = EpisodeSpec("frozen", TrajectorySpec("step", duration_s=1.0), 0.0)

    with pytest.raises(RuntimeError, match="step limit"):
        EpisodeCollector(Path("unused.xml")).run(spec)

    assert simulation.step_count == 3
    assert simulation.stopped


def test_collector_persists_split_manifest_and_episode_files(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config = tmp_path / "episodes.yaml"
    config.write_text(
        "seed: 17\n"
        "episodes:\n"
        "  - id: a\n    trajectory: step\n    duration_s: 1\n    load_kg: 0\n"
        "  - id: b\n    trajectory: step\n    duration_s: 1\n    load_kg: 0\n"
        "  - id: c\n    trajectory: sine\n    duration_s: 1\n    load_kg: 0\n",
        encoding="utf-8",
    )
    output = tmp_path / "episodes"

    class SampleCollector:
        def __init__(self, _model_path: Path) -> None:
            pass

        def run(self, spec: EpisodeSpec):
            return [{"episode_id": spec.episode_id, "step": 0}]

    monkeypatch.setattr(collector_module, "EpisodeCollector", SampleCollector)
    collector_module.main(
        ["--model", "model.xml", "--config", str(config), "--output", str(output)]
    )

    manifest = json.loads((output / "episode_split.json").read_text(encoding="utf-8"))
    assert manifest["seed"] == 17
    assert all(manifest["partitions"][name] for name in ("train", "validation", "test"))
    for episode_id in ("a", "b", "c"):
        csv_path = output / f"{episode_id}.csv"
        with csv_path.open("r", newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            assert tuple(reader.fieldnames or ()) == CSV_FIELDS
            assert len(list(reader)) == 1
        metadata = json.loads(
            (output / f"{episode_id}.json").read_text(encoding="utf-8")
        )
        assert metadata["episode"]["id"] == episode_id
        assert metadata["seed"] == 17
        assert metadata["sample_count"] == 1


def test_split_manifest_rejects_overlapping_episodes(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="disjoint"):
        write_episode_split(
            tmp_path,
            1,
            {"train": ["a"], "validation": ["a"], "test": ["c"]},
        )


def test_project_episode_configuration_loads() -> None:
    config = Path(__file__).parents[1] / "config" / "episodes.yaml"
    seed, output, episodes = load_episode_config(config)
    assert seed == 25012
    assert output.as_posix() == "data/episodes"
    assert {episode.trajectory.kind for episode in episodes} == {"step", "sine"}
    assert {episode.load_kg for episode in episodes} == {0.0, 0.5}
