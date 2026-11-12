import csv
import json

import pytest

from mujoco_musculoskeletal_elbow_nn_control.train import train_model


FIELDS = [
    "episode_id",
    "target_angle_rad",
    "angle_rad",
    "angular_velocity_rad_s",
    "flexor_force_n",
    "extensor_force_n",
    "flexor_activation",
    "extensor_activation",
]


def write_episode_rows(directory, episode_ids):
    for index, episode_id in enumerate(episode_ids):
        path = directory / f"{episode_id}.csv"
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            for _ in range(8):
                writer.writerow(
                    {
                        "episode_id": episode_id,
                        "target_angle_rad": 0.8 + index * 0.01,
                        "angle_rad": 0.7 + index * 0.01,
                        "angular_velocity_rad_s": 0.1,
                        "flexor_force_n": 1.0,
                        "extensor_force_n": 0.5,
                        "flexor_activation": 0.3,
                        "extensor_activation": 0.1,
                    }
                )


def test_train_model_persists_and_reuses_split_manifest(tmp_path):
    write_episode_rows(tmp_path, [f"episode_{index}" for index in range(5)])

    metrics = train_model(tmp_path, tmp_path / "model.json", seed=7)

    assert set(metrics["partitions"]) == {"train", "validation", "test"}
    assert (tmp_path / "model.json").exists()
    assert (tmp_path / "model.metrics.json").exists()
    manifest = json.loads(
        (tmp_path / "episode_split.json").read_text(encoding="utf-8")
    )
    assert metrics["partitions"] == manifest["partitions"]
    assert manifest["seed"] == 7


def test_training_uses_saved_episode_split(tmp_path):
    write_episode_rows(tmp_path, ("a", "b", "c"))
    partitions = {"train": ["a"], "validation": ["b"], "test": ["c"]}
    (tmp_path / "episode_split.json").write_text(
        json.dumps({"seed": 37, "partitions": partitions}), encoding="utf-8"
    )

    metrics = train_model(tmp_path, tmp_path / "model.json")

    assert metrics["partitions"] == partitions


def test_training_rejects_a_stale_split_manifest(tmp_path):
    write_episode_rows(tmp_path, ("a", "b", "c"))
    partitions = {"train": ["a"], "validation": ["b"], "test": ["missing"]}
    (tmp_path / "episode_split.json").write_text(
        json.dumps({"seed": 37, "partitions": partitions}), encoding="utf-8"
    )

    with pytest.raises(ValueError, match="do not match"):
        train_model(tmp_path, tmp_path / "model.json")
