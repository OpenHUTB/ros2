"""Dataset split and quality tests / 数据集划分与质量测试。"""

from __future__ import annotations

import csv
import sys
from dataclasses import replace
from pathlib import Path

MODULE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_ROOT))

from mujoco_abandoned_park_uav_visual_steering import (  # noqa: E402
    DatasetConfig,
    SimulationConfig,
    collect_dataset,
    deterministic_episode_splits,
    inspect_dataset,
)


def test_episode_split_is_reproducible_and_disjoint() -> None:
    config = DatasetConfig.from_yaml(MODULE_ROOT / "config" / "dataset.yaml")
    first = deterministic_episode_splits(config)
    second = deterministic_episode_splits(config)
    assert first == second
    assert set(first) == set(range(config.episodes))
    assert set(first.values()) == {"train", "validation", "test"}
    offsets_by_split: dict[float, set[str]] = {}
    for episode, split in first.items():
        offset = config.initial_lateral_offsets_m[episode % len(config.initial_lateral_offsets_m)]
        offsets_by_split.setdefault(offset, set()).add(split)
    assert all(len(values) == 1 for values in offsets_by_split.values())


def test_collection_and_quality_report(tmp_path: Path) -> None:
    config = DatasetConfig.from_yaml(MODULE_ROOT / "config" / "dataset.yaml")
    simulation_config = replace(
        SimulationConfig.from_yaml(MODULE_ROOT / "config" / "simulation.yaml"),
        render_width=64, render_height=36,
    )
    quality = collect_dataset(
        simulation_config,
        config,
        MODULE_ROOT / "models" / "scene.xml",
        tmp_path / "output",
    )
    assert quality["status"] == "passed"
    assert config.episodes <= quality["records"] <= config.episodes * config.frames_per_episode
    assert set(quality["label_counts"]) == {"LEFT", "RIGHT", "STOP", "STRAIGHT"}
    assert quality["episode_split_leakage"] == []
    assert quality["scene_group_split_leakage"] == []
    assert quality["cross_split_duplicate_hashes"] == []
    assert not any(quality["missing_labels_by_split"].values())
    assert all(set(counts) == {"LEFT", "RIGHT", "STOP", "STRAIGHT"}
               for counts in quality["split_label_counts"].values())
    assert inspect_dataset(tmp_path / "output") == quality

    metadata = tmp_path / "output" / "metadata.csv"
    with metadata.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert all(float(row["y_m"]) >= config.path_end_y_m for row in rows if row["label"] == "STOP")
    duplicate = dict(rows[0], episode_id="999", split=(
        "test" if rows[0]["split"] != "test" else "train"
    ))
    with metadata.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows + [duplicate])
    invalid = inspect_dataset(tmp_path / "output")
    assert invalid["status"] == "failed"
    assert invalid["scene_group_split_leakage"] == [duplicate["scene_group"]]
    assert invalid["cross_split_duplicate_hashes"] == [duplicate["sha256"]]

