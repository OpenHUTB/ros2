"""Dataset split and quality tests / 数据集划分与质量测试。"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

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


def test_collection_and_quality_report(tmp_path: Path) -> None:
    config_text = (MODULE_ROOT / "config" / "dataset.yaml").read_text(encoding="utf-8")
    config_document = yaml.safe_load(config_text)
    config_document["dataset"]["episodes"] = 7
    config_document["dataset"]["frames_per_episode"] = 2
    dataset_config_path = tmp_path / "dataset.yaml"
    dataset_config_path.write_text(
        yaml.safe_dump(config_document, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    quality = collect_dataset(
        SimulationConfig.from_yaml(MODULE_ROOT / "config" / "simulation.yaml"),
        DatasetConfig.from_yaml(dataset_config_path),
        MODULE_ROOT / "models" / "scene.xml",
        tmp_path / "output",
    )
    assert quality["status"] == "passed"
    assert 7 <= quality["records"] <= 14
    assert set(quality["label_counts"]) == {"LEFT", "RIGHT", "STOP", "STRAIGHT"}
    assert quality["episode_split_leakage"] == []
    assert inspect_dataset(tmp_path / "output") == quality

