"""Reproducible RGB dataset pipeline / 可复现 RGB 数据集管线。"""

from __future__ import annotations

import csv
import hashlib
import json
import random
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml
from PIL import Image

from .expert import PathCenterExpert, SteeringLabel
from .simulation import SimulationConfig, UavSimulation


@dataclass(frozen=True)
class DatasetConfig:
    """Validated collection and split parameters / 已校验的采集与划分参数。"""

    seed: int
    episodes: int
    frames_per_episode: int
    capture_interval_steps: int
    initial_lateral_offsets_m: tuple[float, ...]
    steering_threshold: float
    stop_margin_m: float
    path_end_y_m: float
    split_ratios: dict[str, float]

    @classmethod
    def from_yaml(cls, path: Path) -> "DatasetConfig":
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, Mapping) or not isinstance(document.get("dataset"), Mapping):
            raise ValueError(f"Missing dataset mapping / 缺少 dataset 配置段: {path}")
        raw: Mapping[str, Any] = document["dataset"]
        ratios_raw = raw.get("split_ratios")
        if not isinstance(ratios_raw, Mapping):
            raise ValueError("split_ratios must be a mapping / split_ratios 必须为映射")
        ratios = {
            name: float(ratios_raw.get(name, 0.0))
            for name in ("train", "validation", "test")
        }
        if any(value <= 0.0 for value in ratios.values()) or not np.isclose(
            sum(ratios.values()), 1.0
        ):
            raise ValueError("split ratios must be positive and sum to one / 划分比例必须为正且总和为一")
        offsets_raw = raw.get("initial_lateral_offsets_m")
        if (
            not isinstance(offsets_raw, Sequence)
            or isinstance(offsets_raw, (str, bytes))
            or not offsets_raw
        ):
            raise ValueError("initial offsets cannot be empty / 初始横向偏移不能为空")
        return cls(
            seed=_positive_int(raw.get("seed"), "seed"),
            episodes=_positive_int(raw.get("episodes"), "episodes"),
            frames_per_episode=_positive_int(raw.get("frames_per_episode"), "frames_per_episode"),
            capture_interval_steps=_positive_int(
                raw.get("capture_interval_steps"), "capture_interval_steps"
            ),
            initial_lateral_offsets_m=tuple(float(value) for value in offsets_raw),
            steering_threshold=_range_float(
                raw.get("steering_threshold"), "steering_threshold", 0.0, 1.0
            ),
            stop_margin_m=_range_float(
                raw.get("stop_margin_m"), "stop_margin_m", 0.0, float("inf")
            ),
            path_end_y_m=float(raw.get("path_end_y_m")),
            split_ratios=ratios,
        )


@dataclass(frozen=True)
class FrameRecord:
    """One image and its deterministic supervision / 单张图像及确定性监督信息。"""

    image_path: str
    episode_id: int
    frame_id: int
    split: str
    label: str
    steering: float
    lateral_error_m: float
    heading_error_rad: float
    x_m: float
    y_m: float
    z_m: float
    yaw_radians: float
    sha256: str


def deterministic_episode_splits(config: DatasetConfig) -> dict[int, str]:
    """Assign whole episodes to splits / 按完整回合划分数据集。"""
    episode_ids = list(range(config.episodes))
    random.Random(config.seed).shuffle(episode_ids)
    train_count = round(config.episodes * config.split_ratios["train"])
    validation_count = round(config.episodes * config.split_ratios["validation"])
    train_count = min(max(1, train_count), config.episodes - 2)
    validation_count = min(max(1, validation_count), config.episodes - train_count - 1)
    splits: dict[int, str] = {}
    for index, episode_id in enumerate(episode_ids):
        if index < train_count:
            splits[episode_id] = "train"
        elif index < train_count + validation_count:
            splits[episode_id] = "validation"
        else:
            splits[episode_id] = "test"
    return splits


def collect_dataset(
    simulation_config: SimulationConfig,
    dataset_config: DatasetConfig,
    model_path: Path,
    output_dir: Path,
) -> dict[str, Any]:
    """Collect images and metadata into an atomic directory layout / 采集图像与元数据。"""
    if dataset_config.episodes < 3:
        raise ValueError("at least three episodes are required / 至少需要三个回合")
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    images_dir = output_dir / "images"
    images_dir.mkdir(exist_ok=True)
    split_map = deterministic_episode_splits(dataset_config)
    expert = PathCenterExpert(
        path_half_width_m=simulation_config.path_half_width_m,
        steering_threshold=dataset_config.steering_threshold,
        stop_margin_m=dataset_config.stop_margin_m,
        path_end_y_m=dataset_config.path_end_y_m,
    )
    records: list[FrameRecord] = []
    offset_rng = random.Random(dataset_config.seed)
    with UavSimulation(model_path, simulation_config) as simulation:
        for episode_id in range(dataset_config.episodes):
            base_offset = dataset_config.initial_lateral_offsets_m[
                episode_id % len(dataset_config.initial_lateral_offsets_m)
            ]
            offset = base_offset + offset_rng.uniform(-0.025, 0.025)
            simulation.reset(offset)
            for frame_id in range(dataset_config.frames_per_episode):
                state = simulation.state()
                decision = expert.decide(state["x_m"], state["y_m"], state["yaw_radians"])
                frame = simulation.capture_rgb()
                relative_path = Path("images") / (
                    f"episode_{episode_id:03d}_frame_{frame_id:04d}.png"
                )
                absolute_path = output_dir / relative_path
                Image.fromarray(frame, mode="RGB").save(absolute_path, format="PNG", optimize=True)
                records.append(
                    FrameRecord(
                        image_path=relative_path.as_posix(),
                        episode_id=episode_id,
                        frame_id=frame_id,
                        split=split_map[episode_id],
                        label=decision.label.value,
                        steering=decision.steering,
                        lateral_error_m=decision.lateral_error_m,
                        heading_error_rad=decision.heading_error_rad,
                        x_m=state["x_m"],
                        y_m=state["y_m"],
                        z_m=state["z_m"],
                        yaw_radians=state["yaw_radians"],
                        sha256=hashlib.sha256(absolute_path.read_bytes()).hexdigest(),
                    )
                )
                if decision.label is SteeringLabel.STOP:
                    break
                for _ in range(dataset_config.capture_interval_steps):
                    simulation.step(decision.steering)

    metadata_path = output_dir / "metadata.csv"
    fieldnames = list(asdict(records[0]).keys())
    with metadata_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(asdict(record) for record in records)
    manifest = {
        "schema_version": 1,
        "seed": dataset_config.seed,
        "episodes": dataset_config.episodes,
        "frames": len(records),
        "image_format": "PNG RGB8",
        "image_size": [simulation_config.render_width, simulation_config.render_height],
        "episode_splits": {str(key): value for key, value in sorted(split_map.items())},
    }
    _write_json(output_dir / "manifest.json", manifest)
    quality = inspect_dataset(output_dir)
    _write_json(output_dir / "quality_report.json", quality)
    return quality


def inspect_dataset(dataset_dir: Path) -> dict[str, Any]:
    """Inspect paths, images, hashes, labels, and split leakage / 检查路径、图像、哈希、标签及划分泄漏。"""
    metadata_path = dataset_dir / "metadata.csv"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"metadata.csv not found / 找不到 metadata.csv: {dataset_dir}")
    with metadata_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("dataset metadata is empty / 数据集元数据为空")
    labels = Counter(row["label"] for row in rows)
    splits = Counter(row["split"] for row in rows)
    missing_paths: list[str] = []
    invalid_images: list[str] = []
    hash_mismatches: list[str] = []
    hashes: Counter[str] = Counter()
    episode_split_sets: dict[str, set[str]] = {}
    for row in rows:
        image_path = dataset_dir / row["image_path"]
        episode_split_sets.setdefault(row["episode_id"], set()).add(row["split"])
        if not image_path.is_file():
            missing_paths.append(row["image_path"])
            continue
        digest = hashlib.sha256(image_path.read_bytes()).hexdigest()
        hashes[digest] += 1
        if digest != row["sha256"]:
            hash_mismatches.append(row["image_path"])
        try:
            with Image.open(image_path) as image:
                image.verify()
            with Image.open(image_path) as image:
                if image.mode != "RGB":
                    invalid_images.append(row["image_path"])
        except (OSError, ValueError):
            invalid_images.append(row["image_path"])
    leakage = sorted(episode for episode, values in episode_split_sets.items() if len(values) != 1)
    expected_labels = {label.value for label in SteeringLabel}
    missing_labels = sorted(expected_labels - set(labels))
    duplicate_groups = sum(1 for count in hashes.values() if count > 1)
    passed = not any((missing_paths, invalid_images, hash_mismatches, leakage, missing_labels))
    return {
        "status": "passed" if passed else "failed",
        "status_zh": "通过" if passed else "未通过",
        "records": len(rows),
        "label_counts": dict(sorted(labels.items())),
        "split_counts": dict(sorted(splits.items())),
        "missing_paths": missing_paths,
        "invalid_images": invalid_images,
        "hash_mismatches": hash_mismatches,
        "episode_split_leakage": leakage,
        "missing_labels": missing_labels,
        "duplicate_hash_groups": duplicate_groups,
    }


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    text = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    path.write_text(text, encoding="utf-8")


def _positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer / {name} 必须为正整数")
    return value


def _range_float(value: Any, name: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric / {name} 必须为数值")
    result = float(value)
    if not minimum <= result < maximum:
        raise ValueError(f"{name} is outside its valid range / {name} 超出有效范围")
    return result

