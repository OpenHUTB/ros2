"""Reproducible survey dataset generation / 可复现测量数据集生成。"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np

from .expert import ExpertConfig, ExpertInput, FlightLineExpert
from .perception import NadirSensor
from .route import RouteConfig, Waypoint, generate_lawnmower_route
from .scene import SCENARIOS, load_model, set_drone_pose


@dataclass(frozen=True)
class DatasetConfig:
    """Small reproducible collection settings / 小型可复现采集配置。"""

    frames_per_scene: int = 6
    image_width: int = 160
    image_height: int = 120
    seed: int = 20261016
    cross_track_std: float = 0.35
    heading_std: float = 0.12

    def validate(self) -> None:
        if self.frames_per_scene < 3:
            raise ValueError("frames_per_scene must be at least 3 / 每场景帧数至少为 3")
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("image dimensions must be positive / 图像尺寸必须为正数")
        if self.cross_track_std < 0.0 or self.heading_std < 0.0:
            raise ValueError("noise scales cannot be negative / 扰动尺度不能为负")


def assign_splits(sequence_ids: Iterable[str], seed: int) -> dict[str, str]:
    """Assign whole sequences to deterministic splits / 将完整序列确定性分配到各数据划分。"""
    identifiers = sorted(set(sequence_ids))
    if len(identifiers) < 3:
        raise ValueError("at least three sequences are required / 至少需要三个序列")
    generator = np.random.default_rng(seed)
    shuffled = [identifiers[index] for index in generator.permutation(len(identifiers))]
    assignments: dict[str, str] = {}
    for index, identifier in enumerate(shuffled):
        if index == 0:
            split = "validation"
        elif index == 1:
            split = "test"
        else:
            split = "train"
        assignments[identifier] = split
    return assignments


def _sample_waypoints(route: list[Waypoint], count: int) -> list[Waypoint]:
    """Interpolate along elapsed route time to avoid endpoint duplication."""
    if count < 2:
        return [route[0]]
    times = np.linspace(route[0].arrival_time, route[-1].arrival_time, count)
    samples: list[Waypoint] = []
    for sample_index, target in enumerate(times):
        right = next((i for i, point in enumerate(route) if point.arrival_time >= target), len(route) - 1)
        if right == 0:
            samples.append(route[0])
            continue
        left = route[right - 1]
        upper = route[right]
        span = max(upper.arrival_time - left.arrival_time, 1e-9)
        ratio = (target - left.arrival_time) / span
        samples.append(Waypoint(
            index=sample_index,
            x=left.x + ratio * (upper.x - left.x),
            y=left.y + ratio * (upper.y - left.y),
            z=left.z + ratio * (upper.z - left.z),
            yaw=left.yaw if ratio < 0.5 else upper.yaw,
            speed=left.speed,
            arrival_time=float(target),
            strip_index=left.strip_index if ratio < 0.5 else upper.strip_index,
            is_turn=left.is_turn or upper.is_turn,
        ))
    return samples


def _distance_to_turn(route: list[Waypoint], point: Waypoint) -> float:
    strip_end = route[point.strip_index * 2 + 1]
    return math.hypot(strip_end.x - point.x, strip_end.y - point.y)


def generate_dataset(
    output: Path,
    route_config: RouteConfig,
    dataset_config: DatasetConfig,
    scenarios: Iterable[str] = SCENARIOS,
) -> dict[str, object]:
    """Generate aligned frames and expert labels / 生成对齐影像与专家控制标签。"""
    route_config.validate()
    dataset_config.validate()
    scenario_names = tuple(scenarios)
    unknown = set(scenario_names) - set(SCENARIOS)
    if unknown:
        raise ValueError(f"unknown scenarios / 未知场景: {sorted(unknown)}")
    sequence_ids = [f"{name}_sequence_000" for name in scenario_names]
    assignments = assign_splits(sequence_ids, dataset_config.seed)
    route = generate_lawnmower_route(route_config)
    expert = FlightLineExpert(ExpertConfig(cruise_speed=route_config.speed))
    output.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, object]] = []

    for scenario_index, (scenario, sequence_id) in enumerate(zip(scenario_names, sequence_ids)):
        generator = np.random.default_rng(dataset_config.seed + scenario_index)
        model, data = load_model(scenario)
        sampled = _sample_waypoints(route, dataset_config.frames_per_scene)
        with NadirSensor(model, dataset_config.image_width, dataset_config.image_height) as sensor:
            for frame_index, waypoint in enumerate(sampled):
                cross_track = float(generator.normal(0.0, dataset_config.cross_track_std))
                heading_error = float(generator.normal(0.0, dataset_config.heading_std))
                x_position = waypoint.x - math.sin(waypoint.yaw) * cross_track
                y_position = waypoint.y + math.cos(waypoint.yaw) * cross_track
                yaw = waypoint.yaw + heading_error
                set_drone_pose(model, data, x_position, y_position, waypoint.z, yaw)
                frame = sensor.capture(data)
                expert_input = ExpertInput(
                    cross_track_error=cross_track,
                    heading_error=heading_error,
                    distance_to_turn=_distance_to_turn(route, waypoint),
                )
                command = expert.command(expert_input)
                stem = f"{sequence_id}_{frame_index:04d}"
                rgb_relative = Path("images") / assignments[sequence_id] / f"{stem}.png"
                mask_relative = Path("masks") / assignments[sequence_id] / f"{stem}.png"
                (output / rgb_relative).parent.mkdir(parents=True, exist_ok=True)
                (output / mask_relative).parent.mkdir(parents=True, exist_ok=True)
                frame.rgb.save(output / rgb_relative, format="PNG", optimize=True)
                frame.mask.save(output / mask_relative, format="PNG", optimize=True)
                records.append(
                    {
                        "schema_version": 1,
                        "frame_id": stem,
                        "scenario": scenario,
                        "sequence_id": sequence_id,
                        "frame_index": frame_index,
                        "split": assignments[sequence_id],
                        "rgb_path": rgb_relative.as_posix(),
                        "mask_path": mask_relative.as_posix(),
                        "pose": {"x": x_position, "y": y_position, "z": waypoint.z, "yaw": yaw},
                        "expert_input": asdict(expert_input),
                        "expert_command": asdict(command),
                        "pv_pixels": frame.pv_pixels,
                        "total_pixels": frame.total_pixels,
                        "pv_pixel_ratio": frame.pv_pixels / frame.total_pixels,
                    }
                )

    manifest_path = output / "manifest.jsonl"
    manifest_path.write_text(
        "".join(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n" for record in records),
        encoding="utf-8",
    )
    split_path = output / "splits.json"
    split_path.write_text(
        json.dumps({"seed": dataset_config.seed, "assignments": assignments}, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return {
        "schema_version": 1,
        "stage": "dataset_generation",
        "seed": dataset_config.seed,
        "frame_count": len(records),
        "sequence_count": len(sequence_ids),
        "scenario_count": len(scenario_names),
        "split_frame_counts": {
            split: sum(record["split"] == split for record in records)
            for split in ("train", "validation", "test")
        },
        "manifest": manifest_path.name,
        "splits": split_path.name,
    }
