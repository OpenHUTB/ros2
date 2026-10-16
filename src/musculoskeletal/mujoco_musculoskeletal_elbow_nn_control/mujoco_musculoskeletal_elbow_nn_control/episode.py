"""Episode configuration, collection, and on-disk schema."""

import csv
import json
import random
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List

from .controller import AntagonisticPDController, ControlState
from .simulation import ElbowSimulation
from .trajectories import TrajectorySpec


SCHEMA_VERSION = "1.0"
CSV_FIELDS = (
    "episode_id",
    "step",
    "time_s",
    "target_angle_rad",
    "angle_rad",
    "angular_velocity_rad_s",
    "flexor_force_n",
    "extensor_force_n",
    "flexor_activation",
    "extensor_activation",
    "load_kg",
)


@dataclass(frozen=True)
class EpisodeSpec:
    episode_id: str
    trajectory: TrajectorySpec
    load_kg: float

    def __post_init__(self) -> None:
        if not self.episode_id or any(
            character.isspace() for character in self.episode_id
        ):
            raise ValueError("episode id must be non-empty and contain no whitespace")
        if not 0.0 <= self.load_kg <= 2.0:
            raise ValueError("load_kg must be within [0, 2]")


def load_episode_config(path: Path) -> tuple[int, Path, List[EpisodeSpec]]:
    try:
        import yaml
    except ImportError as error:
        raise RuntimeError(
            "PyYAML is required to read episode configuration"
        ) from error
    source = Path(path).resolve()
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or not isinstance(raw.get("episodes"), list):
        raise ValueError("episode configuration must contain an episodes list")
    seed = int(raw.get("seed", 25012))
    output = Path(str(raw.get("output_directory", "data/episodes")))
    specs: List[EpisodeSpec] = []
    for item in raw["episodes"]:
        if not isinstance(item, dict):
            raise ValueError("each episode entry must be a mapping")
        kind = str(item["trajectory"])
        trajectory = TrajectorySpec(
            kind=kind,
            duration_s=float(item["duration_s"]),
            initial_angle_rad=float(item.get("initial_angle_rad", 0.35)),
            target_angle_rad=float(item.get("target_angle_rad", 1.2)),
            amplitude_rad=float(item.get("amplitude_rad", 0.45)),
            offset_rad=float(item.get("offset_rad", 0.8)),
            frequency_hz=float(item.get("frequency_hz", 0.25)),
            step_time_s=float(item.get("step_time_s", 0.5)),
        )
        specs.append(EpisodeSpec(str(item["id"]), trajectory, float(item["load_kg"])))
    if len({spec.episode_id for spec in specs}) != len(specs):
        raise ValueError("episode ids must be unique")
    return seed, output, specs


def split_episode_ids(
    episode_ids: Iterable[str],
    seed: int,
    ratios: tuple[float, float, float] = (0.6, 0.2, 0.2),
) -> Dict[str, List[str]]:
    """Split complete episodes, never adjacent frames, into data partitions."""

    ids = list(episode_ids)
    if len(ids) < 3 or len(set(ids)) != len(ids):
        raise ValueError("at least three unique episode ids are required")
    if any(ratio <= 0.0 for ratio in ratios) or abs(sum(ratios) - 1.0) > 1e-9:
        raise ValueError("split ratios must be positive and sum to one")
    random.Random(seed).shuffle(ids)
    train_end = max(1, int(len(ids) * ratios[0]))
    validation_end = max(train_end + 1, int(len(ids) * (ratios[0] + ratios[1])))
    validation_end = min(validation_end, len(ids) - 1)
    return {
        "train": ids[:train_end],
        "validation": ids[train_end:validation_end],
        "test": ids[validation_end:],
    }


class EpisodeCollector:
    def __init__(
        self, model_path: Path, controller: AntagonisticPDController | None = None
    ) -> None:
        self.model_path = Path(model_path)
        self.controller = controller or AntagonisticPDController()

    def run(self, spec: EpisodeSpec) -> List[Dict[str, Any]]:
        simulation = ElbowSimulation(self.model_path, load_kg=spec.load_kg)
        rows: List[Dict[str, Any]] = []
        observation = simulation.reset(spec.trajectory.initial_angle_rad)
        step = 0
        try:
            while observation.time_s < spec.trajectory.duration_s:
                target = spec.trajectory.sample(observation.time_s)
                activations = self.controller.command(
                    ControlState(
                        target_angle=target,
                        angle=observation.angle_rad,
                        angular_velocity=observation.angular_velocity_rad_s,
                        flexor_force=observation.flexor_force_n,
                        extensor_force=observation.extensor_force_n,
                    )
                )
                rows.append(
                    {
                        "episode_id": spec.episode_id,
                        "step": step,
                        "time_s": observation.time_s,
                        "target_angle_rad": target,
                        "angle_rad": observation.angle_rad,
                        "angular_velocity_rad_s": observation.angular_velocity_rad_s,
                        "flexor_force_n": observation.flexor_force_n,
                        "extensor_force_n": observation.extensor_force_n,
                        "flexor_activation": activations[0],
                        "extensor_activation": activations[1],
                        "load_kg": spec.load_kg,
                    }
                )
                observation = simulation.step(activations)
                step += 1
        finally:
            simulation.safe_stop()
        return rows


def write_episode(
    output_directory: Path, spec: EpisodeSpec, rows: List[Dict[str, Any]], seed: int
) -> None:
    if not rows:
        raise ValueError("an episode must contain at least one sample")
    output_directory = Path(output_directory)
    output_directory.mkdir(parents=True, exist_ok=True)
    csv_path = output_directory / f"{spec.episode_id}.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "seed": seed,
        "episode": {
            "id": spec.episode_id,
            "load_kg": spec.load_kg,
            "trajectory": asdict(spec.trajectory),
        },
        "sample_count": len(rows),
        "columns": list(CSV_FIELDS),
    }
    (output_directory / f"{spec.episode_id}.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
