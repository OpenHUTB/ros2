"""Episode-level dataset schema, split assignment, and CSV writing."""

import csv
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping, Sequence


SCHEMA_VERSION = '1.0'
EPISODE_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$')


@dataclass(frozen=True)
class DatasetSample:
    """One synchronized observation used by later slip-learning work."""

    timestamp_ns: int
    episode_id: str
    split: str
    friction_coefficient: float
    command_linear_x: float
    command_angular_z: float
    wheel_position_left_front: float
    wheel_position_left_rear: float
    wheel_position_right_front: float
    wheel_position_right_rear: float
    wheel_velocity_left_front: float
    wheel_velocity_left_rear: float
    wheel_velocity_right_front: float
    wheel_velocity_right_rear: float
    imu_acceleration_x: float
    imu_acceleration_y: float
    imu_acceleration_z: float
    imu_angular_velocity_x: float
    imu_angular_velocity_y: float
    imu_angular_velocity_z: float
    ground_truth_x: float
    ground_truth_y: float
    ground_truth_yaw: float
    ground_truth_linear_x: float
    ground_truth_angular_z: float


def assign_episode_split(
    episode_id: str,
    seed: int,
    ratios: Sequence[float] = (0.7, 0.15, 0.15),
) -> str:
    """Assign all frames from an episode to one stable dataset split."""
    if not EPISODE_ID_PATTERN.fullmatch(episode_id):
        raise ValueError('episode_id contains unsupported characters')
    invalid_ratio = any(
        not math.isfinite(value) or value < 0 for value in ratios
    )
    if len(ratios) != 3 or invalid_ratio:
        raise ValueError('ratios must contain three finite non-negative values')
    total = sum(ratios)
    if not math.isclose(total, 1.0, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError('split ratios must sum to one')
    digest = hashlib.sha256(f'{seed}:{episode_id}'.encode('utf-8')).digest()
    value = int.from_bytes(digest[:8], 'big') / float(2**64)
    if value < ratios[0]:
        return 'train'
    if value < ratios[0] + ratios[1]:
        return 'validation'
    return 'test'


class EpisodeCsvWriter:
    """Write one CSV and one reproducibility manifest per complete episode."""

    def __init__(
        self,
        output_directory: Path,
        episode_id: str,
        split_seed: int,
        metadata: Mapping[str, Any],
        flush_interval: int = 100,
    ) -> None:
        if not EPISODE_ID_PATTERN.fullmatch(episode_id):
            raise ValueError('episode_id contains unsupported characters')
        if flush_interval <= 0:
            raise ValueError('flush_interval must be positive')
        self.episode_id = episode_id
        self.split = assign_episode_split(episode_id, split_seed)
        self._output_directory = output_directory.resolve()
        self._output_directory.mkdir(parents=True, exist_ok=True)
        self.csv_path = self._output_directory / f'{episode_id}.csv'
        self.manifest_path = self._output_directory / f'{episode_id}.json'
        self._handle = self.csv_path.open('w', encoding='utf-8', newline='')
        fieldnames = list(DatasetSample.__dataclass_fields__)
        self._writer = csv.DictWriter(self._handle, fieldnames=fieldnames)
        self._writer.writeheader()
        self._flush_interval = flush_interval
        self._rows = 0
        self._metadata = dict(metadata)
        self._closed = False

    def write(self, sample: DatasetSample) -> None:
        """Append a validated sample belonging to this episode and split."""
        if self._closed:
            raise RuntimeError('cannot write to a closed episode')
        if sample.episode_id != self.episode_id or sample.split != self.split:
            raise ValueError('sample episode or split does not match writer')
        if sample.timestamp_ns < 0:
            raise ValueError('timestamp_ns must be non-negative')
        values = asdict(sample)
        numeric_values = (
            value
            for name, value in values.items()
            if name not in {'episode_id', 'split', 'timestamp_ns'}
        )
        if any(not math.isfinite(value) for value in numeric_values):
            raise ValueError('sample numeric values must be finite')
        if sample.friction_coefficient <= 0.0:
            raise ValueError('friction_coefficient must be positive')
        self._writer.writerow(values)
        self._rows += 1
        if self._rows % self._flush_interval == 0:
            self._handle.flush()

    def close(self) -> None:
        """Close the CSV and write its manifest exactly once."""
        if self._closed:
            return
        self._handle.flush()
        self._handle.close()
        manifest = {
            'schema_version': SCHEMA_VERSION,
            'episode_id': self.episode_id,
            'split': self.split,
            'rows': self._rows,
            **self._metadata,
        }
        self.manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        self._closed = True

    def __enter__(self) -> 'EpisodeCsvWriter':
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()
