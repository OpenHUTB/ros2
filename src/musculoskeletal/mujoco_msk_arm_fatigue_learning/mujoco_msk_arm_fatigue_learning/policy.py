"""Serializable learning-policy parameters."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from math import isfinite
from pathlib import Path


@dataclass(frozen=True)
class LearningPolicy:
    learning_rate: float = 100.0
    gain_decay_rate: float = 0.20
    maximum_gain: float = 2.0
    activation_deadband: float = 0.02
    error_deadband_rad: float = 0.001
    error_ceiling_rad: float = 0.30

    def __post_init__(self) -> None:
        values = vars(self).values()
        if not all(isfinite(value) for value in values):
            raise ValueError('learning policy values must be finite')
        if self.learning_rate < 0.0 or self.gain_decay_rate < 0.0:
            raise ValueError('learning and decay rates must be non-negative')
        if self.maximum_gain < 1.0:
            raise ValueError('maximum gain must be at least 1')
        if self.activation_deadband < 0.0:
            raise ValueError('activation deadband must be non-negative')
        if not (
            0.0
            <= self.error_deadband_rad
            < self.error_ceiling_rad
        ):
            raise ValueError('error deadbands must be ordered')

    def save(self, path: Path) -> None:
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(asdict(self), indent=2) + '\n',
            encoding='utf-8',
        )

    @classmethod
    def load(cls, path: Path) -> 'LearningPolicy':
        data = json.loads(Path(path).expanduser().read_text(encoding='utf-8'))
        if not isinstance(data, dict):
            raise ValueError('learning policy JSON must contain an object')
        allowed = set(cls.__dataclass_fields__)
        unknown = set(data) - allowed
        if unknown:
            raise ValueError(
                'unknown learning policy keys: %s'
                % ', '.join(sorted(unknown))
            )
        return cls(**data)
