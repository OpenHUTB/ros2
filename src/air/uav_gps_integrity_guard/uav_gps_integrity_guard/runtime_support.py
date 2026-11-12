"""ROS-independent parsing, validation, and CSV journal helpers."""

from __future__ import annotations

from dataclasses import fields
import csv
import json
import math
from pathlib import Path

from .core import GpsIntegrityAssessment, GpsSample


SAMPLE_FIELDS = tuple(field.name for field in fields(GpsSample))
CSV_COLUMNS = (
    "time_s",
    "state",
    "risk",
    "position_speed_mps",
    "speed_residual_mps",
    "imu_residual_mps",
    "accuracy_m",
    "recommendation",
)


def validate_simulator_parameters(
    rate_hz: float, duration_s: float
) -> tuple[float, float]:
    """Validate simulator parameters without silently coercing invalid values."""
    if not math.isfinite(rate_hz) or rate_hz <= 0.0:
        raise ValueError("rate_hz must be finite and greater than zero")
    if not math.isfinite(duration_s) or duration_s < 0.0:
        raise ValueError("duration_s must be finite and non-negative")
    return rate_hz, duration_s


def decode_gps_sample(raw_data: str) -> GpsSample:
    """Decode one strict JSON sample and reject missing or unknown fields."""
    try:
        payload = json.loads(raw_data)
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid GPS sample JSON: {exc.msg}") from exc
    if not isinstance(payload, dict):
        raise ValueError("GPS sample JSON must be an object")

    actual_fields = set(payload)
    expected_fields = set(SAMPLE_FIELDS)
    missing = sorted(expected_fields - actual_fields)
    unexpected = sorted(actual_fields - expected_fields)
    if missing:
        raise ValueError(f"GPS sample missing fields: {', '.join(missing)}")
    if unexpected:
        raise ValueError(f"GPS sample has unknown fields: {', '.join(unexpected)}")
    return GpsSample(**payload)


class IntegrityCsvJournal:
    """Write the published integrity assessment fields to a CSV file."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(CSV_COLUMNS)
        self._file.flush()

    def write(
        self, sample: GpsSample, assessment: GpsIntegrityAssessment
    ) -> None:
        self._writer.writerow(
            (
                f"{sample.timestamp_s:.3f}",
                assessment.state,
                f"{assessment.risk:.4f}",
                f"{assessment.position_speed_mps:.3f}",
                f"{assessment.speed_residual_mps:.3f}",
                f"{assessment.imu_residual_mps:.3f}",
                f"{assessment.accuracy_m:.3f}",
                assessment.recommendation,
            )
        )
        self._file.flush()

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()

    def __enter__(self) -> "IntegrityCsvJournal":
        return self

    def __exit__(self, *args: object) -> None:
        self.close()
