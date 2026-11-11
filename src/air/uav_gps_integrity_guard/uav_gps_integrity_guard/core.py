"""Explainable GPS integrity checks using motion and IMU consistency."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class GpsSample:
    timestamp_s: float
    x_m: float
    y_m: float
    z_m: float
    reported_speed_mps: float
    horizontal_accuracy_m: float
    imu_speed_mps: float

    def validate(self) -> None:
        if not all(math.isfinite(value) for value in asdict(self).values()):
            raise ValueError("GPS sample must contain finite values")
        if self.reported_speed_mps < 0.0 or self.imu_speed_mps < 0.0:
            raise ValueError("speeds must be non-negative")
        if self.horizontal_accuracy_m <= 0.0:
            raise ValueError("horizontal accuracy must be positive")


@dataclass(frozen=True)
class GpsIntegrityConfig:
    suspect_risk: float = 0.35
    reject_risk: float = 0.68
    switch_frames: int = 2

    def __post_init__(self) -> None:
        if not 0.0 <= self.suspect_risk < self.reject_risk <= 1.0:
            raise ValueError("risk thresholds must satisfy 0 <= suspect < reject <= 1")
        if self.switch_frames < 1:
            raise ValueError("switch_frames must be positive")


@dataclass(frozen=True)
class GpsIntegrityAssessment:
    state: str
    risk: float
    position_speed_mps: float
    speed_residual_mps: float
    imu_residual_mps: float
    accuracy_m: float
    recommendation: str

    def to_dict(self) -> dict:
        return asdict(self)


def _clip(value: float) -> float:
    return max(0.0, min(1.0, value))


class GpsIntegrityGuard:
    def __init__(self, config: GpsIntegrityConfig | None = None) -> None:
        self.config = config or GpsIntegrityConfig()
        self.previous: GpsSample | None = None
        self.state = "VALID"
        self.candidate = "VALID"
        self.candidate_frames = 0

    def evaluate(self, sample: GpsSample) -> GpsIntegrityAssessment:
        sample.validate()
        dt = 0.0 if self.previous is None else sample.timestamp_s - self.previous.timestamp_s
        if self.previous is not None and dt <= 0.0:
            raise ValueError("GPS timestamps must increase")
        if self.previous is None:
            position_speed = sample.reported_speed_mps
        else:
            dx = sample.x_m - self.previous.x_m
            dy = sample.y_m - self.previous.y_m
            position_speed = math.hypot(dx, dy) / dt
        speed_residual = abs(position_speed - sample.reported_speed_mps)
        imu_residual = abs(sample.imu_speed_mps - sample.reported_speed_mps)
        jump = position_speed > max(10.0, sample.imu_speed_mps + 5.0)
        risk = (
            0.34 * _clip(speed_residual / 8.0)
            + 0.28 * _clip(imu_residual / 7.0)
            + 0.23 * _clip((sample.horizontal_accuracy_m - 1.2) / 4.8)
            + 0.15 * (_clip((position_speed - 12.0) / 20.0) if jump else 0.0)
        )
        if jump or sample.horizontal_accuracy_m > 6.0:
            desired = "REJECTED"
        elif risk >= self.config.reject_risk:
            desired = "REJECTED"
        elif risk >= self.config.suspect_risk:
            desired = "SUSPECT"
        else:
            desired = "VALID"

        if desired == self.candidate:
            self.candidate_frames += 1
        else:
            self.candidate = desired
            self.candidate_frames = 1
        if self.candidate_frames >= self.config.switch_frames or desired == "REJECTED":
            self.state = desired
        recommendation = {
            "VALID": "USE_GPS",
            "SUSPECT": "CROSS_CHECK",
            "REJECTED": "REJECT_GPS_USE_DEADRECKONING",
        }[self.state]
        self.previous = sample
        return GpsIntegrityAssessment(
            state=self.state,
            risk=risk,
            position_speed_mps=position_speed,
            speed_residual_mps=speed_residual,
            imu_residual_mps=imu_residual,
            accuracy_m=sample.horizontal_accuracy_m,
            recommendation=recommendation,
        )
