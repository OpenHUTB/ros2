"""Explainable control authority and actuator saturation risk assessment."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class ControlSample:
    timestamp_s: float
    commanded_roll_rate_deg_s: float
    actual_roll_rate_deg_s: float
    commanded_pitch_rate_deg_s: float
    actual_pitch_rate_deg_s: float
    motor_saturation_ratio: float
    integrator_ratio: float
    min_cell_voltage_v: float

    def validate(self) -> None:
        if not all(math.isfinite(value) for value in asdict(self).values()):
            raise ValueError("control sample must contain finite values")
        for name, value in (
            ("motor_saturation_ratio", self.motor_saturation_ratio),
            ("integrator_ratio", self.integrator_ratio),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if self.min_cell_voltage_v <= 0.0:
            raise ValueError("min_cell_voltage_v must be positive")


@dataclass(frozen=True)
class ControlAuthorityConfig:
    reduced_risk: float = 0.35
    saturation_risk: float = 0.60
    critical_risk: float = 0.80
    switch_frames: int = 3

    def __post_init__(self) -> None:
        if not 0.0 <= self.reduced_risk < self.saturation_risk < self.critical_risk <= 1.0:
            raise ValueError("risk thresholds must be strictly increasing")
        if self.switch_frames < 1:
            raise ValueError("switch_frames must be positive")


@dataclass(frozen=True)
class ControlAuthorityAssessment:
    state: str
    risk: float
    tracking_error_ratio: float
    authority_ratio: float
    saturation_ratio: float
    integrator_ratio: float
    recommendation: str

    def to_dict(self) -> dict:
        return asdict(self)


def _clip(value: float) -> float:
    return max(0.0, min(1.0, value))


class ControlAuthorityGuard:
    def __init__(self, config: ControlAuthorityConfig | None = None) -> None:
        self.config = config or ControlAuthorityConfig()
        self.last_timestamp: float | None = None
        self.state = "NOMINAL"
        self.candidate = "NOMINAL"
        self.candidate_frames = 0

    def evaluate(self, sample: ControlSample) -> ControlAuthorityAssessment:
        sample.validate()
        if self.last_timestamp is not None and sample.timestamp_s <= self.last_timestamp:
            raise ValueError("control timestamps must increase")
        command_r = math.hypot(
            sample.commanded_roll_rate_deg_s, sample.commanded_pitch_rate_deg_s
        )
        actual_r = math.hypot(
            sample.actual_roll_rate_deg_s, sample.actual_pitch_rate_deg_s
        )
        tracking_error = math.hypot(
            sample.commanded_roll_rate_deg_s - sample.actual_roll_rate_deg_s,
            sample.commanded_pitch_rate_deg_s - sample.actual_pitch_rate_deg_s,
        )
        error_ratio = _clip(tracking_error / max(5.0, command_r))
        authority_ratio = _clip(actual_r / max(5.0, command_r))
        sag_ratio = _clip((3.55 - sample.min_cell_voltage_v) / 0.9)
        risk = (
            0.36 * error_ratio
            + 0.29 * sample.motor_saturation_ratio
            + 0.20 * sample.integrator_ratio
            + 0.15 * sag_ratio
        )
        if (
            sample.motor_saturation_ratio >= 0.92
            and (error_ratio >= 0.55 or sample.integrator_ratio >= 0.85)
        ):
            desired = "CRITICAL_AUTHORITY"
        elif risk >= self.config.critical_risk:
            desired = "CRITICAL_AUTHORITY"
        elif risk >= self.config.saturation_risk:
            desired = "SATURATION_RISK"
        elif risk >= self.config.reduced_risk:
            desired = "REDUCED_AUTHORITY"
        else:
            desired = "NOMINAL"

        if desired == self.candidate:
            self.candidate_frames += 1
        else:
            self.candidate = desired
            self.candidate_frames = 1
        if self.candidate_frames >= self.config.switch_frames or desired == "CRITICAL_AUTHORITY":
            self.state = desired
        recommendation = {
            "NOMINAL": "CONTINUE",
            "REDUCED_AUTHORITY": "LIMIT_MANEUVER",
            "SATURATION_RISK": "ABORT_AGGRESSIVE_MANEUVER",
            "CRITICAL_AUTHORITY": "STABILIZE_AND_DESCEND",
        }[self.state]
        self.last_timestamp = sample.timestamp_s
        return ControlAuthorityAssessment(
            state=self.state,
            risk=risk,
            tracking_error_ratio=error_ratio,
            authority_ratio=authority_ratio,
            saturation_ratio=sample.motor_saturation_ratio,
            integrator_ratio=sample.integrator_ratio,
            recommendation=recommendation,
        )
