"""Estimate payload lateral shift and trim stress from motor response."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class PayloadSample:
    timestamp_s: float
    left_thrust_n: float
    right_thrust_n: float
    roll_correction_deg: float
    sway_rate_mps: float
    payload_mass_kg: float
    max_payload_kg: float

    def validate(self) -> None:
        if not all(math.isfinite(value) for value in asdict(self).values()):
            raise ValueError("payload sample must contain finite values")
        if self.left_thrust_n < 0.0 or self.right_thrust_n < 0.0:
            raise ValueError("motor thrust must be non-negative")
        if self.left_thrust_n + self.right_thrust_n <= 1e-9:
            raise ValueError("total thrust must be positive")
        if self.payload_mass_kg < 0.0 or self.max_payload_kg <= 0.0:
            raise ValueError("payload masses are invalid")


@dataclass(frozen=True)
class PayloadBalanceConfig:
    adjust_risk: float = 0.35
    warning_risk: float = 0.60
    unstable_risk: float = 0.80
    switch_frames: int = 3
    lateral_lever_m: float = 0.42

    def __post_init__(self) -> None:
        if not 0.0 <= self.adjust_risk < self.warning_risk < self.unstable_risk <= 1.0:
            raise ValueError("risk thresholds must be strictly increasing")
        if self.switch_frames < 1 or self.lateral_lever_m <= 0.0:
            raise ValueError("invalid balance guard configuration")


@dataclass(frozen=True)
class PayloadAssessment:
    state: str
    risk: float
    thrust_imbalance_ratio: float
    estimated_cg_offset_m: float
    roll_correction_deg: float
    sway_rate_mps: float
    payload_ratio: float
    recommendation: str

    def to_dict(self) -> dict:
        return asdict(self)


def _clip(value: float) -> float:
    return max(0.0, min(1.0, value))


class PayloadBalanceGuard:
    def __init__(self, config: PayloadBalanceConfig | None = None) -> None:
        self.config = config or PayloadBalanceConfig()
        self.last_timestamp: float | None = None
        self.state = "BALANCED"
        self.candidate = "BALANCED"
        self.candidate_frames = 0

    def evaluate(self, sample: PayloadSample) -> PayloadAssessment:
        sample.validate()
        if self.last_timestamp is not None and sample.timestamp_s <= self.last_timestamp:
            raise ValueError("payload timestamps must increase")
        total_thrust = sample.left_thrust_n + sample.right_thrust_n
        imbalance = abs(sample.left_thrust_n - sample.right_thrust_n) / total_thrust
        signed_imbalance = (
            sample.left_thrust_n - sample.right_thrust_n
        ) / total_thrust
        estimated_offset = (
            signed_imbalance * self.config.lateral_lever_m
        )
        payload_ratio = _clip(sample.payload_mass_kg / sample.max_payload_kg)
        thrust_risk = _clip(imbalance / 0.75)
        roll_risk = _clip(abs(sample.roll_correction_deg) / 18.0)
        sway_risk = _clip(abs(sample.sway_rate_mps) / 1.6)
        payload_risk = _clip((payload_ratio - 0.65) / 0.35)
        risk = (
            0.38 * thrust_risk
            + 0.29 * roll_risk
            + 0.23 * sway_risk
            + 0.10 * payload_risk
        )
        if imbalance >= 0.85 and (
            abs(sample.roll_correction_deg) >= 14.0
            or abs(sample.sway_rate_mps) >= 1.25
        ):
            desired = "UNSTABLE"
        elif risk >= self.config.unstable_risk:
            desired = "UNSTABLE"
        elif risk >= self.config.warning_risk:
            desired = "OFFSET_WARNING"
        elif risk >= self.config.adjust_risk:
            desired = "ADJUST_TRIM"
        else:
            desired = "BALANCED"
        if desired == self.candidate:
            self.candidate_frames += 1
        else:
            self.candidate = desired
            self.candidate_frames = 1
        if self.candidate_frames >= self.config.switch_frames or desired == "UNSTABLE":
            self.state = desired
        recommendation = {
            "BALANCED": "CONTINUE",
            "ADJUST_TRIM": "REDUCE_AGILITY_AND_MONITOR",
            "OFFSET_WARNING": "HOLD_AND_REBALANCE",
            "UNSTABLE": "ABORT_MANEUVER_AND_LAND",
        }[self.state]
        self.last_timestamp = sample.timestamp_s
        return PayloadAssessment(
            state=self.state,
            risk=risk,
            thrust_imbalance_ratio=imbalance,
            estimated_cg_offset_m=estimated_offset,
            roll_correction_deg=sample.roll_correction_deg,
            sway_rate_mps=sample.sway_rate_mps,
            payload_ratio=payload_ratio,
            recommendation=recommendation,
        )
