"""Explainable confidence, latency and freshness monitoring for perception."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class PerceptionSample:
    timestamp_s: float
    frame_id: int
    target_expected: bool
    detection_count: int
    mean_confidence: float
    predictive_entropy: float
    latency_ms: float
    frame_age_ms: float
    illumination_lux: float

    def validate(self) -> None:
        if not all(math.isfinite(value) for value in asdict(self).values()):
            raise ValueError("perception sample must contain finite values")
        if self.detection_count < 0:
            raise ValueError("detection_count must be non-negative")
        if not 0.0 <= self.mean_confidence <= 1.0:
            raise ValueError("mean_confidence must be in [0, 1]")
        if self.predictive_entropy < 0.0:
            raise ValueError("predictive_entropy must be non-negative")
        if self.latency_ms < 0.0 or self.frame_age_ms < 0.0:
            raise ValueError("latency and age must be non-negative")
        if self.illumination_lux < 0.0:
            raise ValueError("illumination_lux must be non-negative")


@dataclass(frozen=True)
class PerceptionGuardConfig:
    degraded_risk: float = 0.35
    unreliable_risk: float = 0.62
    blind_risk: float = 0.82
    switch_frames: int = 3

    def __post_init__(self) -> None:
        if not 0.0 <= self.degraded_risk < self.unreliable_risk < self.blind_risk <= 1.0:
            raise ValueError("risk thresholds must be strictly increasing")
        if self.switch_frames < 1:
            raise ValueError("switch_frames must be positive")


@dataclass(frozen=True)
class PerceptionAssessment:
    state: str
    risk: float
    confidence_deficit: float
    entropy_risk: float
    latency_risk: float
    freshness_risk: float
    recommendation: str

    def to_dict(self) -> dict:
        return asdict(self)


def _clip(value: float) -> float:
    return max(0.0, min(1.0, value))


class PerceptionConfidenceGuard:
    def __init__(self, config: PerceptionGuardConfig | None = None) -> None:
        self.config = config or PerceptionGuardConfig()
        self.last_timestamp: float | None = None
        self.state = "NOMINAL"
        self.candidate = "NOMINAL"
        self.candidate_frames = 0

    def evaluate(self, sample: PerceptionSample) -> PerceptionAssessment:
        sample.validate()
        if self.last_timestamp is not None and sample.timestamp_s <= self.last_timestamp:
            raise ValueError("perception timestamps must increase")
        missing_expected = sample.target_expected and sample.detection_count == 0
        confidence_deficit = _clip(0.68 - sample.mean_confidence)
        entropy_risk = _clip((sample.predictive_entropy - 0.55) / 1.25)
        latency_risk = _clip((sample.latency_ms - 80.0) / 420.0)
        freshness_risk = _clip((sample.frame_age_ms - 250.0) / 1250.0)
        illumination_risk = _clip((80.0 - sample.illumination_lux) / 70.0)
        missing_risk = 0.55 if missing_expected else 0.0
        risk = (
            0.30 * confidence_deficit
            + 0.22 * entropy_risk
            + 0.20 * latency_risk
            + 0.16 * freshness_risk
            + 0.07 * illumination_risk
            + 0.05 * missing_risk
        )
        if missing_expected or sample.frame_age_ms >= 1400.0 or sample.latency_ms >= 480.0:
            desired = "BLIND"
        elif risk >= self.config.blind_risk:
            desired = "BLIND"
        elif risk >= self.config.unreliable_risk:
            desired = "UNRELIABLE"
        elif risk >= self.config.degraded_risk:
            desired = "DEGRADED"
        else:
            desired = "NOMINAL"
        if desired == self.candidate:
            self.candidate_frames += 1
        else:
            self.candidate = desired
            self.candidate_frames = 1
        if self.candidate_frames >= self.config.switch_frames or desired == "BLIND":
            self.state = desired
        recommendation = {
            "NOMINAL": "USE_PERCEPTION",
            "DEGRADED": "INCREASE_MARGIN",
            "UNRELIABLE": "HOLD_OR_RETURN",
            "BLIND": "STOP_VISION_DEPENDENT_MOTION",
        }[self.state]
        self.last_timestamp = sample.timestamp_s
        return PerceptionAssessment(
            state=self.state,
            risk=min(risk, 1.0),
            confidence_deficit=confidence_deficit,
            entropy_risk=entropy_risk,
            latency_risk=latency_risk,
            freshness_risk=freshness_risk,
            recommendation=recommendation,
        )
