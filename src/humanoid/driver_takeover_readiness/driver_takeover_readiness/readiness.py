"""Explainable takeover-readiness score.

The score is a small human-factors prototype: eyes on road, hands on wheel,
feet ready for pedals, and elapsed time after a takeover request. It is not a
medical or production safety model.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TakeoverReadinessPolicy:
    gaze_weight: float = 0.35
    hands_weight: float = 0.25
    feet_weight: float = 0.20
    alert_weight: float = 0.20
    ready_threshold: float = 0.70
    critical_gaze_off_s: float = 2.5

    def validate(self) -> None:
        weights = [
            self.gaze_weight,
            self.hands_weight,
            self.feet_weight,
            self.alert_weight,
        ]
        if any(weight < 0.0 for weight in weights):
            raise ValueError("weights must be non-negative")
        if abs(sum(weights) - 1.0) > 1e-6:
            raise ValueError("weights must sum to 1.0")
        if not 0.0 <= self.ready_threshold <= 1.0:
            raise ValueError("ready_threshold must be in [0, 1]")
        if self.critical_gaze_off_s < 0.0:
            raise ValueError("critical_gaze_off_s must be non-negative")

    def evaluate(
        self,
        gaze_on_road: bool,
        hands_on_wheel: bool,
        feet_on_pedals: bool,
        alert_age_s: float,
        gaze_off_road_s: float,
    ) -> "TakeoverReadiness":
        self.validate()
        if alert_age_s < 0.0 or gaze_off_road_s < 0.0:
            raise ValueError("time values must be non-negative")

        gaze_score = 1.0 if gaze_on_road and gaze_off_road_s < 1.0 else max(
            0.0, 1.0 - gaze_off_road_s / max(self.critical_gaze_off_s, 1e-6)
        )
        alert_score = min(1.0, alert_age_s / 1.5)
        score = (
            self.gaze_weight * gaze_score
            + self.hands_weight * (1.0 if hands_on_wheel else 0.0)
            + self.feet_weight * (1.0 if feet_on_pedals else 0.0)
            + self.alert_weight * alert_score
        )
        critical = (not gaze_on_road and gaze_off_road_s >= self.critical_gaze_off_s) or not hands_on_wheel
        ready = score >= self.ready_threshold and not critical
        reasons = []
        if not gaze_on_road:
            reasons.append("gaze_off_road")
        if not hands_on_wheel:
            reasons.append("hands_off_wheel")
        if not feet_on_pedals:
            reasons.append("feet_not_ready")
        if alert_age_s < 1.0:
            reasons.append("orientation_time_short")
        if critical:
            reasons.insert(0, "critical_condition")

        level = "ready" if ready else ("critical" if critical else "attention")
        return TakeoverReadiness(
            score=round(score, 4),
            ready=ready,
            level=level,
            reasons=reasons or ["ready"],
        )


@dataclass(frozen=True)
class TakeoverReadiness:
    score: float
    ready: bool
    level: str
    reasons: list[str]

    def as_dict(self) -> dict:
        return {
            "score": self.score,
            "ready": self.ready,
            "level": self.level,
            "reasons": self.reasons,
        }
