"""Explainable link-quality scoring and hysteretic protection states."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import math
import statistics


@dataclass(frozen=True)
class LinkSample:
    timestamp_s: float
    rssi_dbm: float
    latency_ms: float
    packet_loss: float
    age_ms: float

    def validate(self) -> None:
        values = asdict(self)
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError("link sample must contain finite values")
        if self.rssi_dbm > 0.0:
            raise ValueError("rssi_dbm must be non-positive")
        if self.latency_ms < 0.0 or self.age_ms < 0.0:
            raise ValueError("latency and age must be non-negative")
        if not 0.0 <= self.packet_loss <= 1.0:
            raise ValueError("packet_loss must be in [0, 1]")


@dataclass(frozen=True)
class LinkGuardConfig:
    degraded_risk: float = 0.35
    hold_risk: float = 0.62
    return_risk: float = 0.78
    switch_frames: int = 3
    stale_age_ms: float = 1600.0
    trend_window: int = 20

    def __post_init__(self) -> None:
        if not 0.0 <= self.degraded_risk < self.hold_risk < self.return_risk <= 1.0:
            raise ValueError("risk thresholds must be strictly increasing in [0, 1]")
        if self.switch_frames < 1 or self.trend_window < 3:
            raise ValueError("switch_frames and trend_window are too small")


@dataclass(frozen=True)
class LinkAssessment:
    state: str
    risk: float
    rssi_ewma: float
    latency_ewma: float
    loss_ewma: float
    trend_db_per_s: float
    time_to_horizon_s: float | None
    recommendation: str
    candidate_state: str

    def to_dict(self) -> dict:
        return asdict(self)


def _clip(value: float) -> float:
    return max(0.0, min(1.0, value))


class LinkQualityGuard:
    """Combines absolute quality, freshness and RSSI trend with state hysteresis."""

    def __init__(self, config: LinkGuardConfig | None = None) -> None:
        self.config = config or LinkGuardConfig()
        self.history: deque[LinkSample] = deque(maxlen=self.config.trend_window)
        self.state = "HEALTHY"
        self.candidate_state = "HEALTHY"
        self.candidate_count = 0
        self.rssi_ewma: float | None = None
        self.latency_ewma: float | None = None
        self.loss_ewma: float | None = None

    def _trend(self) -> float:
        if len(self.history) < 3:
            return 0.0
        times = [sample.timestamp_s for sample in self.history]
        rssi = [sample.rssi_dbm for sample in self.history]
        mean_t = statistics.fmean(times)
        mean_r = statistics.fmean(rssi)
        denominator = sum((value - mean_t) ** 2 for value in times)
        if denominator <= 1e-12:
            return 0.0
        return sum(
            (time_s - mean_t) * (value - mean_r)
            for time_s, value in zip(times, rssi)
        ) / denominator

    def _desired_state(self, sample: LinkSample, risk: float) -> str:
        if (
            sample.age_ms >= self.config.stale_age_ms
            or sample.rssi_dbm <= -93.0
            or sample.latency_ms >= 600.0
            or sample.packet_loss >= 0.80
        ):
            return "LINK_LOST"
        if risk >= self.config.return_risk or sample.packet_loss >= 0.55:
            return "RETURN"
        if risk >= self.config.hold_risk:
            return "HOLD"
        if risk >= self.config.degraded_risk:
            return "DEGRADED"
        return "HEALTHY"

    def evaluate(self, sample: LinkSample) -> LinkAssessment:
        sample.validate()
        if self.history and sample.timestamp_s <= self.history[-1].timestamp_s:
            raise ValueError("link samples must have increasing timestamps")
        self.history.append(sample)
        alpha = 0.24
        if self.rssi_ewma is None:
            self.rssi_ewma = sample.rssi_dbm
            self.latency_ewma = sample.latency_ms
            self.loss_ewma = sample.packet_loss
        else:
            self.rssi_ewma += alpha * (sample.rssi_dbm - self.rssi_ewma)
            self.latency_ewma += alpha * (sample.latency_ms - self.latency_ewma)
            self.loss_ewma += alpha * (sample.packet_loss - self.loss_ewma)

        trend = self._trend()
        trend_penalty = _clip((-trend - 0.5) / 5.0)
        risk = (
            0.27 * _clip((-sample.rssi_dbm - 50.0) / 40.0)
            + 0.25 * _clip((sample.latency_ms - 70.0) / 250.0)
            + 0.25 * _clip(sample.packet_loss / 0.45)
            + 0.13 * _clip((sample.age_ms - 250.0) / 1400.0)
            + 0.10 * trend_penalty
        )
        desired = self._desired_state(sample, risk)
        if desired == self.candidate_state:
            self.candidate_count += 1
        else:
            self.candidate_state = desired
            self.candidate_count = 1
        if (
            self.candidate_count >= self.config.switch_frames
            or desired == "LINK_LOST"
        ):
            self.state = desired

        if trend < -1.0:
            time_to_horizon = max(0.0, (sample.rssi_dbm + 85.0) / -trend)
        else:
            time_to_horizon = None
        recommendation = {
            "HEALTHY": "CONTINUE",
            "DEGRADED": "REDUCE_RATE",
            "HOLD": "HOLD_AND_RETRY",
            "RETURN": "RETURN_HOME",
            "LINK_LOST": "EXECUTE_FAILSAFE",
        }[self.state]
        return LinkAssessment(
            state=self.state,
            risk=risk,
            rssi_ewma=self.rssi_ewma,
            latency_ewma=self.latency_ewma,
            loss_ewma=self.loss_ewma,
            trend_db_per_s=trend,
            time_to_horizon_s=time_to_horizon,
            recommendation=recommendation,
            candidate_state=self.candidate_state,
        )
