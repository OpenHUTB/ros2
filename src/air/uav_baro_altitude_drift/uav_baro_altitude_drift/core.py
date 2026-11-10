"""Barometric altitude drift watch against GPS and radar altitude references."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class AltSample:
    timestamp_s: float
    baro_alt_m: float
    gps_alt_m: float
    radar_alt_m: float
    vertical_speed_mps: float
    pressure_hpa: float
    baro_temp_c: float

    def validate(self) -> None:
        for v in asdict(self).values():
            if not math.isfinite(v):
                raise ValueError("alt sample must be finite")
        if self.pressure_hpa <= 0.0:
            raise ValueError("pressure must be positive")


@dataclass(frozen=True)
class BaroWatchConfig:
    watch_risk: float = 0.35
    drift_risk: float = 0.60
    lost_risk: float = 0.82
    switch_frames: int = 3
    radar_valid_alt_m: float = 10.0
    window: int = 20

    def __post_init__(self) -> None:
        if not 0.0 <= self.watch_risk < self.drift_risk < self.lost_risk <= 1.0:
            raise ValueError("thresholds must be increasing")


@dataclass(frozen=True)
class AltAssessment:
    state: str
    risk: float
    baro_gps_diff_m: float
    baro_radar_diff_m: float
    pressure_rate_hpa_s: float
    recommendation: str

    def to_dict(self) -> dict:
        return asdict(self)


def _clip01(v: float) -> float:
    return max(0.0, min(1.0, v))


class BaroAltitudeDriftWatch:
    def __init__(self, config: BaroWatchConfig | None = None) -> None:
        self.cfg = config or BaroWatchConfig()
        self.history: deque[AltSample] = deque(maxlen=self.cfg.window)
        self.state = "CALM"
        self.candidate = "CALM"
        self.candidate_count = 0

    def _pressure_rate(self) -> float:
        if len(self.history) < 3:
            return 0.0
        t0, t1 = self.history[0].timestamp_s, self.history[-1].timestamp_s
        if t1 - t0 < 1e-3:
            return 0.0
        return (self.history[-1].pressure_hpa - self.history[0].pressure_hpa) / (t1 - t0)

    def evaluate(self, sample: AltSample) -> AltAssessment:
        sample.validate()
        if self.history and sample.timestamp_s <= self.history[-1].timestamp_s:
            raise ValueError("timestamps must increase")
        self.history.append(sample)

        bg = abs(sample.baro_alt_m - sample.gps_alt_m)
        br = (
            abs(sample.baro_alt_m - sample.radar_alt_m)
            if sample.radar_alt_m < self.cfg.radar_valid_alt_m
            else 0.0
        )
        rate = self._pressure_rate()
        rate_abs = abs(rate)

        risk = (
            0.40 * _clip01((bg - 2.0) / 15.0)
            + 0.30 * _clip01((br - 1.0) / 5.0)
            + 0.20 * _clip01((rate_abs - 0.5) / 3.0)
            + 0.10 * _clip01(abs(sample.vertical_speed_mps) / 8.0)
        )

        hard_bad = bg >= 25.0 or br >= 8.0 or rate_abs >= 8.0
        if hard_bad or risk >= self.cfg.lost_risk:
            desired = "BARO_LOST"
        elif risk >= self.cfg.drift_risk or bg >= 18.0 or br >= 6.0:
            desired = "DRIFT"
        elif risk >= self.cfg.watch_risk:
            desired = "WATCH"
        else:
            desired = "CALM"

        if desired == self.candidate:
            self.candidate_count += 1
        else:
            self.candidate = desired
            self.candidate_count = 1
        if self.candidate_count >= self.cfg.switch_frames or desired == "BARO_LOST":
            self.state = desired

        rec = {
            "CALM": "CONTINUE",
            "WATCH": "WATCH_BARO",
            "DRIFT": "FUSE_GPS_RADAR",
            "BARO_LOST": "DISREGARD_BARO",
        }[self.state]

        return AltAssessment(
            state=self.state,
            risk=risk,
            baro_gps_diff_m=bg,
            baro_radar_diff_m=br,
            pressure_rate_hpa_s=rate,
            recommendation=rec,
        )
