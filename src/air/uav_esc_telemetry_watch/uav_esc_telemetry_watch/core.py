"""ESC telemetry watchdog: frame freshness, voltage sag, current surge, temp."""

from __future__ import annotations

from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class ESCSample:
    timestamp_s: float
    voltage_v: float
    current_a: float
    rpm: float
    temp_c: float
    frame_age_ms: float

    def validate(self) -> None:
        for v in asdict(self).values():
            if v != v or v in (float("inf"), float("-inf")):
                raise ValueError("esc sample must be finite")
        if self.voltage_v < 0.0 or self.current_a < 0.0:
            raise ValueError("voltage/current must be non-negative")


@dataclass(frozen=True)
class ESCWatchConfig:
    nominal_voltage_v: float = 22.2  # 6S
    sag_voltage_v: float = 19.0
    low_voltage_v: float = 18.0
    high_current_a: float = 45.0
    esc_overtemp_c: float = 80.0
    stale_age_ms: float = 200.0


@dataclass(frozen=True)
class ESCAssessment:
    state: str
    risk: float
    voltage_v: float
    current_a: float
    temp_c: float
    frame_age_ms: float
    recommendation: str

    def to_dict(self) -> dict:
        return asdict(self)


def _clip01(v: float) -> float:
    return max(0.0, min(1.0, v))


class ESCTelemetryWatch:
    def __init__(self, config: ESCWatchConfig | None = None) -> None:
        self.cfg = config or ESCWatchConfig()
        self.state = "OK"

    def evaluate(self, sample: ESCSample) -> ESCAssessment:
        sample.validate()
        cfg = self.cfg

        v_risk = _clip01((cfg.sag_voltage_v - sample.voltage_v) / 4.0)
        i_risk = _clip01((sample.current_a - cfg.high_current_a + 10.0) / 20.0)
        t_risk = _clip01((sample.temp_c - cfg.esc_overtemp_c + 20.0) / 25.0)
        age_risk = _clip01((sample.frame_age_ms - 80.0) / 150.0)

        risk = 0.30 * v_risk + 0.25 * i_risk + 0.20 * t_risk + 0.25 * age_risk

        hard_bad = (
            sample.voltage_v <= cfg.low_voltage_v
            or sample.frame_age_ms >= cfg.stale_age_ms
            or sample.temp_c >= cfg.esc_overtemp_c
        )
        if hard_bad or risk >= 0.8:
            self.state = "ESC_LOST"
        elif risk >= 0.55:
            self.state = "DEGRADED"
        elif risk >= 0.3:
            self.state = "WATCH"
        else:
            self.state = "OK"

        rec = {
            "OK": "CONTINUE",
            "WATCH": "MONITOR",
            "DEGRADED": "REDUCE_THROTTLE",
            "ESC_LOST": "MOTOR_FAILSAFE",
        }[self.state]

        return ESCAssessment(
            state=self.state, risk=risk,
            voltage_v=sample.voltage_v, current_a=sample.current_a,
            temp_c=sample.temp_c, frame_age_ms=sample.frame_age_ms,
            recommendation=rec,
        )
