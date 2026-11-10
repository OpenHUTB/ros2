"""Explainable multi-cell battery health risk assessment."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import statistics


@dataclass(frozen=True)
class BatterySample:
    timestamp_s: float
    cell_voltages_v: tuple[float, ...]
    current_a: float
    temperature_c: float
    internal_resistance_mohm: float

    def validate(self) -> None:
        if not math.isfinite(self.timestamp_s):
            raise ValueError("timestamp must be finite")
        if len(self.cell_voltages_v) < 2:
            raise ValueError("at least two cell voltages are required")
        if not all(math.isfinite(value) and value > 0.0 for value in self.cell_voltages_v):
            raise ValueError("cell voltages must be finite and positive")
        if not all(
            math.isfinite(value)
            for value in (
                self.current_a,
                self.temperature_c,
                self.internal_resistance_mohm,
            )
        ):
            raise ValueError("battery sample must contain finite values")
        if self.current_a < 0.0 or self.internal_resistance_mohm < 0.0:
            raise ValueError("current and internal resistance must be non-negative")


@dataclass(frozen=True)
class BatteryHealthConfig:
    degraded_imbalance_v: float = 0.06
    limit_imbalance_v: float = 0.14
    critical_imbalance_v: float = 0.24
    switch_frames: int = 3

    def __post_init__(self) -> None:
        if not (
            0.0
            <= self.degraded_imbalance_v
            < self.limit_imbalance_v
            < self.critical_imbalance_v
        ):
            raise ValueError("imbalance thresholds must be strictly increasing")
        if self.switch_frames < 1:
            raise ValueError("switch_frames must be positive")


@dataclass(frozen=True)
class BatteryAssessment:
    state: str
    risk: float
    imbalance_v: float
    min_cell_v: float
    mean_cell_v: float
    temperature_c: float
    internal_resistance_mohm: float
    recommendation: str

    def to_dict(self) -> dict:
        payload = asdict(self)
        payload["cell_voltages_v"] = None
        return payload


def _clip(value: float) -> float:
    return max(0.0, min(1.0, value))


class BatteryHealthGuard:
    def __init__(self, config: BatteryHealthConfig | None = None) -> None:
        self.config = config or BatteryHealthConfig()
        self.last_timestamp: float | None = None
        self.state = "HEALTHY"
        self.candidate = "HEALTHY"
        self.candidate_frames = 0

    def evaluate(self, sample: BatterySample) -> BatteryAssessment:
        sample.validate()
        if self.last_timestamp is not None and sample.timestamp_s <= self.last_timestamp:
            raise ValueError("battery timestamps must increase")
        imbalance = max(sample.cell_voltages_v) - min(sample.cell_voltages_v)
        minimum = min(sample.cell_voltages_v)
        mean = statistics.fmean(sample.cell_voltages_v)
        weak_cell = max(mean - minimum, 0.0)
        imbalance_risk = _clip(imbalance / 0.35)
        weak_cell_risk = _clip(weak_cell / 0.25)
        temperature_risk = _clip((sample.temperature_c - 38.0) / 27.0)
        resistance_risk = _clip((sample.internal_resistance_mohm - 18.0) / 42.0)
        low_voltage_risk = _clip((3.35 - minimum) / 0.30)
        load_risk = _clip((sample.current_a - 35.0) / 55.0) * _clip(
            sample.internal_resistance_mohm / 35.0
        )
        risk = (
            0.29 * imbalance_risk
            + 0.19 * weak_cell_risk
            + 0.20 * temperature_risk
            + 0.16 * resistance_risk
            + 0.12 * low_voltage_risk
            + 0.04 * load_risk
        )
        if (
            imbalance >= self.config.critical_imbalance_v
            or minimum < 3.10
            or sample.temperature_c >= 66.0
        ):
            desired = "LAND_NOW"
        elif risk >= 0.72 or imbalance >= self.config.limit_imbalance_v:
            desired = "LIMIT_LOAD"
        elif risk >= 0.42 or imbalance >= self.config.degraded_imbalance_v:
            desired = "DEGRADED"
        else:
            desired = "HEALTHY"
        if desired == self.candidate:
            self.candidate_frames += 1
        else:
            self.candidate = desired
            self.candidate_frames = 1
        if self.candidate_frames >= self.config.switch_frames or desired == "LAND_NOW":
            self.state = desired
        recommendation = {
            "HEALTHY": "CONTINUE",
            "DEGRADED": "MONITOR_AND_REDUCE_PEAK_LOAD",
            "LIMIT_LOAD": "RETURN_WITH_LOW_POWER",
            "LAND_NOW": "LAND_AT_NEAREST_SAFE_SITE",
        }[self.state]
        self.last_timestamp = sample.timestamp_s
        return BatteryAssessment(
            state=self.state,
            risk=risk,
            imbalance_v=imbalance,
            min_cell_v=minimum,
            mean_cell_v=mean,
            temperature_c=sample.temperature_c,
            internal_resistance_mohm=sample.internal_resistance_mohm,
            recommendation=recommendation,
        )
