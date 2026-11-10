"""Motor winding thermal guard based on a simple first-order RC model."""

from __future__ import annotations

from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class MotorSample:
    timestamp_s: float
    current_a: float
    ambient_temp_c: float
    rpm: float

    def validate(self) -> None:
        for v in asdict(self).values():
            if v != v or v in (float("inf"), float("-inf")):
                raise ValueError("motor sample must be finite")
        if self.current_a < 0.0:
            raise ValueError("current must be non-negative")


@dataclass(frozen=True)
class TempConfig:
    warm_c: float = 55.0
    hot_c: float = 75.0
    overtemp_c: float = 90.0
    # thermal model: C_th [J/K], R_th [K/W], R_winding [Ohm]
    heat_capacity: float = 20.0
    thermal_resistance: float = 2.0
    winding_resistance: float = 0.05
    dt: float = 0.1


@dataclass(frozen=True)
class TempAssessment:
    state: str
    estimated_temp_c: float
    current_a: float
    recommendation: str

    def to_dict(self) -> dict:
        return asdict(self)


class MotorTempGuard:
    def __init__(self, config: TempConfig | None = None) -> None:
        self.cfg = config or TempConfig()
        self.temp_c: float | None = None
        self.last_t: float | None = None

    def evaluate(self, sample: MotorSample) -> TempAssessment:
        sample.validate()
        if self.temp_c is None:
            self.temp_c = sample.ambient_temp_c
            self.last_t = sample.timestamp_s
            state = "COOL"
            rec = "CONTINUE"
        else:
            dt = max(0.01, sample.timestamp_s - self.last_t)
            # heat generation = I^2 * R, heat loss = (T - Tamb)/R_th
            p_heat = sample.current_a ** 2 * self.cfg.winding_resistance
            p_loss = (self.temp_c - sample.ambient_temp_c) / self.cfg.thermal_resistance
            dT = (p_heat - p_loss) / self.cfg.heat_capacity * dt
            self.temp_c += dT
            self.last_t = sample.timestamp_s

            t = self.temp_c
            if t >= self.cfg.overtemp_c:
                state, rec = "OVERTEMP", "AUTO_LAND"
            elif t >= self.cfg.hot_c:
                state, rec = "HOT", "REDUCE_THROTTLE"
            elif t >= self.cfg.warm_c:
                state, rec = "WARM", "MONITOR"
            else:
                state, rec = "COOL", "CONTINUE"

        return TempAssessment(state=state, estimated_temp_c=self.temp_c,
                              current_a=sample.current_a, recommendation=rec)
