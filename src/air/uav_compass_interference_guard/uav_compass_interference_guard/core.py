"""Explainable magnetometer interference scoring and hysteretic compass states."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import math


# Typical geomagnetic field magnitude over mainland China: ~45-55 uT.
NOMINAL_FIELD_UT = 50.0
FIELD_TOLERANCE_UT = 18.0  # |B - B0| beyond this is suspicious


def _angle_diff_deg(a_deg: float, b_deg: float) -> float:
    """Smallest signed difference (a - b) wrapped to [-180, 180]."""
    diff = (a_deg - b_deg + 180.0) % 360.0 - 180.0
    return diff


def _clip01(value: float) -> float:
    return max(0.0, min(1.0, value))


@dataclass(frozen=True)
class CompassSample:
    timestamp_s: float
    mag_heading_deg: float
    gps_course_deg: float
    vio_heading_deg: float
    mag_field_ut: float
    motor_current_a: float
    ground_speed_mps: float
    altitude_m: float

    def validate(self) -> None:
        values = asdict(self)
        if not all(math.isfinite(v) for v in values.values()):
            raise ValueError("compass sample must contain finite values")
        if self.mag_field_ut <= 0.0:
            raise ValueError("mag_field_ut must be positive")
        if self.motor_current_a < 0.0:
            raise ValueError("motor_current_a must be non-negative")
        if self.ground_speed_mps < 0.0 or self.altitude_m < 0.0:
            raise ValueError("speed and altitude must be non-negative")


@dataclass(frozen=True)
class CompassGuardConfig:
    suspect_risk: float = 0.35
    biased_risk: float = 0.60
    lost_risk: float = 0.82
    switch_frames: int = 3
    min_gps_speed_mps: float = 3.0
    trend_window: int = 20
    # Hard limits that immediately force COMPASS_LOST.
    hard_field_low_ut: float = 10.0
    hard_field_high_ut: float = 120.0
    hard_heading_diff_deg: float = 60.0

    def __post_init__(self) -> None:
        if not (
            0.0 <= self.suspect_risk < self.biased_risk < self.lost_risk <= 1.0
        ):
            raise ValueError("risk thresholds must be strictly increasing in [0, 1]")
        if self.switch_frames < 1 or self.trend_window < 3:
            raise ValueError("switch_frames and trend_window are too small")


@dataclass(frozen=True)
class CompassAssessment:
    state: str
    risk: float
    field_deviation_ut: float
    gps_heading_diff_deg: float
    vio_heading_diff_deg: float
    current_ewma_a: float
    low_altitude_flag: bool
    recommendation: str
    candidate_state: str

    def to_dict(self) -> dict:
        return asdict(self)


class CompassInterferenceGuard:
    """Fuses magnetometer magnitude, heading cross-checks and current surge.

    The guard treats the magnetometer as a relative sensor: it does not know the
    true heading, but it can detect when the magnetic field magnitude drifts from
    the local geomagnetic field, when the magnetic heading disagrees with GPS
    course (only at speed, where GPS course is trustworthy) and with VIO heading,
    and when motor current surges that usually come with EMI.
    """

    def __init__(self, config: CompassGuardConfig | None = None) -> None:
        self.config = config or CompassGuardConfig()
        self.history: deque[CompassSample] = deque(maxlen=self.config.trend_window)
        self.state = "CALM"
        self.candidate_state = "CALM"
        self.candidate_count = 0
        self.current_ewma: float | None = None

    def _field_risk(self, sample: CompassSample) -> float:
        deviation = abs(sample.mag_field_ut - NOMINAL_FIELD_UT)
        return _clip01((deviation - 6.0) / FIELD_TOLERANCE_UT)

    def _gps_heading_risk(self, sample: CompassSample) -> tuple[float, float]:
        """Returns (risk, absolute heading diff). GPS course only counts at speed."""
        if sample.ground_speed_mps < self.config.min_gps_speed_mps:
            return 0.0, 0.0
        diff = abs(_angle_diff_deg(sample.mag_heading_deg, sample.gps_course_deg))
        risk = _clip01((diff - 8.0) / 45.0)
        return risk, diff

    def _vio_heading_risk(self, sample: CompassSample) -> tuple[float, float]:
        diff = abs(_angle_diff_deg(sample.mag_heading_deg, sample.vio_heading_deg))
        risk = _clip01((diff - 10.0) / 50.0)
        return risk, diff

    def _current_risk(self, sample: CompassSample) -> float:
        if self.current_ewma is None:
            self.current_ewma = sample.motor_current_a
            return 0.0
        alpha = 0.20
        self.current_ewma += alpha * (sample.motor_current_a - self.current_ewma)
        surge = sample.motor_current_a - self.current_ewma
        # A >4 A surge above the running mean is the suspicious band.
        return _clip01((surge - 1.5) / 4.0)

    def _altitude_risk(self, sample: CompassSample) -> tuple[float, bool]:
        if sample.altitude_m >= 15.0:
            return 0.0, False
        risk = _clip01((15.0 - sample.altitude_m) / 15.0)
        return risk, True

    def _desired_state(
        self, sample: CompassSample, risk: float, gps_diff: float, vio_diff: float
    ) -> str:
        cfg = self.config
        hard_bad = (
            sample.mag_field_ut <= cfg.hard_field_low_ut
            or sample.mag_field_ut >= cfg.hard_field_high_ut
            or gps_diff >= cfg.hard_heading_diff_deg
            or vio_diff >= cfg.hard_heading_diff_deg
        )
        if hard_bad or risk >= cfg.lost_risk:
            return "COMPASS_LOST"
        if risk >= cfg.biased_risk or gps_diff >= 25.0 or vio_diff >= 25.0:
            return "BIASED"
        if risk >= cfg.suspect_risk:
            return "SUSPECTED"
        return "CALM"

    def evaluate(self, sample: CompassSample) -> CompassAssessment:
        sample.validate()
        if self.history and sample.timestamp_s <= self.history[-1].timestamp_s:
            raise ValueError("compass samples must have increasing timestamps")
        self.history.append(sample)

        field_risk = self._field_risk(sample)
        gps_risk, gps_diff = self._gps_heading_risk(sample)
        vio_risk, vio_diff = self._vio_heading_risk(sample)
        current_risk = self._current_risk(sample)
        altitude_risk, low_altitude = self._altitude_risk(sample)

        risk = (
            0.30 * field_risk
            + 0.30 * gps_risk
            + 0.20 * vio_risk
            + 0.10 * current_risk
            + 0.10 * altitude_risk
        )

        desired = self._desired_state(sample, risk, gps_diff, vio_diff)
        if desired == self.candidate_state:
            self.candidate_count += 1
        else:
            self.candidate_state = desired
            self.candidate_count = 1
        if (
            self.candidate_count >= self.config.switch_frames
            or desired == "COMPASS_LOST"
        ):
            self.state = desired

        recommendation = {
            "CALM": "CONTINUE",
            "SUSPECTED": "MONITOR_COMPASS",
            "BIASED": "FUSE_GPS_VIO_HEADING",
            "COMPASS_LOST": "DISREGARD_COMPASS",
        }[self.state]

        return CompassAssessment(
            state=self.state,
            risk=risk,
            field_deviation_ut=abs(sample.mag_field_ut - NOMINAL_FIELD_UT),
            gps_heading_diff_deg=gps_diff,
            vio_heading_diff_deg=vio_diff,
            current_ewma_a=self.current_ewma or sample.motor_current_a,
            low_altitude_flag=low_altitude,
            recommendation=recommendation,
            candidate_state=self.candidate_state,
        )
