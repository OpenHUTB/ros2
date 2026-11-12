"""ROS-independent gradeability and traction-margin calculation logic."""

from __future__ import annotations

from dataclasses import dataclass
import math


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


@dataclass(frozen=True)
class GradeabilityConfig:
    mass_kg: float = 1500.0
    gravity: float = 9.81
    crr: float = 0.012
    cda: float = 0.72
    air_density: float = 1.225
    max_drive_force_n: float = 4500.0
    friction_coef: float = 0.6
    slope_timeout_s: float = 1.0
    speed_timeout_s: float = 1.0
    control_rate_hz: float = 20.0

    def __post_init__(self) -> None:
        for name in (
            "mass_kg",
            "gravity",
            "crr",
            "cda",
            "air_density",
            "max_drive_force_n",
            "friction_coef",
            "slope_timeout_s",
            "speed_timeout_s",
            "control_rate_hz",
        ):
            _finite(getattr(self, name), name)
        if self.mass_kg <= 0.0:
            raise ValueError("mass_kg must be greater than zero")
        if self.gravity <= 0.0:
            raise ValueError("gravity must be greater than zero")
        if self.crr < 0.0:
            raise ValueError("crr must be non-negative")
        if self.cda < 0.0:
            raise ValueError("cda must be non-negative")
        if self.air_density <= 0.0:
            raise ValueError("air_density must be greater than zero")
        if self.max_drive_force_n < 0.0:
            raise ValueError("max_drive_force_n must be non-negative")
        if self.friction_coef <= 0.0:
            raise ValueError("friction_coef must be greater than zero")
        if self.slope_timeout_s <= 0.0:
            raise ValueError("slope_timeout_s must be greater than zero")
        if self.speed_timeout_s <= 0.0:
            raise ValueError("speed_timeout_s must be greater than zero")
        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be greater than zero")

    @property
    def weight_n(self) -> float:
        return self.mass_kg * self.gravity


def grade_force_n(mass_kg: float, gravity: float, slope_deg: float) -> float:
    """Longitudinal force caused by slope gravity component."""
    return mass_kg * gravity * math.sin(math.radians(_finite(slope_deg, "slope")))


def rolling_resistance_n(
    mass_kg: float, gravity: float, crr: float, slope_deg: float
) -> float:
    """Rolling resistance along the road surface."""
    return (
        mass_kg
        * gravity
        * crr
        * math.cos(math.radians(_finite(slope_deg, "slope")))
    )


def aero_drag_n(
    air_density: float, cda: float, speed_mps: float
) -> float:
    """Air drag at the given speed."""
    speed_mps = _finite(speed_mps, "speed")
    return 0.5 * air_density * cda * speed_mps * speed_mps


def required_force_n(config: GradeabilityConfig, slope_deg: float, speed_mps: float) -> float:
    """Total longitudinal traction force required to hold the slope at speed."""
    return (
        grade_force_n(config.mass_kg, config.gravity, slope_deg)
        + rolling_resistance_n(
            config.mass_kg, config.gravity, config.crr, slope_deg
        )
        + aero_drag_n(config.air_density, config.cda, speed_mps)
    )


def traction_limit_n(config: GradeabilityConfig, slope_deg: float) -> float:
    """Maximum force the tire-road contact can transfer without slipping."""
    return (
        config.friction_coef
        * config.weight_n
        * math.cos(math.radians(_finite(slope_deg, "slope")))
    )


@dataclass(frozen=True)
class GradeabilityResult:
    mode: str
    slope_deg: float
    speed_mps: float
    required_n: float
    drive_limit_n: float
    traction_limit_n: float
    available_n: float
    margin_n: float
    slip_risk: bool
    power_deficit: bool

    def to_dict(self) -> dict[str, str | float | bool]:
        return {
            "mode": self.mode,
            "slope_deg": round(self.slope_deg, 3),
            "speed_mps": round(self.speed_mps, 3),
            "required_n": round(self.required_n, 3),
            "drive_limit_n": round(self.drive_limit_n, 3),
            "traction_limit_n": round(self.traction_limit_n, 3),
            "available_n": round(self.available_n, 3),
            "margin_n": round(self.margin_n, 3),
            "slip_risk": self.slip_risk,
            "power_deficit": self.power_deficit,
        }


class GradeabilityChecker:
    """Compare required traction with drive and friction limits per control cycle."""

    def __init__(self, config: GradeabilityConfig | None = None) -> None:
        self.config = config or GradeabilityConfig()
        self.slope_deg: float | None = None
        self.slope_received_at: float | None = None
        self.speed_mps: float | None = None
        self.speed_received_at: float | None = None

    def update_slope(self, slope_deg: float, now_s: float) -> None:
        self.slope_deg = _finite(slope_deg, "slope")
        self.slope_received_at = _finite(now_s, "now")

    def update_speed(self, speed_mps: float, now_s: float) -> None:
        self.speed_mps = _finite(speed_mps, "speed")
        self.speed_received_at = _finite(now_s, "now")

    def step(self, now_s: float) -> GradeabilityResult | None:
        if self.slope_deg is None or self.speed_mps is None:
            return None
        now_s = _finite(now_s, "now")
        slope_fresh = (
            self.slope_received_at is not None
            and now_s - self.slope_received_at <= self.config.slope_timeout_s
        )
        speed_fresh = (
            self.speed_received_at is not None
            and now_s - self.speed_received_at <= self.config.speed_timeout_s
        )
        if not slope_fresh:
            return self._stale_result("SLOPE_STALE")
        if not speed_fresh:
            return self._stale_result("SPEED_STALE")

        slope = self.slope_deg
        speed = self.speed_mps
        required = required_force_n(self.config, slope, speed)
        drive_limit = self.config.max_drive_force_n
        traction_limit = traction_limit_n(self.config, slope)
        available = min(drive_limit, traction_limit)
        margin = available - required
        slip_risk = required > traction_limit
        power_deficit = required > drive_limit

        # 失败情形只有侧滑（TRACTION_LIMIT）与动力不足（POWER_LIMIT）两种；
        # 两者都不满足时 margin 必然 >= 0，归为 CLIMBABLE，不再单列冗余分支。
        if slip_risk:
            mode = "TRACTION_LIMIT"
        elif power_deficit:
            mode = "POWER_LIMIT"
        else:
            mode = "CLIMBABLE"

        return GradeabilityResult(
            mode=mode,
            slope_deg=slope,
            speed_mps=speed,
            required_n=required,
            drive_limit_n=drive_limit,
            traction_limit_n=traction_limit,
            available_n=available,
            margin_n=margin,
            slip_risk=slip_risk,
            power_deficit=power_deficit,
        )

    def _stale_result(self, mode: str) -> GradeabilityResult:
        slope = self.slope_deg if self.slope_deg is not None else 0.0
        speed = self.speed_mps if self.speed_mps is not None else 0.0
        return GradeabilityResult(
            mode=mode,
            slope_deg=slope,
            speed_mps=speed,
            required_n=0.0,
            drive_limit_n=self.config.max_drive_force_n,
            traction_limit_n=traction_limit_n(self.config, slope),
            available_n=0.0,
            margin_n=0.0,
            slip_risk=False,
            power_deficit=False,
        )

    def max_climbable_slope_deg(self, speed_mps: float, step_deg: float = 0.1) -> float:
        """Largest steady slope that satisfies both drive and friction limits."""
        best = 0.0
        slope = 0.0
        while slope <= 45.0:
            required = required_force_n(self.config, slope, speed_mps)
            traction = traction_limit_n(self.config, slope)
            if required <= self.config.max_drive_force_n and required <= traction:
                best = slope
            slope += step_deg
        return round(best, 2)
