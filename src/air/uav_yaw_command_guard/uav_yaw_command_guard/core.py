"""ROS-independent yaw shortest-path and rate-limiting logic."""

from __future__ import annotations

from dataclasses import dataclass
import math


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to the half-open interval [-pi, pi)."""
    angle_rad = _finite(angle_rad, "angle")
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def shortest_angle(target_rad: float, current_rad: float) -> float:
    """Return the signed shortest rotation from current to target."""
    return wrap_angle(_finite(target_rad, "target") - _finite(current_rad, "current"))


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class YawGuardConfig:
    control_rate_hz: float = 20.0
    max_yaw_rate_deg_s: float = 90.0
    target_timeout_s: float = 1.0
    current_timeout_s: float = 1.0

    def __post_init__(self) -> None:
        for name in (
            "control_rate_hz",
            "max_yaw_rate_deg_s",
            "target_timeout_s",
            "current_timeout_s",
        ):
            _finite(getattr(self, name), name)
        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be greater than zero")
        if self.max_yaw_rate_deg_s <= 0.0:
            raise ValueError("max_yaw_rate_deg_s must be greater than zero")
        if self.target_timeout_s <= 0.0:
            raise ValueError("target_timeout_s must be greater than zero")
        if self.current_timeout_s <= 0.0:
            raise ValueError("current_timeout_s must be greater than zero")

    @property
    def max_yaw_rate_rad_s(self) -> float:
        return math.radians(self.max_yaw_rate_deg_s)

    @property
    def max_step_rad(self) -> float:
        return self.max_yaw_rate_rad_s / self.control_rate_hz


@dataclass(frozen=True)
class YawGuardResult:
    mode: str
    current_yaw_rad: float
    target_yaw_rad: float
    error_rad: float
    yaw_rate_rad_s: float
    limited: bool

    def to_dict(self) -> dict[str, str | float | bool]:
        return {
            "mode": self.mode,
            "current_yaw_deg": round(math.degrees(self.current_yaw_rad), 3),
            "target_yaw_deg": round(math.degrees(self.target_yaw_rad), 3),
            "error_deg": round(math.degrees(self.error_rad), 3),
            "rate_deg_s": round(math.degrees(self.yaw_rate_rad_s), 3),
            "limited": self.limited,
        }


class YawGuard:
    """Track the shortest yaw error and publish a bounded angular-rate result."""

    def __init__(self, config: YawGuardConfig | None = None) -> None:
        self.config = config or YawGuardConfig()
        self.current_yaw: float | None = None
        self.current_received_at: float | None = None
        self.target_yaw: float | None = None
        self.target_received_at: float | None = None

    def update_current(self, current_yaw_rad: float, now_s: float) -> None:
        self.current_yaw = wrap_angle(current_yaw_rad)
        self.current_received_at = _finite(now_s, "now")

    def update_target(self, target_yaw_rad: float, now_s: float) -> None:
        self.target_yaw = wrap_angle(target_yaw_rad)
        self.target_received_at = _finite(now_s, "now")

    def step(self, now_s: float) -> YawGuardResult | None:
        if self.current_yaw is None:
            return None
        now_s = _finite(now_s, "now")
        current_fresh = (
            self.current_received_at is not None
            and now_s - self.current_received_at <= self.config.current_timeout_s
        )
        target_fresh = (
            self.target_yaw is not None
            and self.target_received_at is not None
            and now_s - self.target_received_at <= self.config.target_timeout_s
        )
        if not current_fresh:
            target = (
                self.current_yaw if self.target_yaw is None else self.target_yaw
            )
            error = 0.0
            rate = 0.0
            mode = "CURRENT_STALE"
            limited = False
        elif not target_fresh:
            target = self.current_yaw
            error = 0.0
            rate = 0.0
            mode = "HOLD"
            limited = False
        else:
            target = self.target_yaw
            error = shortest_angle(target, self.current_yaw)
            rate = _clamp(
                error * self.config.control_rate_hz,
                self.config.max_yaw_rate_rad_s,
            )
            limited = abs(error) > self.config.max_step_rad
            if limited:
                mode = "LIMITED"
            elif abs(error) > 1e-9:
                mode = "TRACKING"
            else:
                mode = "ALIGNED"
        return YawGuardResult(
            mode=mode,
            current_yaw_rad=self.current_yaw,
            target_yaw_rad=float(target),
            error_rad=error,
            yaw_rate_rad_s=rate,
            limited=limited,
        )
