"""Constant-velocity closest-approach prediction for moving circular zones."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class UAVMotion:
    x: float
    y: float
    vx: float
    vy: float

    def validate(self) -> None:
        if not all(math.isfinite(value) for value in asdict(self).values()):
            raise ValueError("UAV motion must contain finite values")


@dataclass(frozen=True)
class DynamicZone:
    zone_id: str
    center_x: float
    center_y: float
    radius_m: float
    vx: float = 0.0
    vy: float = 0.0
    active: bool = True

    def validate(self) -> None:
        if not self.zone_id:
            raise ValueError("zone_id must not be empty")
        values = (
            self.center_x,
            self.center_y,
            self.radius_m,
            self.vx,
            self.vy,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{self.zone_id}: zone values must be finite")
        if self.radius_m <= 0.0:
            raise ValueError(f"{self.zone_id}: radius_m must be positive")


@dataclass(frozen=True)
class GeofenceConfig:
    horizon_s: float = 10.0
    warning_margin_m: float = 5.0
    critical_margin_m: float = 1.5

    def __post_init__(self) -> None:
        if self.horizon_s <= 0.0:
            raise ValueError("prediction horizon must be positive")
        if not 0.0 <= self.critical_margin_m < self.warning_margin_m:
            raise ValueError("margins must satisfy 0 <= critical < warning")


@dataclass(frozen=True)
class ZoneAssessment:
    zone_id: str
    state: str
    predicted_clearance_m: float
    current_signed_distance_m: float
    time_to_closest_s: float
    avoid_x: float
    avoid_y: float

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class GeofenceAssessment:
    state: str
    limiting_zone_id: str | None
    predicted_clearance_m: float | None
    time_to_closest_s: float | None
    avoid_x: float
    avoid_y: float
    recommendation: str
    zones: tuple[ZoneAssessment, ...]

    def to_dict(self) -> dict:
        payload = asdict(self)
        payload["zones"] = [asdict(zone) for zone in self.zones]
        return payload


def _normalize(x: float, y: float) -> tuple[float, float]:
    norm = math.hypot(x, y)
    if norm < 1e-9:
        return 0.0, 0.0
    return x / norm, y / norm


class GeofenceGuard:
    def __init__(self, config: GeofenceConfig | None = None) -> None:
        self.config = config or GeofenceConfig()

    def assess_zone(self, uav: UAVMotion, zone: DynamicZone) -> ZoneAssessment:
        uav.validate()
        zone.validate()
        c = self.config
        rx = zone.center_x - uav.x
        ry = zone.center_y - uav.y
        rvx = zone.vx - uav.vx
        rvy = zone.vy - uav.vy
        relative_speed_sq = rvx * rvx + rvy * rvy
        if relative_speed_sq > 1e-12:
            t_star = -(rx * rvx + ry * rvy) / relative_speed_sq
        else:
            t_star = 0.0
        t_star = max(0.0, min(c.horizon_s, t_star))

        current_distance = math.hypot(rx, ry) - zone.radius_m
        closest_uav_x = uav.x + uav.vx * t_star
        closest_uav_y = uav.y + uav.vy * t_star
        closest_center_x = zone.center_x + zone.vx * t_star
        closest_center_y = zone.center_y + zone.vy * t_star
        dx = closest_uav_x - closest_center_x
        dy = closest_uav_y - closest_center_y
        clearance = math.hypot(dx, dy) - zone.radius_m
        avoid_x, avoid_y = _normalize(dx, dy)
        if avoid_x == 0.0 and avoid_y == 0.0:
            avoid_x, avoid_y = _normalize(-rvy, rvx)

        if current_distance < 0.0:
            state = "BREACH"
        elif clearance <= 0.0:
            state = "CRITICAL"
        elif clearance <= c.critical_margin_m:
            state = "CRITICAL"
        elif clearance <= c.warning_margin_m:
            state = "WARNING"
        else:
            state = "CLEAR"
        return ZoneAssessment(
            zone_id=zone.zone_id,
            state=state,
            predicted_clearance_m=clearance,
            current_signed_distance_m=current_distance,
            time_to_closest_s=t_star,
            avoid_x=avoid_x,
            avoid_y=avoid_y,
        )

    def assess(
        self, uav: UAVMotion, zones: list[DynamicZone]
    ) -> GeofenceAssessment:
        active = [zone for zone in zones if zone.active]
        if not active:
            return GeofenceAssessment(
                "CLEAR", None, None, None, 0.0, 0.0, "CONTINUE", ()
            )
        assessments = tuple(self.assess_zone(uav, zone) for zone in active)
        priority = {"BREACH": 3, "CRITICAL": 2, "WARNING": 1, "CLEAR": 0}
        limiting = max(
            assessments,
            key=lambda item: (priority[item.state], -item.predicted_clearance_m),
        )
        recommendation = {
            "BREACH": "EXIT_ZONE_IMMEDIATELY",
            "CRITICAL": "DIVERT_NOW",
            "WARNING": "PREPARE_DIVERSION",
            "CLEAR": "CONTINUE",
        }[limiting.state]
        return GeofenceAssessment(
            state=limiting.state,
            limiting_zone_id=limiting.zone_id,
            predicted_clearance_m=limiting.predicted_clearance_m,
            time_to_closest_s=limiting.time_to_closest_s,
            avoid_x=limiting.avoid_x,
            avoid_y=limiting.avoid_y,
            recommendation=recommendation,
            zones=assessments,
        )
