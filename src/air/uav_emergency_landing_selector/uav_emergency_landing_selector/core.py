"""Pure Python scoring and hard-constraint evaluation for landing sites."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class UAVState:
    x: float
    y: float
    altitude_m: float
    battery_percent: float
    wind_speed_mps: float = 0.0

    def validate(self) -> None:
        values = asdict(self)
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError("UAV state must contain finite values")
        if self.altitude_m <= 0.0:
            raise ValueError("altitude_m must be positive")
        if not 0.0 <= self.battery_percent <= 100.0:
            raise ValueError("battery_percent must be in [0, 100]")
        if self.wind_speed_mps < 0.0:
            raise ValueError("wind_speed_mps must be non-negative")


@dataclass(frozen=True)
class LandingSite:
    site_id: str
    x: float
    y: float
    slope_deg: float
    roughness_m: float
    obstacle_risk: float
    people_risk: float
    surface_confidence: float

    def validate(self) -> None:
        if not self.site_id:
            raise ValueError("site_id must not be empty")
        numeric = (
            self.x,
            self.y,
            self.slope_deg,
            self.roughness_m,
            self.obstacle_risk,
            self.people_risk,
            self.surface_confidence,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError(f"{self.site_id}: site values must be finite")
        if self.slope_deg < 0.0 or self.roughness_m < 0.0:
            raise ValueError(f"{self.site_id}: slope and roughness must be non-negative")
        for name, value in (
            ("obstacle_risk", self.obstacle_risk),
            ("people_risk", self.people_risk),
            ("surface_confidence", self.surface_confidence),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{self.site_id}: {name} must be in [0, 1]")


@dataclass(frozen=True)
class LandingConstraints:
    max_slope_deg: float = 15.0
    max_roughness_m: float = 0.45
    max_obstacle_risk: float = 0.70
    max_people_risk: float = 0.40
    max_distance_m: float = 80.0
    reserve_margin_m: float = 12.0

    def __post_init__(self) -> None:
        if min(
            self.max_slope_deg,
            self.max_roughness_m,
            self.max_obstacle_risk,
            self.max_people_risk,
            self.max_distance_m,
            self.reserve_margin_m,
        ) < 0.0:
            raise ValueError("landing constraints must be non-negative")


@dataclass(frozen=True)
class SiteEvaluation:
    site_id: str
    distance_m: float
    cost: float
    score: float
    eligible: bool
    rejection_reasons: tuple[str, ...]


@dataclass(frozen=True)
class LandingDecision:
    state: str
    selected_site_id: str | None
    selected_x: float | None
    selected_y: float | None
    score: float | None
    eligible_count: int
    evaluations: tuple[SiteEvaluation, ...]

    def to_dict(self) -> dict:
        return {
            "state": self.state,
            "selected_site_id": self.selected_site_id,
            "selected_x": self.selected_x,
            "selected_y": self.selected_y,
            "score": self.score,
            "eligible_count": self.eligible_count,
            "evaluations": [asdict(evaluation) for evaluation in self.evaluations],
        }


class LandingSiteSelector:
    """Filters unsafe sites and ranks the remainder with an explainable cost."""

    def __init__(self, constraints: LandingConstraints | None = None) -> None:
        self.constraints = constraints or LandingConstraints()

    def evaluate(self, uav: UAVState, site: LandingSite) -> SiteEvaluation:
        uav.validate()
        site.validate()
        c = self.constraints
        distance = math.hypot(site.x - uav.x, site.y - uav.y)
        reasons: list[str] = []
        if site.slope_deg > c.max_slope_deg:
            reasons.append("slope")
        if site.roughness_m > c.max_roughness_m:
            reasons.append("roughness")
        if site.obstacle_risk > c.max_obstacle_risk:
            reasons.append("obstacle")
        if site.people_risk > c.max_people_risk:
            reasons.append("people")
        if distance > c.max_distance_m:
            reasons.append("distance")
        usable_range = c.max_distance_m * max(0.0, uav.battery_percent - 10.0) / 90.0
        if distance + c.reserve_margin_m > usable_range:
            reasons.append("energy_margin")

        slope_term = min(site.slope_deg / max(c.max_slope_deg, 1e-6), 2.0)
        roughness_term = min(site.roughness_m / max(c.max_roughness_m, 1e-6), 2.0)
        distance_term = min(distance / max(c.max_distance_m, 1e-6), 2.0)
        wind_term = min((uav.wind_speed_mps / 15.0) * (1.0 + slope_term), 1.0)
        confidence_term = 1.0 - site.surface_confidence
        cost = (
            0.23 * slope_term
            + 0.14 * roughness_term
            + 0.20 * site.obstacle_risk
            + 0.27 * site.people_risk
            + 0.10 * distance_term
            + 0.03 * wind_term
            + 0.03 * confidence_term
        )
        return SiteEvaluation(
            site_id=site.site_id,
            distance_m=distance,
            cost=cost,
            score=max(0.0, 100.0 * (1.0 - cost)),
            eligible=not reasons,
            rejection_reasons=tuple(reasons),
        )

    def select(self, uav: UAVState, sites: list[LandingSite]) -> LandingDecision:
        if not sites:
            return LandingDecision("NO_CANDIDATE", None, None, None, None, 0, ())
        evaluations = tuple(self.evaluate(uav, site) for site in sites)
        eligible = [item for item in evaluations if item.eligible]
        if not eligible:
            return LandingDecision(
                "NO_SAFE_SITE", None, None, None, None, 0, evaluations
            )
        best = min(eligible, key=lambda item: (item.cost, item.site_id))
        source = next(site for site in sites if site.site_id == best.site_id)
        state = "SELECTED" if best.cost <= 0.35 else "CAUTION"
        return LandingDecision(
            state=state,
            selected_site_id=best.site_id,
            selected_x=source.x,
            selected_y=source.y,
            score=best.score,
            eligible_count=len(eligible),
            evaluations=evaluations,
        )
