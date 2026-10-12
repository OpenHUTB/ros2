"""Model-free AEB logic shared by the simulator and ROS adapter."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import inf


class SafetyState(str, Enum):
    SAFE = "safe"
    WARNING = "warning"
    BRAKING = "braking"
    STOPPED = "stopped"


@dataclass
class AEBConfig:
    """Thresholds for the forward collision warning and emergency braking."""

    warning_ttc: float = 2.5
    braking_ttc: float = 1.2
    stop_distance: float = 1.0
    max_brake: float = 8.0
    response_delay: float = 0.15
    warning_distance_factor: float = 1.8
    release_factor: float = 3.0
    min_release_speed: float = 2.0


@dataclass
class AEBDecision:
    state: SafetyState
    ttc: float
    brake: float
    target_speed: float
    required_distance: float
    reason: str


class AEBController:
    """Decide whether to warn or brake from obstacle distance and ego speed.

    Two criteria are combined:

    1. distance criterion -- be able to stop within the available distance,
       ``d_req = v^2 / (2 * a_max) + v * t_response + d_stop``;
    2. time criterion -- time to collision, ``TTC = d / v``.

    Distance is the primary criterion because a pure TTC threshold stops being
    meaningful once the vehicle has slowed down: the drop in speed raises TTC
    again, so the brake command would chatter between full braking and warning.
    Emergency braking is therefore latched once triggered, and the latch is
    released only when the obstacle is genuinely far away, that is when both
    ``d > release_distance`` and ``TTC > warning_ttc`` hold, where
    ``release_distance = max(release_factor * warning_ttc * max(v, min_release_speed),
    warning_distance_factor * d_req)``. The speed floor keeps the threshold from
    collapsing while the vehicle is nearly stopped. A static obstacle in front
    therefore keeps the vehicle braking monotonically to a standstill instead of
    creeping into it.
    """

    def __init__(self, config: AEBConfig | None = None):
        self.config = config or AEBConfig()
        self._latched = False

    def reset(self) -> None:
        self._latched = False

    @property
    def latched(self) -> bool:
        return self._latched

    def required_stopping_distance(self, ego_speed: float) -> float:
        cfg = self.config
        speed = max(0.0, float(ego_speed))
        if cfg.max_brake <= 0.0:
            return inf
        return speed * speed / (2.0 * cfg.max_brake) + speed * cfg.response_delay + cfg.stop_distance

    def _release_distance(self, ego_speed: float, required: float) -> float:
        cfg = self.config
        reference_speed = max(max(0.0, float(ego_speed)), cfg.min_release_speed)
        return max(
            cfg.release_factor * cfg.warning_ttc * reference_speed,
            cfg.warning_distance_factor * required,
        )

    def decide(self, distance: float, ego_speed: float) -> AEBDecision:
        distance = max(0.0, float(distance))
        speed = max(0.0, float(ego_speed))
        cfg = self.config
        ttc = distance / speed if speed > 1e-6 else inf
        required = self.required_stopping_distance(speed)
        release_distance = self._release_distance(speed, required)

        if self._latched and distance > release_distance and ttc > cfg.warning_ttc:
            self._latched = False

        if distance <= cfg.stop_distance:
            self._latched = True
            return AEBDecision(SafetyState.STOPPED, ttc, cfg.max_brake, 0.0, required, "stop distance reached")

        if self._latched or distance <= required or ttc <= cfg.braking_ttc:
            self._latched = True
            return AEBDecision(SafetyState.BRAKING, ttc, cfg.max_brake, 0.0, required, "stopping distance exceeded")

        warning_limit = required * cfg.warning_distance_factor
        if distance <= warning_limit or ttc <= cfg.warning_ttc:
            brake = min(cfg.max_brake * 0.35, max(0.0, (warning_limit - distance) * 0.5))
            return AEBDecision(
                SafetyState.WARNING,
                ttc,
                brake,
                max(0.0, speed - brake * 0.15),
                required,
                "approaching required stopping distance",
            )

        return AEBDecision(SafetyState.SAFE, ttc, 0.0, speed, required, "safe following distance")

    def update(self, distance: float, ego_speed: float, dt: float) -> tuple[float, AEBDecision]:
        """Advance a simple longitudinal model and return (new_speed, decision)."""
        decision = self.decide(distance, ego_speed)
        new_speed = max(0.0, float(ego_speed) - decision.brake * max(0.0, dt))
        return new_speed, decision
