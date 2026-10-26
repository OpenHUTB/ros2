"""Pure-Python curb-crossing policy.

The policy is deliberately independent of ROS and CARLA so it can be unit
tested on any machine. The ROS node in ``node.py`` is a thin adapter.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class CrossingPolicy:
    """Gap-acceptance parameters for one pedestrian."""

    min_time_to_collision: float = 3.0
    max_vehicle_distance: float = 12.0
    min_vehicle_distance: float = 1.5

    def validate(self) -> None:
        if self.min_time_to_collision < 0.0:
            raise ValueError("min_time_to_collision must be non-negative")
        if self.max_vehicle_distance <= 0.0:
            raise ValueError("max_vehicle_distance must be positive")
        if self.min_vehicle_distance < 0.0:
            raise ValueError("min_vehicle_distance must be non-negative")
        if self.min_vehicle_distance > self.max_vehicle_distance:
            raise ValueError("min_vehicle_distance must not exceed max_vehicle_distance")

    def should_wait(self, distance: float, closing_speed: float) -> bool:
        """Return whether the pedestrian should wait at the curb.

        ``distance`` is metres from vehicle to pedestrian. ``closing_speed`` is
        the vehicle speed projected onto the vehicle-to-pedestrian direction.
        A non-positive closing speed means the vehicle is not approaching.
        """

        self.validate()
        if distance < 0.0:
            raise ValueError("distance must be non-negative")
        if distance <= self.min_vehicle_distance:
            return True
        if distance > self.max_vehicle_distance:
            return False
        if closing_speed <= 0.1:
            return False
        return distance / closing_speed < self.min_time_to_collision

    def reason(self, distance: float, closing_speed: float) -> str:
        """Return a short human-readable reason for the decision."""

        if distance <= self.min_vehicle_distance:
            return "vehicle_inside_min_distance"
        if distance > self.max_vehicle_distance:
            return "vehicle_outside_scan_range"
        if closing_speed <= 0.1:
            return "vehicle_not_approaching"
        if distance / closing_speed < self.min_time_to_collision:
            return "ttc_below_threshold"
        return "gap_accepted"


@dataclass(frozen=True)
class CrossingDecision:
    wait: bool
    reason: str
    time_to_collision: Optional[float]

    @classmethod
    def evaluate(
        cls,
        policy: CrossingPolicy,
        distance: float,
        closing_speed: float,
    ) -> "CrossingDecision":
        ttc = None
        if closing_speed > 0.1 and distance > 0.0:
            ttc = distance / closing_speed
        return cls(
            wait=policy.should_wait(distance, closing_speed),
            reason=policy.reason(distance, closing_speed),
            time_to_collision=ttc,
        )
