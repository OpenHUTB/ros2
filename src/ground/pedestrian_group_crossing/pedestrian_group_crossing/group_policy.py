"""Small, testable group-crossing policy.

This module models the social part of a crossing decision: a group waits until
enough members have gathered, the group is not too spread out, and the gap is
acceptable. It intentionally does not depend on CARLA or ROS.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GroupCrossingPolicy:
    group_size: int = 3
    cohesion_ratio: float = 0.8
    min_gap_seconds: float = 3.0
    max_spread_m: float = 2.0
    member_delay_s: float = 0.25

    def validate(self) -> None:
        if self.group_size < 1:
            raise ValueError("group_size must be at least 1")
        if not 0.0 < self.cohesion_ratio <= 1.0:
            raise ValueError("cohesion_ratio must be in (0, 1]")
        if self.min_gap_seconds < 0.0:
            raise ValueError("min_gap_seconds must be non-negative")
        if self.max_spread_m < 0.0 or self.member_delay_s < 0.0:
            raise ValueError("spread and member delay must be non-negative")

    def evaluate(
        self,
        arrived_count: int,
        spread_m: float,
        gap_seconds: float,
        leader_waiting: bool,
    ) -> "GroupCrossingDecision":
        self.validate()
        if not 0 <= arrived_count <= self.group_size:
            raise ValueError("arrived_count must be within [0, group_size]")
        if spread_m < 0.0 or gap_seconds < 0.0:
            raise ValueError("spread_m and gap_seconds must be non-negative")

        required = max(1, int(math.ceil(self.group_size * self.cohesion_ratio)))
        reasons = []
        if arrived_count < required:
            reasons.append("waiting_for_group")
        if spread_m > self.max_spread_m:
            reasons.append("group_too_spread")
        if leader_waiting:
            reasons.append("leader_waiting")
        if gap_seconds < self.min_gap_seconds:
            reasons.append("gap_below_threshold")

        return GroupCrossingDecision(
            wait=bool(reasons),
            reason=reasons[0] if reasons else "group_clear_to_cross",
            arrived_count=arrived_count,
            required_count=required,
            member_delay_s=self.member_delay_s,
        )


@dataclass(frozen=True)
class GroupCrossingDecision:
    wait: bool
    reason: str
    arrived_count: int
    required_count: int
    member_delay_s: float

    def as_dict(self) -> dict:
        return {
            "wait": self.wait,
            "reason": self.reason,
            "arrived_count": self.arrived_count,
            "required_count": self.required_count,
            "member_delay_s": self.member_delay_s,
        }
