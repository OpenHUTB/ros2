"""Planar two-link kinematics used by the reaching controller."""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, hypot, pi, sin
from typing import Tuple


@dataclass(frozen=True)
class ArmGeometry:
    """Link lengths for the two-joint arm."""

    upper_arm_length_m: float = 0.30
    forearm_length_m: float = 0.28
    minimum_elbow_rad: float = -0.10
    maximum_elbow_rad: float = 2.30

    def __post_init__(self) -> None:
        if self.upper_arm_length_m <= 0.0 or self.forearm_length_m <= 0.0:
            raise ValueError('link lengths must be positive')
        if self.minimum_elbow_rad >= self.maximum_elbow_rad:
            raise ValueError('elbow limits must be ordered')

    @property
    def reach_min_m(self) -> float:
        return abs(self.upper_arm_length_m - self.forearm_length_m)

    @property
    def reach_max_m(self) -> float:
        return self.upper_arm_length_m + self.forearm_length_m


def forward_kinematics(
    shoulder_rad: float,
    elbow_rad: float,
    geometry: ArmGeometry = ArmGeometry(),
) -> Tuple[float, float]:
    """Return end-effector x/z coordinates in the arm plane."""
    x = (
        geometry.upper_arm_length_m * cos(shoulder_rad)
        + geometry.forearm_length_m * cos(shoulder_rad + elbow_rad)
    )
    z = (
        -geometry.upper_arm_length_m * sin(shoulder_rad)
        - geometry.forearm_length_m * sin(shoulder_rad + elbow_rad)
    )
    return x, z


def inverse_kinematics(
    target_x_m: float,
    target_z_m: float,
    geometry: ArmGeometry = ArmGeometry(),
) -> Tuple[float, float]:
    """Solve elbow-up inverse kinematics for a reachable planar target."""
    radius = hypot(target_x_m, target_z_m)
    tolerance = 1e-9
    if radius < geometry.reach_min_m - tolerance:
        raise ValueError('target is inside the minimum reach')
    if radius > geometry.reach_max_m + tolerance:
        raise ValueError('target is outside the maximum reach')

    l1 = geometry.upper_arm_length_m
    l2 = geometry.forearm_length_m
    cosine_elbow = (
        radius * radius - l1 * l1 - l2 * l2
    ) / (2.0 * l1 * l2)
    cosine_elbow = max(-1.0, min(1.0, cosine_elbow))
    elbow = acos(cosine_elbow)
    shoulder = atan2(-target_z_m, target_x_m) - atan2(
        l2 * sin(elbow),
        l1 + l2 * cos(elbow),
    )
    shoulder = (shoulder + pi) % (2.0 * pi) - pi
    if not (
        geometry.minimum_elbow_rad
        <= elbow
        <= geometry.maximum_elbow_rad
    ):
        raise ValueError('target requires an elbow angle outside the limits')
    return shoulder, elbow
