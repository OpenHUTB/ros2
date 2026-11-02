"""Pose and path primitives shared by the sensor, utility and planner modules."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import numpy as np

__all__ = ["Pose", "wrap_angle_deg", "heading_between"]


@dataclass(frozen=True)
class Pose:
    """A planar pose in the local NED frame.

    ``x`` is north, ``y`` is east (both metres), ``yaw_deg`` is the heading in
    degrees, measured counter-clockwise from the north axis.  Altitude is a
    vehicle-level constant for coverage missions and lives in the
    :class:`~ap_cpp.sensor.SensorConfig`.
    """

    x: float
    y: float
    yaw_deg: float = 0.0

    @property
    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y], dtype=float)

    def moved_to(self, x: float, y: float, yaw_deg: float = None) -> "Pose":
        return Pose(x, y, self.yaw_deg if yaw_deg is None else yaw_deg)

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.yaw_deg)

    def __iter__(self):
        # Allows ``x, y, yaw = pose`` and tuple-unpacking in call sites that
        # predate the dataclass.
        yield self.x
        yield self.y
        yield self.yaw_deg


def wrap_angle_deg(angle: float) -> float:
    """Wrap an angle to ``(-180, 180]``."""
    return (angle + 180.0) % 360.0 - 180.0


def heading_between(p0: Pose, p1: Pose) -> float:
    """Heading in degrees of the vector ``p0 -> p1``, measured from north."""
    dx = p1.x - p0.x
    dy = p1.y - p0.y
    if abs(dx) < 1e-12 and abs(dy) < 1e-12:
        return p0.yaw_deg
    return math.degrees(math.atan2(dy, dx))


def poses_from_xy(points: Sequence[Sequence[float]], yaw_deg: float = 0.0) -> List[Pose]:
    """Convert a ``[(x, y), ...]`` sequence into poses with a fixed heading."""
    return [Pose(float(p[0]), float(p[1]), yaw_deg) for p in points]
