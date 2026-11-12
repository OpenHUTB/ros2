"""Synthetic underwater scene used by the sonar simulation.

Coordinates follow NED as used elsewhere in this repository: x forward,
y right, z **down** (a larger z is deeper).  Heights are returned as z values
in metres, so the seabed lies at ``z = base_depth + ...`` and an obstacle top is
a *smaller* z than the seabed it stands on.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class BoxObstacle:
    """Axis-aligned block standing on the seabed."""

    cx: float
    cy: float
    half_x: float
    half_y: float
    height: float  # metres above the local seabed

    def contains(self, x, y):
        return abs(x - self.cx) <= self.half_x and abs(y - self.cy) <= self.half_y


class Seabed:
    """Smooth undulating seabed built from two sinusoidal terms."""

    def __init__(self, base_depth=10.0, amplitude=1.5, period=12.5):
        if base_depth <= 0:
            raise ValueError('base_depth must be positive')
        if amplitude < 0:
            raise ValueError('amplitude must be nonnegative')
        if period <= 0:
            raise ValueError('period must be positive')
        self.base_depth = base_depth
        self.amplitude = amplitude
        self.period = period

    def height(self, x, y):
        k = 2.0 * math.pi / self.period
        return (self.base_depth
                + self.amplitude * math.sin(k * x)
                + self.amplitude * math.cos(k * y))


class World:
    """Seabed plus obstacles, with ray casting for the sonar model."""

    def __init__(self, seabed=None, obstacles=None):
        self.seabed = seabed or Seabed()
        self.obstacles = list(obstacles or [])

    def obstacle_top(self, box):
        """Top of the block as a z value (smaller is higher)."""
        return self.seabed.height(box.cx, box.cy) - box.height

    def _ray_box_distance(self, x0, y0, dx, dy, box):
        """Entry distance of a 2-D ray into an axis-aligned box, or None."""
        t_near, t_far = -math.inf, math.inf
        for origin, direction, lo, hi in (
                (x0, dx, box.cx - box.half_x, box.cx + box.half_x),
                (y0, dy, box.cy - box.half_y, box.cy + box.half_y)):
            if abs(direction) < 1e-12:
                if origin < lo or origin > hi:
                    return None
                continue
            t1 = (lo - origin) / direction
            t2 = (hi - origin) / direction
            if t1 > t2:
                t1, t2 = t2, t1
            t_near = max(t_near, t1)
            t_far = min(t_far, t2)
            if t_near > t_far:
                return None
        if t_far < 0:
            return None
        return max(t_near, 0.0)

    def cast_ray(self, x0, y0, z0, angle_rad, max_range, step=0.25):
        """Distance to the first return of a horizontal beam at depth ``z0``.

        A beam is blocked either by a block whose top rises above the beam
        depth, or by the seabed where the floor rises above the beam depth.
        Returns ``max_range`` when nothing is hit.
        """
        dx, dy = math.cos(angle_rad), math.sin(angle_rad)

        best = max_range
        for box in self.obstacles:
            if self.obstacle_top(box) >= z0:
                continue  # block is entirely below the beam
            distance = self._ray_box_distance(x0, y0, dx, dy, box)
            if distance is not None and distance < best:
                best = distance

        travelled = step
        while travelled < best:
            x = x0 + dx * travelled
            y = y0 + dy * travelled
            if self.seabed.height(x, y) <= z0:
                best = travelled
                break
            travelled += step
        return best


def ray_march_scan(world, x, y, z, bearings, max_range, step=0.25):
    """Vectorised wrapper: returns one range per bearing (rad, body frame)."""
    bearings = np.asarray(bearings, dtype=float)
    ranges = np.empty(bearings.shape, dtype=float)
    for i, bearing in enumerate(bearings):
        ranges[i] = world.cast_ray(x, y, z, bearing, max_range, step)
    return ranges
