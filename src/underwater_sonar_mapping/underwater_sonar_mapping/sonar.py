"""Multibeam scanning sonar model.

The sonar fans ``beams`` acoustic rays across ``fov_deg`` around the vehicle
heading, at the vehicle's current depth.  Each ray returns the first echo
distance from :class:`~underwater_sonar_mapping.terrain.World`, optionally
perturbed by range noise and dropouts, which is what a real scanning sonar head
produces.
"""
import numpy as np

from .terrain import ray_march_scan


class ScanningSonar:
    def __init__(self, beams=72, fov_deg=360.0, max_range=30.0, min_range=0.3,
                 range_sigma=0.02, dropout=0.0):
        if beams < 3:
            raise ValueError('beams must be at least 3')
        if not 0 < fov_deg <= 360.0:
            raise ValueError('fov_deg must be in (0, 360]')
        if min_range <= 0 or max_range <= min_range:
            raise ValueError('require 0 < min_range < max_range')
        if range_sigma < 0:
            raise ValueError('range_sigma must be nonnegative')
        if not 0.0 <= dropout < 1.0:
            raise ValueError('dropout must be in [0, 1)')
        self.beams = int(beams)
        self.fov_deg = float(fov_deg)
        self.max_range = float(max_range)
        self.min_range = float(min_range)
        self.range_sigma = float(range_sigma)
        self.dropout = float(dropout)

    def body_bearings(self):
        """Beam bearings in the body frame (rad), centred on the heading."""
        half = np.deg2rad(self.fov_deg) / 2.0
        return np.linspace(-half, half, self.beams, endpoint=False)

    def scan(self, world, x, y, z, yaw, rng=None):
        """Return ``(body_bearings, ranges)`` for one revolution.

        ``rng`` is an ``numpy.random.Generator``; pass one with a fixed seed for
        reproducible runs.  Without it the model is noise-free.
        """
        body = self.body_bearings()
        world_bearings = yaw + body
        ranges = ray_march_scan(world, x, y, z, world_bearings, self.max_range)

        if rng is not None:
            if self.range_sigma > 0:
                ranges = ranges + rng.normal(0.0, self.range_sigma, ranges.shape)
            if self.dropout > 0:
                missed = rng.random(ranges.shape) < self.dropout
                ranges = np.where(missed, self.max_range, ranges)

        return body, np.clip(ranges, self.min_range, self.max_range)
