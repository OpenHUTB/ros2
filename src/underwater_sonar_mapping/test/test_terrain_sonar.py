#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Seabed / sonar geometry tests: no ROS and no simulator required.

A. seabed height field is analytic and configurable
B. horizontal ray casting against blocks and the rising floor
C. scanning sonar beam layout, range clipping and seeded noise

Run with: python3 test/test_terrain_sonar.py
"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from underwater_sonar_mapping.sonar import ScanningSonar          # noqa: E402
from underwater_sonar_mapping.terrain import (BoxObstacle, Seabed,  # noqa: E402
                                              World)


class Slope:
    """Floor rising towards -x, used to exercise the seabed branch."""

    @staticmethod
    def height(x, y):
        return 12.0 - 0.5 * x


class SeabedTests(unittest.TestCase):
    def test_flat_seabed_keeps_base_depth(self):
        bed = Seabed(base_depth=10.0, amplitude=0.0)
        for x, y in ((0, 0), (3, -7), (100, 42)):
            self.assertAlmostEqual(bed.height(x, y), 10.0)

    def test_amplitude_oscillates_around_base(self):
        bed = Seabed(base_depth=10.0, amplitude=1.5, period=12.5)
        xs = np.linspace(0.0, 12.5, 60)
        ys = np.linspace(0.0, 12.5, 60)
        samples = [bed.height(x, y) for x in xs for y in ys]
        self.assertLess(min(samples), 10.0 - 1.5)
        self.assertGreater(max(samples), 10.0 + 1.5)
        self.assertGreaterEqual(min(samples), 10.0 - 3.0 - 1e-9)
        self.assertLessEqual(max(samples), 10.0 + 3.0 + 1e-9)

    def test_invalid_parameters_rejected(self):
        for kwargs in ({'base_depth': 0}, {'amplitude': -1}, {'period': 0}):
            with self.assertRaises(ValueError):
                Seabed(**kwargs)


class RayCastingTests(unittest.TestCase):
    def world(self, boxes=(), bed=None):
        return World(bed or Seabed(base_depth=10.0, amplitude=0.0), list(boxes))

    def test_no_return_in_empty_water(self):
        world = self.world()
        self.assertAlmostEqual(world.cast_ray(0, 0, 6.0, 0.0, 30.0), 30.0)

    def test_block_ahead_is_hit_at_its_face(self):
        world = self.world([BoxObstacle(5.0, 0.0, 1.0, 1.0, 8.0)])
        # Block spans x 4..6, top at 2 m depth, so a beam at 6 m depth hits it.
        self.assertAlmostEqual(world.cast_ray(0, 0, 6.0, 0.0, 30.0), 4.0)

    def test_block_below_the_beam_is_ignored(self):
        world = self.world([BoxObstacle(5.0, 0.0, 1.0, 1.0, 1.0)])
        # Top at 9 m depth while the beam runs at 6 m: the beam passes above it.
        self.assertAlmostEqual(world.cast_ray(0, 0, 6.0, 0.0, 30.0), 30.0)

    def test_block_beside_the_beam_is_ignored(self):
        world = self.world([BoxObstacle(5.0, 6.0, 1.0, 1.0, 8.0)])
        self.assertAlmostEqual(world.cast_ray(0, 0, 6.0, 0.0, 30.0), 30.0)

    def test_rising_floor_terminates_the_beam(self):
        world = World(Slope(), [])
        # Floor is 12 - 0.5x, beam at 9.5 m depth: hit where the floor reaches it.
        self.assertAlmostEqual(world.cast_ray(0, 0, 9.5, 0.0, 30.0), 5.0, delta=0.3)

    def test_block_inside_range_limit_only(self):
        world = self.world([BoxObstacle(40.0, 0.0, 1.0, 1.0, 8.0)])
        self.assertAlmostEqual(world.cast_ray(0, 0, 6.0, 0.0, 20.0), 20.0)


class ScanningSonarTests(unittest.TestCase):
    def test_beam_layout_for_full_revolution(self):
        sonar = ScanningSonar(beams=72, fov_deg=360.0)
        bearings = sonar.body_bearings()
        self.assertEqual(bearings.shape, (72,))
        self.assertAlmostEqual(bearings[0], -math.pi)
        self.assertAlmostEqual(math.degrees(bearings[1] + math.pi), 5.0)

    def test_partial_fov_stays_inside_limits(self):
        sonar = ScanningSonar(beams=10, fov_deg=90.0)
        bearings = np.degrees(sonar.body_bearings())
        self.assertGreaterEqual(bearings.min(), -45.0)
        self.assertLess(bearings.max(), 45.0)

    def test_scan_ranges_are_clipped(self):
        world = World(Seabed(amplitude=0.0), [BoxObstacle(5.0, 0.0, 1.0, 1.0, 8.0)])
        sonar = ScanningSonar(beams=36, max_range=12.0, min_range=0.5)
        _, ranges = sonar.scan(world, 0, 0, 6.0, 0.0)
        self.assertTrue(np.all(ranges >= 0.5))
        self.assertTrue(np.all(ranges <= 12.0))

    def test_forward_beam_sees_the_block(self):
        world = World(Seabed(amplitude=0.0), [BoxObstacle(5.0, 0.0, 1.0, 1.0, 8.0)])
        sonar = ScanningSonar(beams=72, fov_deg=360.0)
        bearings, ranges = sonar.scan(world, 0, 0, 6.0, 0.0)
        forward = int(np.argmin(np.abs(bearings)))
        self.assertAlmostEqual(ranges[forward], 4.0, delta=1.0)

    def test_noise_is_reproducible_with_a_seed(self):
        world = World(Seabed(amplitude=0.0), [BoxObstacle(5.0, 0.0, 1.0, 1.0, 8.0)])
        sonar = ScanningSonar(beams=36, range_sigma=0.2)
        _, a = sonar.scan(world, 0, 0, 6.0, 0.0, np.random.default_rng(3))
        _, b = sonar.scan(world, 0, 0, 6.0, 0.0, np.random.default_rng(3))
        _, c = sonar.scan(world, 0, 0, 6.0, 0.0, np.random.default_rng(4))
        np.testing.assert_allclose(a, b)
        self.assertFalse(np.allclose(a, c))

    def test_invalid_parameters_rejected(self):
        for kwargs in ({'beams': 2}, {'fov_deg': 0}, {'fov_deg': 361},
                       {'min_range': 0}, {'max_range': 1.0, 'min_range': 2.0},
                       {'range_sigma': -1}, {'dropout': 1.0}):
            with self.assertRaises(ValueError):
                ScanningSonar(**kwargs)


if __name__ == '__main__':
    unittest.main(verbosity=2)
