#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VFH+ unit tests: no ROS and no simulator required.

A. angle helpers and sector layout
B. polar obstacle density from sonar echoes
C. circular smoothing
D. valley selection, narrow-gap rejection, hysteresis
E. speed scaling by clearance

Run with: python3 test/test_vfh.py
"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from underwater_sonar_mapping.vfh import (  # noqa: E402
    angle_difference_deg, polar_histogram, sector_angles, select_direction,
    smooth_histogram, speed_scale, wrap_deg)


class AngleHelperTests(unittest.TestCase):
    def test_wrap_deg(self):
        self.assertAlmostEqual(wrap_deg(190.0), -170.0)
        self.assertAlmostEqual(wrap_deg(-190.0), 170.0)
        self.assertAlmostEqual(wrap_deg(0.0), 0.0)

    def test_angle_difference_is_signed_and_shortest(self):
        self.assertAlmostEqual(angle_difference_deg(10.0, 350.0), 20.0)
        self.assertAlmostEqual(angle_difference_deg(350.0, 10.0), -20.0)

    def test_sector_centres_cover_the_circle(self):
        angles = sector_angles(72)
        self.assertEqual(angles.size, 72)
        self.assertAlmostEqual(angles[0], -177.5)
        self.assertAlmostEqual(angles[-1], 177.5)


class HistogramTests(unittest.TestCase):
    def test_free_echoes_leave_the_histogram_empty(self):
        bearings = np.radians(np.linspace(-180, 180, 72, endpoint=False))
        ranges = np.full(72, 30.0)
        hist = polar_histogram(bearings, ranges, sectors=72, max_range=30.0)
        self.assertTrue(np.allclose(hist, 0.0))

    def test_echo_ahead_peaks_near_zero_degrees(self):
        bearings = np.array([0.0])
        ranges = np.array([3.0])
        hist = polar_histogram(bearings, ranges, sectors=72, max_range=30.0)
        self.assertAlmostEqual(hist.max(), 1.0)
        peak_angle = sector_angles(72)[int(np.argmax(hist))]
        self.assertLess(abs(peak_angle), 10.0)

    def test_close_echo_saturates_its_sector(self):
        bearings = np.array([0.0])
        ranges = np.array([0.4])  # inside robot_radius + safety_margin
        hist = polar_histogram(bearings, ranges, sectors=72, max_range=30.0,
                               robot_radius=0.6, safety_margin=0.5)
        self.assertAlmostEqual(hist.max(), 1.0)

    def test_close_echo_outweighs_a_distant_one(self):
        bearings = np.array([0.0, np.pi / 2.0])
        ranges = np.array([3.0, 20.0])
        hist = polar_histogram(bearings, ranges, sectors=72, max_range=30.0)
        angles = sector_angles(72)
        near = hist[int(np.argmin(np.abs(angles - 0.0)))]
        far = hist[int(np.argmin(np.abs(angles - 90.0)))]
        self.assertGreater(near, far)

    def test_extra_radius_widens_the_obstacle(self):
        narrow = polar_histogram(np.array([0.0]), np.array([3.0]), sectors=72,
                                 robot_radius=0.1, safety_margin=0.1)
        wide = polar_histogram(np.array([0.0]), np.array([3.0]), sectors=72,
                               robot_radius=1.5, safety_margin=0.5)
        self.assertGreater(np.count_nonzero(wide), np.count_nonzero(narrow))


class SmoothingTests(unittest.TestCase):
    def test_constant_histogram_is_unchanged(self):
        hist = np.full(72, 0.5)
        np.testing.assert_allclose(smooth_histogram(hist), hist)

    def test_spike_is_spread_to_neighbours(self):
        hist = np.zeros(72)
        hist[0] = 1.0
        smoothed = smooth_histogram(hist)
        self.assertLess(smoothed[0], 1.0)
        self.assertGreater(smoothed[1], 0.0)
        self.assertGreater(smoothed[-1], 0.0)
        self.assertAlmostEqual(smoothed.sum(), 1.0, places=6)


class SelectionTests(unittest.TestCase):
    def test_open_field_heads_straight_for_the_goal(self):
        hist = np.zeros(72)
        result = select_direction(hist, goal_bearing_deg=30.0)
        self.assertFalse(result.blocked)
        self.assertLess(abs(angle_difference_deg(result.angle_deg, 30.0)), 6.0)

    def test_blocked_sector_forces_a_detour(self):
        hist = np.zeros(72)
        hist[33:39] = 1.0  # roughly -15..+15 degrees
        result = select_direction(hist, goal_bearing_deg=0.0)
        self.assertFalse(result.blocked)
        self.assertGreater(abs(result.angle_deg), 10.0)

    def test_everything_blocked_reports_blocked(self):
        hist = np.ones(72)
        result = select_direction(hist, goal_bearing_deg=0.0)
        self.assertTrue(result.blocked)
        self.assertEqual(result.sector, -1)

    def test_narrow_gap_below_vehicle_width_is_rejected(self):
        hist = np.ones(72)
        hist[35] = 0.0  # a single 5 degree sector
        result = select_direction(hist, goal_bearing_deg=0.0, min_valley_deg=15.0)
        self.assertTrue(result.blocked)

    def test_hysteresis_keeps_the_previous_valley(self):
        hist = np.ones(72)
        hist[29:32] = 0.0   # valley A, centre about -27.5 deg
        hist[39:42] = 0.0   # valley B, centre about +22.5 deg

        fresh = select_direction(hist, goal_bearing_deg=0.0)
        kept = select_direction(hist, goal_bearing_deg=0.0, hysteresis_deg=15.0,
                                previous_deg=-27.5)

        self.assertGreater(fresh.angle_deg, 0.0)   # B is closer to the goal
        self.assertLess(kept.angle_deg, 0.0)       # but hysteresis keeps A
        self.assertAlmostEqual(kept.angle_deg, -22.5, delta=5.0)


class SpeedScaleTests(unittest.TestCase):
    def test_stop_slow_and_full_speed(self):
        self.assertEqual(speed_scale(0.5, slow_distance=2.5, stop_distance=0.8), 0.0)
        self.assertEqual(speed_scale(3.0, slow_distance=2.5, stop_distance=0.8), 1.0)
        mid = speed_scale(1.65, slow_distance=2.5, stop_distance=0.8)
        self.assertAlmostEqual(mid, 0.5, places=6)

    def test_monotonic(self):
        values = [speed_scale(d) for d in np.linspace(0.0, 3.0, 20)]
        self.assertEqual(values, sorted(values))


if __name__ == '__main__':
    unittest.main(verbosity=2)
