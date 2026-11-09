#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import unittest

import numpy as np

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
)

from trajectory import (  # noqa: E402
    MinimumJerkTrajectory,
    minimum_jerk_duration,
    minimum_jerk_state,
)


PARAMS = {
    "max_speed_horizontal": 1.0,
    "max_speed_vertical": 0.8,
    "max_acceleration_horizontal": 0.7,
    "max_acceleration_vertical": 0.5,
    "min_segment_duration": 0.5,
}


class TrajectoryTest(unittest.TestCase):
    def test_minimum_jerk_boundary_conditions(self):
        start = np.array([1.0, 2.0, 3.0])
        end = np.array([4.0, 2.0, 5.0])
        duration = 3.0
        p0, v0, a0, _ = minimum_jerk_state(start, end, duration, 0.0)
        p1, v1, a1, _ = minimum_jerk_state(start, end, duration, duration)
        np.testing.assert_allclose(p0, start)
        np.testing.assert_allclose(p1, end)
        np.testing.assert_allclose(v0, 0.0, atol=1e-12)
        np.testing.assert_allclose(v1, 0.0, atol=1e-12)
        np.testing.assert_allclose(a0, 0.0, atol=1e-12)
        np.testing.assert_allclose(a1, 0.0, atol=1e-12)

    def test_minimum_jerk_constraints_are_used_in_duration(self):
        duration = minimum_jerk_duration(2.0, 1.0, 0.5, 0.5)
        self.assertGreaterEqual(duration, 1.875 * 2.0 / 1.0)
        self.assertGreaterEqual(duration, (5.7735 * 2.0 / 0.5) ** 0.5)

    def test_multisegment_reaches_last_waypoint(self):
        trajectory = MinimumJerkTrajectory(
            [0.0, 0.0, 0.0],
            [[2.0, 0.0, 1.0], [4.0, 0.0, 2.0]],
            PARAMS,
        )
        sample = trajectory.sample(trajectory.total_duration)
        np.testing.assert_allclose(sample.position, [4.0, 0.0, 2.0])
        self.assertEqual(sample.segment_index, 1)

    def test_linear_mode_has_constant_segment_velocity(self):
        trajectory = MinimumJerkTrajectory(
            [0.0, 0.0, 0.0],
            [[2.0, 0.0, 0.0]],
            PARAMS,
            mode="linear",
        )
        middle = trajectory.sample(0.5 * trajectory.total_duration)
        np.testing.assert_allclose(middle.position, [1.0, 0.0, 0.0])
        np.testing.assert_allclose(middle.velocity, [2.0 / trajectory.total_duration, 0.0, 0.0])

    def test_linear_endpoint_velocity_is_zero(self):
        trajectory = MinimumJerkTrajectory(
            [0.0, 0.0, 0.0],
            [[2.0, 0.0, 0.0]],
            PARAMS,
            mode="linear",
        )
        endpoint = trajectory.sample(trajectory.total_duration)
        np.testing.assert_allclose(endpoint.position, [2.0, 0.0, 0.0])
        np.testing.assert_allclose(endpoint.velocity, 0.0, atol=1e-12)

    def test_zero_length_waypoint_is_skipped(self):
        trajectory = MinimumJerkTrajectory(
            [0.0, 0.0, 0.0],
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            PARAMS,
        )
        self.assertEqual(len(trajectory.segments), 1)


if __name__ == "__main__":
    unittest.main()
