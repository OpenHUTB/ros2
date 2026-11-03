#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import unittest

import numpy as np

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
)

from controller import (  # noqa: E402
    body_frame_is_safe,
    body_offsets_to_world,
    tracking_velocity,
)
from trajectory import TrajectorySample  # noqa: E402


PARAMS = {
    "max_horizontal_speed": 1.0,
    "max_vertical_speed": 0.6,
    "max_horizontal_accel": 1.0,
    "max_vertical_accel": 0.8,
    "position_gain_xy": 1.0,
    "position_gain_z": 1.0,
}


class ControllerTest(unittest.TestCase):
    def test_body_offsets_rotate_with_yaw(self):
        points = body_offsets_to_world(
            [0.0, 0.0, 2.0],
            np.pi / 2.0,
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.5]],
        )
        np.testing.assert_allclose(points[0], [0.0, 1.0, 2.0], atol=1e-9)
        np.testing.assert_allclose(points[1], [-1.0, 0.0, 2.5], atol=1e-9)

    def test_body_frame_requires_explicit_false(self):
        self.assertTrue(body_frame_is_safe(False))
        self.assertFalse(body_frame_is_safe(True))
        self.assertFalse(body_frame_is_safe(None))

    def test_tracking_uses_reference_velocity_without_error(self):
        reference = TrajectorySample(
            position=np.array([0.0, 0.0, 2.0]),
            velocity=np.array([0.3, 0.0, 0.0]),
            acceleration=np.zeros(3),
            jerk=np.zeros(3),
            elapsed=1.0,
            total_duration=3.0,
            segment_index=0,
        )
        command = tracking_velocity(
            reference.position,
            reference.velocity,
            reference,
            PARAMS,
            0.1,
        )
        self.assertAlmostEqual(command.vx, 0.3)
        self.assertAlmostEqual(command.vy, 0.0)
        self.assertAlmostEqual(command.vz, 0.0)

    def test_command_acceleration_limit(self):
        reference = TrajectorySample(
            position=np.array([5.0, 0.0, 0.0]),
            velocity=np.zeros(3),
            acceleration=np.zeros(3),
            jerk=np.zeros(3),
            elapsed=1.0,
            total_duration=3.0,
            segment_index=0,
        )
        command = tracking_velocity(
            np.zeros(3),
            np.zeros(3),
            reference,
            PARAMS,
            0.1,
        )
        self.assertLessEqual(command.vx, PARAMS["max_horizontal_accel"] * 0.1)


if __name__ == "__main__":
    unittest.main()
