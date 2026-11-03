#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import unittest

import numpy as np

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
)

from controller import bearing_error_deg, body_frame_is_safe, compute_velocity  # noqa: E402


PARAMS = {
    "goal_tolerance_xy": 0.2,
    "goal_tolerance_z": 0.2,
    "forward_corridor_half_width": 1.2,
    "slow_distance": 4.0,
    "stop_distance": 1.0,
    "max_horizontal_speed": 2.0,
    "max_vertical_speed": 1.0,
    "max_horizontal_accel": 2.5,
    "max_vertical_accel": 1.5,
    "vertical_gain": 0.9,
    "repulsion_radius": 2.0,
    "repulsion_gain": 0.8,
    "vertical_influence": 1.2,
    "vertical_influence_below": 0.15,
}


class ControllerTest(unittest.TestCase):
    def test_moves_toward_goal(self):
        command = compute_velocity(
            np.array([0.0, 0.0, 2.0]),
            np.zeros(3),
            np.array([5.0, 0.0, 2.0]),
            2.0,
            np.empty((0, 3)),
            PARAMS,
            0.1,
        )
        self.assertGreater(command.vx, 0.0)
        self.assertLessEqual(abs(command.vy), 1e-9)
        self.assertLessEqual(abs(command.vz), 1e-9)

    def test_acceleration_limit(self):
        command = compute_velocity(
            np.array([0.0, 0.0, 0.0]),
            np.zeros(3),
            np.array([10.0, 0.0, 0.0]),
            0.0,
            np.empty((0, 3)),
            PARAMS,
            0.1,
        )
        self.assertLessEqual(command.vx, PARAMS["max_horizontal_accel"] * 0.1)

    def test_vertical_target_produces_upward_command(self):
        command = compute_velocity(
            np.array([0.0, 0.0, 0.0]),
            np.zeros(3),
            np.array([5.0, 0.0, 3.0]),
            2.0,
            np.empty((0, 3)),
            PARAMS,
            0.1,
        )
        self.assertGreater(command.vz, 0.0)

    def test_near_forward_obstacle_slows_horizontal_motion(self):
        command = compute_velocity(
            np.array([0.0, 0.0, 0.0]),
            np.zeros(3),
            np.array([5.0, 0.0, 0.0]),
            0.0,
            np.array([[0.5, 0.0, 0.0]]),
            PARAMS,
            0.1,
        )
        self.assertLessEqual(command.speed_scale, 1e-9)

    def test_altitude_separation_ignores_lower_obstacle(self):
        command = compute_velocity(
            np.array([0.0, 0.0, 0.0]),
            np.zeros(3),
            np.array([5.0, 0.0, 3.0]),
            3.0,
            np.array([[2.0, 0.0, 0.0]]),
            PARAMS,
            0.1,
        )
        self.assertAlmostEqual(command.speed_scale, 1.0)

    def test_ground_below_target_does_not_block_motion(self):
        command = compute_velocity(
            np.array([0.0, 0.0, 0.0]),
            np.zeros(3),
            np.array([5.0, 0.0, 0.0]),
            0.0,
            np.array([[0.5, 0.0, -0.4]]),
            PARAMS,
            0.1,
        )
        self.assertAlmostEqual(command.speed_scale, 1.0)

    def test_depth_guard_can_stop_horizontal_motion(self):
        command = compute_velocity(
            np.array([0.0, 0.0, 0.0]),
            np.zeros(3),
            np.array([5.0, 0.0, 0.0]),
            0.0,
            np.empty((0, 3)),
            PARAMS,
            0.1,
            depth_speed_scale=0.0,
        )
        self.assertAlmostEqual(command.speed_scale, 0.0)

    def test_depth_guard_overrides_repulsion(self):
        command = compute_velocity(
            np.array([0.0, 0.0, 0.0]),
            np.zeros(3),
            np.array([5.0, 0.0, 0.0]),
            0.0,
            np.array([[1.0, 0.2, 0.0]]),
            PARAMS,
            0.1,
            depth_speed_scale=0.0,
        )
        self.assertAlmostEqual(command.vx, 0.0)
        self.assertAlmostEqual(command.vy, 0.0)

    def test_bearing_error_rejects_lateral_goal(self):
        # 机头指向东，目标在北，夹角应为 90 度。
        self.assertAlmostEqual(
            bearing_error_deg(
                np.array([0.0, 0.0, 0.0]),
                np.array([0.0, 5.0, 0.0]),
                0.0,
            ),
            90.0,
        )

    def test_bearing_error_wraps_across_pi(self):
        self.assertAlmostEqual(
            bearing_error_deg(
                np.array([0.0, 0.0, 0.0]),
                np.array([5.0, 0.0, 0.0]),
                np.deg2rad(179.0),
            ),
            179.0,
        )

    def test_body_frame_requires_explicit_false(self):
        self.assertTrue(body_frame_is_safe(False))
        self.assertFalse(body_frame_is_safe(True))
        self.assertFalse(body_frame_is_safe(None))


if __name__ == "__main__":
    unittest.main()
