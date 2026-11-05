# -*- coding: utf-8 -*-
"""速度整形与差速运动学单测（不需要 MuJoCo）。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mujoco_car_avoidance.controller import (  # noqa: E402
    RampParams, SpeedRamp, differential_drive,
)


class TestSpeedRamp(unittest.TestCase):
    """加减速限幅。"""

    def setUp(self):
        self.ramp = SpeedRamp(RampParams(acc_max=2.0, brake_max=3.5))

    def test_acceleration_is_limited(self):
        dt = 0.02
        speed = self.ramp.update(2.0, dt)
        self.assertAlmostEqual(speed, 2.0 * dt, places=9)

    def test_reaches_target_without_overshoot(self):
        dt = 0.02
        for _ in range(200):
            speed = self.ramp.update(1.0, dt)
        self.assertAlmostEqual(speed, 1.0, places=9)

    def test_braking_is_limited(self):
        self.ramp.reset(2.0)
        speed = self.ramp.update(0.0, 0.02)
        self.assertAlmostEqual(speed, 2.0 - 3.5 * 0.02, places=9)

    def test_stops_at_zero_after_enough_time(self):
        self.ramp.reset(1.0)
        for _ in range(200):
            speed = self.ramp.update(0.0, 0.02)
        self.assertAlmostEqual(speed, 0.0, places=9)

    def test_negative_target_is_clamped(self):
        self.ramp.reset(0.5)
        speed = self.ramp.update(-3.0, 0.02)
        self.assertGreaterEqual(speed, 0.0)

    def test_zero_dt_keeps_speed(self):
        self.ramp.reset(0.7)
        self.assertAlmostEqual(self.ramp.update(2.0, 0.0), 0.7, places=9)


class TestDifferentialDrive(unittest.TestCase):
    """(v, omega) -> 左右轮角速度。"""

    def test_straight_line(self):
        left, right, saturated = differential_drive(1.0, 0.0, 0.05, 0.25)
        self.assertAlmostEqual(left, 20.0, places=9)
        self.assertAlmostEqual(right, 20.0, places=9)
        self.assertFalse(saturated)

    def test_left_turn_slows_left_wheel(self):
        left, right, _ = differential_drive(1.0, 1.0, 0.05, 0.25)
        self.assertLess(left, right)

    def test_rotation_in_place(self):
        left, right, _ = differential_drive(0.0, 1.0, 0.05, 0.25)
        self.assertAlmostEqual(left, -right, places=9)

    def test_saturation_flag_and_clamp(self):
        left, right, saturated = differential_drive(5.0, 0.0, 0.05, 0.25,
                                                    max_wheel_omega=60.0)
        self.assertTrue(saturated)
        self.assertAlmostEqual(left, 60.0, places=9)
        self.assertAlmostEqual(right, 60.0, places=9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
