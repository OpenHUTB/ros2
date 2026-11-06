# -*- coding: utf-8 -*-
"""差速运动学纯函数测试（不依赖 MuJoCo）。"""
import unittest

from mujoco_car_rollover_guard.kinematics import differential_wheel_vels

V_MAX = 2.0
OMEGA_MAX = 3.0
WHEEL_R = 0.05
TRACK = 0.18
W_MAX = 60.0


def vels(steer, throttle):
    return differential_wheel_vels(steer, throttle, V_MAX, OMEGA_MAX,
                                   WHEEL_R, TRACK, W_MAX)


class TestKinematics(unittest.TestCase):
    def test_straight_motion_is_symmetric(self):
        wl, wr = vels(0.0, 0.5)
        self.assertAlmostEqual(wl, wr)
        self.assertAlmostEqual(wl, 0.5 * V_MAX / WHEEL_R)

    def test_left_steer_slows_left_wheel(self):
        wl, wr = vels(0.6, 0.5)
        self.assertLess(wl, wr)

    def test_right_steer_slows_right_wheel(self):
        wl, wr = vels(-0.6, 0.5)
        self.assertGreater(wl, wr)

    def test_throttle_upper_clamped(self):
        wl, _ = vels(0.0, 1.5)
        self.assertAlmostEqual(wl, V_MAX / WHEEL_R)

    def test_throttle_negative_clamped_to_zero(self):
        wl, wr = vels(0.0, -0.3)
        self.assertEqual(wl, 0.0)
        self.assertEqual(wr, 0.0)

    def test_wheel_speed_clamped(self):
        wl, wr = vels(1.0, 0.0)
        self.assertGreaterEqual(wl, -W_MAX)
        self.assertLessEqual(wr, W_MAX)


if __name__ == "__main__":
    unittest.main()
