# -*- coding: utf-8 -*-
"""差速运动学单测（不需要 MuJoCo）。

(steer, throttle) 到左右轮角速度的解算，与 car_env 共用 kinematics.py 里的纯函数。
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mujoco_car_nn_avoidance.config import Config  # noqa: E402
from mujoco_car_nn_avoidance.kinematics import differential_wheel_vels  # noqa: E402


class TestDifferentialDrive(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()
        self.kw = dict(
            v_max=self.cfg.v_max,
            omega_max=self.cfg.omega_max,
            wheel_r=self.cfg.wheel_r,
            track=self.cfg.track,
            max_wheel_omega=self.cfg.max_wheel_omega,
        )

    def test_straight_line_wheels_match(self):
        w_l, w_r = differential_wheel_vels(0.0, 0.8, **self.kw)
        self.assertAlmostEqual(w_l, w_r, places=9)
        self.assertGreater(w_l, 0.0)

    def test_turning_left_speeds_right_wheel(self):
        # steer>0（yaw 增大，左转）：左轮慢、右轮快
        w_l, w_r = differential_wheel_vels(1.0, 0.5, **self.kw)
        self.assertLess(w_l, w_r)

    def test_turning_right_speeds_left_wheel(self):
        # steer<0（yaw 减小，右转）：左轮快、右轮慢
        w_l, w_r = differential_wheel_vels(-1.0, 0.5, **self.kw)
        self.assertGreater(w_l, w_r)

    def test_zero_throttle_rotates_in_place(self):
        w_l, w_r = differential_wheel_vels(1.0, 0.0, **self.kw)
        self.assertAlmostEqual(w_l, -w_r, places=9)

    def test_wheel_velocity_is_clamped(self):
        # 油门与转向同时给满，轮速也不得超过 ctrlrange 上限
        w_l, w_r = differential_wheel_vels(1.0, 1.0, **self.kw)
        self.assertLessEqual(abs(w_l), self.cfg.max_wheel_omega + 1e-9)
        self.assertLessEqual(abs(w_r), self.cfg.max_wheel_omega + 1e-9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
