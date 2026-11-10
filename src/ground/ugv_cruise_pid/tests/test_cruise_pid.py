#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""UGV PID 巡航速度跟踪的回归测试（纯 numpy，不需要 ROS / CARLA）。

运行::

    python -m unittest discover -s tests -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

from cruise_pid_node import (  # noqa: E402
    PIDCruiseController,
    UGVPlant,
    evaluate,
    simulate,
)


class TestCruisePID(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.times, cls.vs, cls.vts, cls.xs, cls.accels = simulate()
        cls.metrics = evaluate(cls.times, cls.vs, cls.vts, cls.xs)

    def test_step_steady_error_within_band(self):
        """8 m/s 阶跃平稳段（阵风前）稳态误差 < 0.2 m/s。"""
        self.assertLess(self.metrics["steady_err"], 0.2)

    def test_step_overshoot_within_band(self):
        """8 m/s 阶跃超调 < 0.3 m/s。"""
        self.assertLess(self.metrics["overshoot"], 0.3)

    def test_distance_in_reasonable_range(self):
        """20 s 仿真总行程落在合理区间（80–110 m）。"""
        self.assertGreater(self.metrics["distance"], 80.0)
        self.assertLess(self.metrics["distance"], 110.0)

    def test_final_speed_tracks_last_target(self):
        """末段速度收敛到 3 m/s 目标附近。"""
        self.assertGreater(self.metrics["final_speed"], 2.5)
        self.assertLess(self.metrics["final_speed"], 3.2)

    def test_controller_integral_is_anti_windup_bounded(self):
        """积分项被抗饱和限幅，不会随长时间误差发散。"""
        plant = UGVPlant()
        ctrl = PIDCruiseController()
        for _ in range(2000):
            ctrl.update(8.0, plant.v)
        self.assertLessEqual(ctrl.integral, ctrl.i_limit)
        self.assertGreaterEqual(ctrl.integral, -ctrl.i_limit)


if __name__ == "__main__":
    unittest.main()
