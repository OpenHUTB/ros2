# -*- coding: utf-8 -*-
"""打靶弹道解算与成绩统计的离线测试（纯数学，不依赖 mujoco）.

    python3 tests/test_mujoco_shooting_ballistics.py

覆盖：
* ``ballistics.solve_launch_velocity`` 的弹道反解精度（无风 / 有风）
* ``controller.launch_velocity`` 对风偏的补偿与 ``boost`` 的语义
* ``ballistics.score_shots`` 的成绩统计
"""
from __future__ import annotations

import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "ground", "mujoco_shooting")
sys.path.insert(0, PKG)

import controller as ctrl  # noqa: E402
from ballistics import (  # noqa: E402
    predict_position,
    score_shots,
    solve_launch_velocity,
    wrap_angle,
)


class SolveLaunchTests(unittest.TestCase):
    START = (0.63, 0.0, 0.17)
    TARGET = (3.40, 0.0, 0.25)

    def test_no_wind_hits_target(self):
        v = solve_launch_velocity(self.START, self.TARGET, 0.45)
        p = predict_position(self.START, v, 0.45)
        for got, want in zip(p, self.TARGET):
            self.assertAlmostEqual(got, want, places=12)

    def test_wind_compensated(self):
        wind = (3.0, -2.0)
        v = solve_launch_velocity(self.START, self.TARGET, 0.45, wind=wind)
        p = predict_position(self.START, v, 0.45, wind=wind)
        for got, want in zip(p, self.TARGET):
            self.assertAlmostEqual(got, want, places=12)

    def test_wind_reduces_forward_speed(self):
        v0 = solve_launch_velocity(self.START, self.TARGET, 0.45)
        v1 = solve_launch_velocity(self.START, self.TARGET, 0.45, wind=(3.0, 0.0))
        self.assertLess(v1[0], v0[0])
        self.assertAlmostEqual(v0[0] - v1[0], 0.5 * 3.0 * 0.45, places=12)

    def test_vertical_component(self):
        v = solve_launch_velocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0)
        self.assertAlmostEqual(v[2], 0.5 * 9.81, places=12)

    def test_invalid_flight_time(self):
        with self.assertRaises(ValueError):
            solve_launch_velocity(self.START, self.TARGET, 0.0)

    def test_invalid_dimensions(self):
        with self.assertRaises(ValueError):
            solve_launch_velocity((0.0, 0.0), self.TARGET, 0.4)


class ControllerWindTests(unittest.TestCase):
    """controller.launch_velocity 是主循环真正调用的入口."""

    def test_matches_existing_behaviour_without_wind(self):
        muzzle = ctrl.muzzle_position(0.0)
        target = (2.0, 0.3)
        vx, vy, vz, dist = ctrl.launch_velocity(muzzle, target, flight_time=0.45)
        t = 0.45
        x = muzzle[0] + vx * t
        y = muzzle[1] + vy * t
        z = ctrl.MUZZLE_Z + vz * t - 0.5 * ctrl.GRAVITY * t * t
        self.assertAlmostEqual(math.hypot(x - target[0], y - target[1]), 0.0, places=6)
        self.assertAlmostEqual(z, ctrl.TARGET_Z, places=6)

    def test_wind_is_compensated(self):
        muzzle = ctrl.muzzle_position(0.0)
        target = (3.0, 1.0)
        wind = (2.0, -1.0)
        vx, vy, vz, _dist = ctrl.launch_velocity(muzzle, target, wind=wind)
        x, y, z = predict_position(
            (muzzle[0], muzzle[1], ctrl.MUZZLE_Z), (vx, vy, vz), 0.45, wind=wind)
        self.assertAlmostEqual(x, target[0], places=6)
        self.assertAlmostEqual(y, target[1], places=6)
        self.assertAlmostEqual(z, ctrl.TARGET_Z, places=6)

    def test_uncompensated_wind_misses_by_expected_offset(self):
        """不做补偿时，落点偏移应等于 ½·a·t²（这就是补偿要扣掉的部分）."""
        muzzle = ctrl.muzzle_position(0.0)
        target = (3.0, 0.0)
        t, wind_x = 0.45, 3.0
        vx, vy, vz, _ = ctrl.launch_velocity(muzzle, target)
        x, _y, _z = predict_position(
            (muzzle[0], muzzle[1], ctrl.MUZZLE_Z), (vx, vy, vz), t, wind=(wind_x, 0.0))
        self.assertAlmostEqual(x - target[0], 0.5 * wind_x * t * t, places=9)

    def test_boost_scales_horizontal_speed(self):
        muzzle = ctrl.muzzle_position(0.0)
        vx1, vy1, _, _ = ctrl.launch_velocity(muzzle, (2.0, 0.0), boost=1.0)
        vx2, vy2, _, _ = ctrl.launch_velocity(muzzle, (2.0, 0.0), boost=1.25)
        self.assertAlmostEqual(math.hypot(vx2, vy2) / math.hypot(vx1, vy1),
                               1.25, places=6)

    def test_distance_scales_speed(self):
        near = ctrl.launch_velocity(ctrl.muzzle_position(0.0), (1.0, 0.0))[0]
        far = ctrl.launch_velocity(ctrl.muzzle_position(0.0), (3.0, 0.0))[0]
        self.assertGreater(far, near)

    def test_zero_distance_returns_zeros(self):
        self.assertEqual(ctrl.launch_velocity((1.0, 1.0), (1.0, 1.0)),
                         (0.0, 0.0, 0.0, 0.0))


class ScoringTests(unittest.TestCase):
    def test_empty(self):
        s = score_shots([], 0, 0.0, "球用尽")
        self.assertEqual(s["shots"], 0)
        self.assertEqual(s["hit_rate"], 0.0)
        self.assertIsNone(s["avg_shots_per_down"])
        self.assertFalse(s["finished"])

    def test_perfect_run(self):
        shots = [{"target": i, "hit": True} for i in range(6)]
        s = score_shots(shots, 6, 9.5, "全部击倒")
        self.assertAlmostEqual(s["hit_rate"], 1.0)
        self.assertEqual(s["first_shot_hits"], 6)
        self.assertAlmostEqual(s["avg_shots_per_down"], 1.0)
        self.assertTrue(s["finished"])

    def test_retries(self):
        shots = [{"target": 0, "hit": False}, {"target": 0, "hit": True},
                 {"target": 1, "hit": True}]
        s = score_shots(shots, 2, 12.0, "球用尽")
        self.assertAlmostEqual(s["hit_rate"], 2 / 3.0)
        self.assertEqual(s["first_shot_hits"], 1)
        self.assertAlmostEqual(s["avg_shots_per_down"], 1.5)

    def test_single_miss_is_not_first_shot_hit(self):
        s = score_shots([{"target": 3, "hit": False}], 0, 3.0, "超时")
        self.assertEqual(s["first_shot_hits"], 0)

    def test_out_of_range_target_ignored(self):
        s = score_shots([{"target": 99, "hit": True}], 0, 1.0, "超时")
        self.assertEqual(s["shots"], 1)
        self.assertEqual(s["first_shot_hits"], 0)


class HelperTests(unittest.TestCase):
    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(0.3), 0.3, places=12)
        self.assertAlmostEqual(wrap_angle(2 * math.pi + 0.3), 0.3, places=9)
        self.assertAlmostEqual(abs(wrap_angle(3 * math.pi)), math.pi, places=9)


if __name__ == "__main__":
    unittest.main()
