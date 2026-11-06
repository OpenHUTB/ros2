# -*- coding: utf-8 -*-
"""打靶弹道解算与成绩统计的单元测试。

只测纯数学部分（不 import mujoco），所以在任何装了 Python3 的机器上都能跑：

    cd src/ground/mujoco_shooting
    python3 -m unittest discover -s test
"""

from __future__ import annotations

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ballistics import (GRAVITY, predict_position, required_speed,  # noqa: E402
                        score_shots, solve_launch_velocity, wrap_angle)


class TestSolveLaunch(unittest.TestCase):
    START = (0.63, 0.0, 0.17)
    TARGET = (3.40, 0.0, 0.25)

    def test_no_wind_hits_target_exactly(self):
        v = solve_launch_velocity(self.START, self.TARGET, 0.45)
        p = predict_position(self.START, v, 0.45)
        for got, want in zip(p, self.TARGET):
            self.assertAlmostEqual(got, want, places=12)

    def test_wind_is_compensated(self):
        wind = (3.0, -2.0)
        v = solve_launch_velocity(self.START, self.TARGET, 0.45, wind=wind)
        p = predict_position(self.START, v, 0.45, wind=wind)
        for got, want in zip(p, self.TARGET):
            self.assertAlmostEqual(got, want, places=12)

    def test_wind_reduces_forward_speed(self):
        """顺风（+x）时初速度要更小，否则会打过头。"""
        v0 = solve_launch_velocity(self.START, self.TARGET, 0.45)
        v1 = solve_launch_velocity(self.START, self.TARGET, 0.45, wind=(3.0, 0.0))
        self.assertLess(v1[0], v0[0])
        self.assertAlmostEqual(v0[0] - v1[0], 0.5 * 3.0 * 0.45, places=12)

    def test_vertical_component_compensates_gravity(self):
        v = solve_launch_velocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0)
        self.assertAlmostEqual(v[2], 0.5 * GRAVITY, places=12)

    def test_short_flight_time_needs_higher_speed(self):
        slow = solve_launch_velocity(self.START, self.TARGET, 0.5)
        fast = solve_launch_velocity(self.START, self.TARGET, 0.3)
        self.assertGreater(math.hypot(fast[0], fast[2]),
                           math.hypot(slow[0], slow[2]))

    def test_invalid_flight_time(self):
        for bad in (0.0, -1.0):
            with self.assertRaises(ValueError):
                solve_launch_velocity(self.START, self.TARGET, bad)

    def test_invalid_dimensions(self):
        with self.assertRaises(ValueError):
            solve_launch_velocity((0.0, 0.0), self.TARGET, 0.4)
        with self.assertRaises(ValueError):
            solve_launch_velocity(self.START, (0.0, 0.0), 0.4)


class TestPredict(unittest.TestCase):
    def test_gravity_drop(self):
        p = predict_position((0, 0, 10), (0, 0, 0), 1.0)
        self.assertAlmostEqual(p[2], 10 - 0.5 * GRAVITY, places=12)

    def test_zero_time(self):
        p = predict_position((1, 2, 3), (5, 6, 7), 0.0)
        self.assertEqual(p, (1, 2, 3))

    def test_wind_offset_is_quadratic(self):
        a = (2.0, 0.0)
        p1 = predict_position((0, 0, 0), (0, 0, 0), 1.0, wind=a)
        p2 = predict_position((0, 0, 0), (0, 0, 0), 2.0, wind=a)
        self.assertAlmostEqual(p2[0], 4.0 * p1[0], places=12)


class TestHelpers(unittest.TestCase):
    def test_required_speed(self):
        self.assertAlmostEqual(required_speed(2.7, 0.45), 6.0, places=12)
        with self.assertRaises(ValueError):
            required_speed(1.0, 0.0)

    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(0.3), 0.3, places=12)
        self.assertAlmostEqual(wrap_angle(2 * math.pi + 0.3), 0.3, places=9)
        self.assertAlmostEqual(wrap_angle(-2 * math.pi - 0.3), -0.3, places=9)
        self.assertAlmostEqual(abs(wrap_angle(3 * math.pi)), math.pi, places=9)


class TestScoreShots(unittest.TestCase):
    def test_empty(self):
        s = score_shots([], 0, 0.0, "球用尽")
        self.assertEqual(s["shots"], 0)
        self.assertEqual(s["hit_rate"], 0.0)
        self.assertIsNone(s["avg_shots_per_down"])
        self.assertFalse(s["finished"])

    def test_perfect_run(self):
        shots = [{"target": i, "hit": True} for i in range(6)]
        s = score_shots(shots, 6, 9.5, "全部击倒")
        self.assertEqual(s["shots"], 6)
        self.assertEqual(s["downed"], 6)
        self.assertAlmostEqual(s["hit_rate"], 1.0)
        self.assertEqual(s["first_shot_hits"], 6)
        self.assertAlmostEqual(s["avg_shots_per_down"], 1.0)
        self.assertTrue(s["finished"])

    def test_retries(self):
        shots = [
            {"target": 0, "hit": False},
            {"target": 0, "hit": True},
            {"target": 1, "hit": True},
            {"target": 2, "hit": False},
        ]
        s = score_shots(shots, 2, 12.0, "球用尽")
        self.assertEqual(s["shots"], 4)
        self.assertEqual(s["downed"], 2)
        self.assertAlmostEqual(s["hit_rate"], 0.5)
        # 靶 1 只打了一发且命中
        self.assertEqual(s["first_shot_hits"], 1)
        self.assertAlmostEqual(s["avg_shots_per_down"], 2.0)

    def test_single_miss_not_counted_as_first_shot_hit(self):
        shots = [{"target": 3, "hit": False}]
        s = score_shots(shots, 0, 3.0, "超时")
        self.assertEqual(s["first_shot_hits"], 0)

    def test_out_of_range_target_ignored(self):
        shots = [{"target": 99, "hit": True}]
        s = score_shots(shots, 0, 1.0, "超时")
        self.assertEqual(s["shots"], 1)
        self.assertEqual(s["first_shot_hits"], 0)


if __name__ == "__main__":
    unittest.main()
