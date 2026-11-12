# -*- coding: utf-8 -*-
"""阿克曼运动学与 Pure Pursuit 的单元测试（不依赖 ROS / Gazebo）。

    cd src/chap7/smartcar_control
    python3 -m unittest discover -s test
"""

from __future__ import annotations

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "scripts"))

from ackermann_kinematics import (  # noqa: E402
    MAX_STEER,
    TRACK,
    WHEELBASE,
    Path,
    PurePursuit,
    ackermann_angles,
    bicycle_step,
    circle_path,
    curvature,
    lane_change_path,
    min_turn_radius,
    simulate,
    steer_for_curvature,
    wheel_speeds,
    wrap_angle,
)


class TestAckermannGeometry(unittest.TestCase):
    def test_straight(self):
        left, right = ackermann_angles(0.0)
        self.assertAlmostEqual(left, 0.0, places=12)
        self.assertAlmostEqual(right, 0.0, places=12)
        v_left, v_right = wheel_speeds(0.4, 0.0)
        self.assertAlmostEqual(v_left, v_right, places=12)

    def test_left_turn_inner_wheel_steeper(self):
        """左转时左轮是内侧轮，转角要更大。"""
        left, right = ackermann_angles(0.2)
        self.assertGreater(left, 0.0)
        self.assertGreater(right, 0.0)
        self.assertGreater(left, right)

    def test_right_turn_is_mirror(self):
        left_l, right_l = ackermann_angles(0.2)
        left_r, right_r = ackermann_angles(-0.2)
        self.assertAlmostEqual(left_l, -right_r, places=12)
        self.assertAlmostEqual(right_l, -left_r, places=12)

    def test_matches_cpp_formula(self):
        """与 C++ 节点里的公式逐项对照。"""
        delta = 0.15
        t = math.tan(delta)
        want_left = math.atan(WHEELBASE * t / (WHEELBASE - 0.5 * TRACK * t))
        want_right = math.atan(WHEELBASE * t / (WHEELBASE + 0.5 * TRACK * t))
        left, right = ackermann_angles(delta)
        self.assertAlmostEqual(left, want_left, places=12)
        self.assertAlmostEqual(right, want_right, places=12)

    def test_inner_wheel_slower(self):
        v_left, v_right = wheel_speeds(0.4, 0.2)
        self.assertLess(v_left, v_right)
        self.assertAlmostEqual(v_left + v_right, 0.8, places=12)

    def test_speed_ratio_matches_radius_ratio(self):
        """内外轮速度比应等于转弯半径比。"""
        delta = 0.25
        v_left, v_right = wheel_speeds(0.5, delta)
        radius = WHEELBASE / math.tan(delta)
        expected = (radius - 0.5 * TRACK) / (radius + 0.5 * TRACK)
        self.assertAlmostEqual(v_left / v_right, expected, places=9)

    def test_extreme_steer_raises(self):
        with self.assertRaises(ValueError):
            ackermann_angles(math.radians(89.0))

    def test_bad_wheelbase(self):
        with self.assertRaises(ValueError):
            ackermann_angles(0.1, wheelbase=0.0)
        with self.assertRaises(ValueError):
            wheel_speeds(0.4, 0.1, wheelbase=-1.0)


class TestCurvature(unittest.TestCase):
    def test_curvature_definition(self):
        delta = 0.2
        self.assertAlmostEqual(curvature(delta), math.tan(delta) / WHEELBASE, places=12)

    def test_inverse(self):
        k = 1.5
        delta = steer_for_curvature(k)
        self.assertAlmostEqual(curvature(delta), k, places=9)

    def test_clamped(self):
        self.assertAlmostEqual(steer_for_curvature(100.0), MAX_STEER, places=12)
        self.assertAlmostEqual(steer_for_curvature(-100.0), -MAX_STEER, places=12)

    def test_min_turn_radius(self):
        r = min_turn_radius()
        self.assertAlmostEqual(r, WHEELBASE / math.tan(MAX_STEER), places=12)
        self.assertGreater(r, 0.0)

    def test_circle_radius_from_max_steer(self):
        """满舵匀速跑一圈，回代出来的半径应等于最小转弯半径。"""
        radius = min_turn_radius()
        speed = 0.3
        delta = MAX_STEER
        # 走一圈所需时间
        period = 2.0 * math.pi * radius / speed
        steps = int(round(period / 0.001))
        pose = (radius, 0.0, math.pi / 2)
        for _ in range(steps):
            pose = bicycle_step(pose, speed, delta, 0.001)
        self.assertLess(math.hypot(pose[0] - radius, pose[1]), 0.02)

    def test_bad_max_steer(self):
        with self.assertRaises(ValueError):
            min_turn_radius(max_steer=0.0)


class TestBicycle(unittest.TestCase):
    def test_straight_step(self):
        pose = bicycle_step((0.0, 0.0, 0.0), 0.5, 0.0, 0.1)
        self.assertAlmostEqual(pose[0], 0.05, places=12)
        self.assertAlmostEqual(pose[1], 0.0, places=12)
        self.assertAlmostEqual(pose[2], 0.0, places=12)

    def test_turn_changes_heading(self):
        pose = bicycle_step((0.0, 0.0, 0.0), 0.5, 0.2, 0.1)
        self.assertGreater(pose[2], 0.0)

    def test_arc_length(self):
        """走一小段圆弧，终点到圆心的距离应保持不变。"""
        radius = 0.5
        delta = math.atan(WHEELBASE / radius)
        pose = (radius, 0.0, math.pi / 2)
        for _ in range(500):
            pose = bicycle_step(pose, 0.2, delta, 0.002)
        self.assertAlmostEqual(math.hypot(pose[0], pose[1]), radius, places=3)

    def test_bad_dt(self):
        with self.assertRaises(ValueError):
            bicycle_step((0, 0, 0), 0.5, 0.0, 0.0)

    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(3 * math.pi), math.pi, places=9)
        self.assertAlmostEqual(wrap_angle(-3 * math.pi), math.pi, places=9)


class TestPath(unittest.TestCase):
    def test_length(self):
        path = Path([(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)])
        self.assertAlmostEqual(path.length, 7.0, places=12)

    def test_projection(self):
        path = Path([(0.0, 0.0), (10.0, 0.0)])
        s, cross = path.project((3.0, 1.0, 0.0))
        self.assertAlmostEqual(s, 3.0, places=9)
        self.assertAlmostEqual(cross, 1.0, places=9)     # 左侧为正
        _s, cross_right = path.project((3.0, -1.0, 0.0))
        self.assertAlmostEqual(cross_right, -1.0, places=9)

    def test_projection_clamped_to_segment(self):
        path = Path([(0.0, 0.0), (10.0, 0.0)])
        s, _cross = path.project((-5.0, 0.0, 0.0))
        self.assertAlmostEqual(s, 0.0, places=9)

    def test_point_at(self):
        path = Path([(0.0, 0.0), (10.0, 0.0)])
        self.assertAlmostEqual(path.point_at(4.0)[0], 4.0, places=9)
        self.assertAlmostEqual(path.point_at(-1.0)[0], 0.0, places=9)
        self.assertAlmostEqual(path.point_at(99.0)[0], 10.0, places=9)

    def test_requires_two_points(self):
        with self.assertRaises(ValueError):
            Path([(0.0, 0.0)])


class TestPurePursuit(unittest.TestCase):
    def straight(self):
        return Path([(0.0, 0.0), (10.0, 0.0)])

    def test_zero_steer_on_straight(self):
        ctrl = PurePursuit(self.straight(), lookahead=0.5)
        _v, steer, _t, _s = ctrl.command((2.0, 0.0, 0.0))
        self.assertAlmostEqual(steer, 0.0, places=9)

    def test_turn_towards_left_target(self):
        ctrl = PurePursuit(self.straight(), lookahead=0.5)
        _v, steer, _t, _s = ctrl.command((2.0, 0.3, 0.0))
        self.assertLess(steer, 0.0)      # 目标在左下方，需要右打方向

    def test_steer_is_clamped(self):
        ctrl = PurePursuit(self.straight(), lookahead=0.1)
        _v, steer, target, _s = ctrl.command((2.0, 5.0, 0.0))
        self.assertLessEqual(abs(steer), MAX_STEER + 1e-12)

    def test_bad_params(self):
        with self.assertRaises(ValueError):
            PurePursuit(self.straight(), lookahead=0.0)
        with self.assertRaises(ValueError):
            PurePursuit(self.straight(), speed=-1.0)


class TestSimulate(unittest.TestCase):
    def test_circle_tracking(self):
        path = circle_path(radius=1.0)
        result = simulate(path, start_pose=(1.0, 0.0, math.pi / 2),
                          lookahead=0.5, speed=0.4)
        self.assertTrue(result["finished"])
        self.assertLess(result["cross_track_rms"], 0.02)
        self.assertLess(result["cross_track_max"], 0.05)
        self.assertAlmostEqual(result["distance"], path.length, delta=0.3)

    def test_lane_change_tracking(self):
        path = lane_change_path(length=4.0, offset=0.6)
        result = simulate(path, lookahead=0.5, speed=0.4)
        self.assertTrue(result["finished"])
        self.assertLess(result["cross_track_rms"], 0.03)

    def test_lookahead_too_small_still_runs(self):
        path = lane_change_path(length=4.0, offset=0.6)
        result = simulate(path, lookahead=0.1, speed=0.4, max_time=80.0)
        self.assertGreater(result["cross_track_max"], 0.0)

    def test_bad_dt(self):
        with self.assertRaises(ValueError):
            simulate(circle_path(), dt=0.0)

    def test_short_run_has_no_metrics(self):
        """仿真短于起步段时指标必须是 None，不能伪装成 0（review 回归用例）。"""
        path = Path([(0.0, 0.0), (0.1, 0.0)])
        result = simulate(path, start_pose=(0.0, 5.0, 0.0),
                          lookahead=0.5, speed=0.4, max_time=5.0)
        self.assertLess(result["duration"], 1.0)
        self.assertIsNone(result["cross_track_rms"])
        self.assertIsNone(result["cross_track_max"])
        self.assertEqual(result["samples"], 0)

    def test_metrics_present_for_long_run(self):
        path = circle_path(radius=1.0)
        result = simulate(path, start_pose=(1.0, 0.0, math.pi / 2),
                          lookahead=0.5, speed=0.4)
        self.assertGreater(result["samples"], 0)
        self.assertIsNotNone(result["cross_track_rms"])

    def test_bad_wheelbase_raises_value_error(self):
        with self.assertRaises(ValueError):
            bicycle_step((0.0, 0.0, 0.0), 0.5, 0.0, 0.1, wheelbase=0.0)
        with self.assertRaises(ValueError):
            PurePursuit(Path([(0.0, 0.0), (1.0, 0.0)]), wheelbase=0.0)

    def test_trajectory_shape(self):
        path = circle_path(radius=1.0, samples=60)
        result = simulate(path, start_pose=(1.0, 0.0, math.pi / 2),
                          lookahead=0.5, speed=0.5, dt=0.02)
        self.assertGreater(len(result["trajectory"]), 10)
        first = result["trajectory"][0]
        self.assertEqual(len(first), 5)


class TestHelperPaths(unittest.TestCase):
    def test_circle_closed(self):
        path = circle_path(radius=2.0)
        self.assertAlmostEqual(path.points[0][0], path.points[-1][0], places=9)
        self.assertAlmostEqual(path.points[0][1], path.points[-1][1], places=9)

    def test_circle_rejects_bad_radius(self):
        with self.assertRaises(ValueError):
            circle_path(radius=0.0)

    def test_lane_change_monotonic_x(self):
        path = lane_change_path()
        xs = [p[0] for p in path.points]
        self.assertEqual(xs, sorted(xs))
        self.assertAlmostEqual(path.points[0][1], 0.0, places=12)
        self.assertAlmostEqual(path.points[-1][1], 0.6, places=12)


if __name__ == "__main__":
    unittest.main()
