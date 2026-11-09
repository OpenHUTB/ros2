#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_slam_nav 的本地单元测试，不需要 ROS。

跑法：
    python3 src/air/uav_slam_nav/tests/test_uav_slam_nav_local.py

覆盖两块：
1. nav_logic.py 里的纯函数（点云投影、扇区测距、兜底决策、速度仲裁）
2. 静态回归：simple_nav 不许直接发 /uav/cmd_vel，必须订阅 /move_base/status
"""
import importlib.util
import math
import os
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SCRIPTS = os.path.join(PKG, "scripts")

DEG = math.pi / 180.0


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


nl = _load("nav_logic", os.path.join(SCRIPTS, "nav_logic.py"))


def _identity_quat():
    return (0.0, 0.0, 0.0, 1.0)


def _scan_ranges(n_bins, angle_min, angle_inc, value):
    return [value] * n_bins


class TestClamp(unittest.TestCase):
    def test_inside(self):
        self.assertEqual(nl.clamp(0.3, -1.0, 1.0), 0.3)

    def test_upper_and_lower(self):
        self.assertEqual(nl.clamp(5.0, -1.0, 1.0), 1.0)
        self.assertEqual(nl.clamp(-5.0, -1.0, 1.0), -1.0)

    def test_bad_range(self):
        with self.assertRaises(ValueError):
            nl.clamp(0.0, 1.0, -1.0)


class TestStatus(unittest.TestCase):
    def test_failed(self):
        for code in (4, 5, 8, 9):          # ABORTED / REJECTED / RECALLED / LOST
            self.assertTrue(nl.status_is_failed(code), code)

    def test_healthy(self):
        for code in (0, 1, 2, 3, 6, 7):    # PENDING / ACTIVE / PREEMPTED / SUCCEEDED ...
            self.assertFalse(nl.status_is_failed(code), code)


class TestProject(unittest.TestCase):
    def setUp(self):
        self.args = dict(angle_min=-math.pi, angle_max=math.pi, angle_inc=1.0 * DEG,
                         range_min=0.3, range_max=30.0, height_lo=-0.3, height_hi=3.0)
        self.n_bins = 361

    def _index_of(self, angle):
        return int(np.clip(math.floor((angle + math.pi) / (1.0 * DEG)), 0, self.n_bins - 1))

    def test_single_point_on_x_axis(self):
        pts = np.array([[2.0, 0.0, 0.5]])
        r = nl.project(pts, np.zeros(3), _identity_quat(), **self.args)
        self.assertEqual(r.shape[0], self.n_bins)
        self.assertAlmostEqual(r[self._index_of(0.0)], 2.0, places=6)
        self.assertAlmostEqual(r[0], 30.0, places=6)

    def test_ground_is_sliced_out(self):
        pts = np.array([[2.0, 0.0, -1.5]])          # 低于 height_lo，属于地面
        r = nl.project(pts, np.zeros(3), _identity_quat(), **self.args)
        self.assertEqual(r.size, 0)

    def test_tall_structure_is_sliced_out(self):
        pts = np.array([[2.0, 0.0, 6.0]])           # 高于 height_hi
        r = nl.project(pts, np.zeros(3), _identity_quat(), **self.args)
        self.assertEqual(r.size, 0)

    def test_nearest_point_wins_in_same_bin(self):
        pts = np.array([[5.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        r = nl.project(pts, np.zeros(3), _identity_quat(), **self.args)
        self.assertAlmostEqual(r[self._index_of(0.0)], 2.0, places=6)

    def test_below_range_min_dropped(self):
        pts = np.array([[0.1, 0.0, 0.0]])           # 0.1 < range_min 0.3
        r = nl.project(pts, np.zeros(3), _identity_quat(), **self.args)
        self.assertEqual(r.size, 0)

    def test_beyond_range_max_dropped(self):
        pts = np.array([[40.0, 0.0, 0.0]])
        r = nl.project(pts, np.zeros(3), _identity_quat(), **self.args)
        self.assertEqual(r.size, 0)

    def test_yaw_rotation_is_applied(self):
        # 机体逆时针转 90 度时，世界系 +y 方向的点应落到机头正前方
        s = math.sin(math.pi / 4.0)
        c = math.cos(math.pi / 4.0)
        pts = np.array([[0.0, 2.0, 0.0]])
        r = nl.project(pts, np.zeros(3), (0.0, 0.0, s, c), **self.args)
        self.assertAlmostEqual(r[self._index_of(0.0)], 2.0, places=6)

    def test_translation_is_subtracted(self):
        pts = np.array([[3.0, 0.0, 0.0]])
        r = nl.project(pts, np.array([1.0, 0.0, 0.0]), _identity_quat(), **self.args)
        self.assertAlmostEqual(r[self._index_of(0.0)], 2.0, places=6)

    def test_empty_cloud(self):
        r = nl.project(np.zeros((0, 3)), np.zeros(3), _identity_quat(), **self.args)
        self.assertEqual(r.size, 0)

    def test_bad_angle_increment(self):
        bad = dict(self.args)
        bad["angle_inc"] = 0.0
        with self.assertRaises(ValueError):
            nl.project(np.array([[1.0, 0.0, 0.0]]), np.zeros(3), _identity_quat(), **bad)


class TestSectorRanges(unittest.TestCase):
    def setUp(self):
        self.angle_min = -math.pi
        self.inc = 1.0 * DEG
        self.n = 361

    def test_front_min_picks_closest(self):
        ranges = _scan_ranges(self.n, self.angle_min, self.inc, 10.0)
        ranges[180] = 1.2     # 正前方
        ranges[170] = 2.5
        self.assertAlmostEqual(nl.front_min_range(ranges, self.angle_min, self.inc, 30 * DEG), 1.2)

    def test_front_ignores_points_outside_sector(self):
        ranges = _scan_ranges(self.n, self.angle_min, self.inc, 10.0)
        ranges[100] = 0.5     # 约 -80 度，在前向扇区之外
        self.assertAlmostEqual(nl.front_min_range(ranges, self.angle_min, self.inc, 30 * DEG), 10.0)

    def test_none_when_all_invalid(self):
        ranges = [float("inf")] * self.n
        self.assertIsNone(nl.front_min_range(ranges, self.angle_min, self.inc, 30 * DEG))

    def test_side_sectors(self):
        ranges = _scan_ranges(self.n, self.angle_min, self.inc, 10.0)
        ranges[61] = 1.5      # 约 -119 度，属于右侧扇区 (-120, -60)
        ranges[299] = 2.0     # 约 +119 度，属于左侧扇区 (60, 120)
        left = nl.sector_min_range(ranges, self.angle_min, self.inc, 60 * DEG, 120 * DEG)
        right = nl.sector_min_range(ranges, self.angle_min, self.inc, -120 * DEG, -60 * DEG)
        self.assertAlmostEqual(left, 2.0)
        self.assertAlmostEqual(right, 1.5)


class TestTurnSign(unittest.TestCase):
    def test_turn_towards_clearer_side(self):
        self.assertEqual(nl.choose_turn_sign(0.5, 2.0), -1)   # 右边更空 -> 右转
        self.assertEqual(nl.choose_turn_sign(2.0, 0.5), 1)    # 左边更空 -> 左转

    def test_default_left_when_unknown(self):
        self.assertEqual(nl.choose_turn_sign(None, None), 1)
        self.assertEqual(nl.choose_turn_sign(None, 1.0), 1)


class TestDecideCommand(unittest.TestCase):
    def test_obstacle_stop_and_turn_away(self):
        vx, wz = nl.decide_command(0.4, 0.6, 1.5, 0.0)
        self.assertEqual(vx, 0.0)
        self.assertLess(wz, 0.0)          # 右边空 -> 右转（角速度为负）

    def test_clear_path_goes_forward(self):
        vx, wz = nl.decide_command(5.0, 5.0, 5.0, 0.0)
        self.assertAlmostEqual(vx, nl.DEFAULT_FALLBACK_CFG["max_vx"])
        self.assertAlmostEqual(wz, 0.0)

    def test_no_goal_holds_still(self):
        vx, wz = nl.decide_command(5.0, 5.0, 5.0, None)
        self.assertEqual((vx, wz), (0.0, 0.0))

    def test_big_bearing_turns_in_place(self):
        vx, wz = nl.decide_command(5.0, 5.0, 5.0, math.pi / 2.0)
        self.assertEqual(vx, 0.0)
        self.assertAlmostEqual(wz, 0.6)

    def test_small_bearing_slows_down(self):
        vx, wz = nl.decide_command(5.0, 5.0, 5.0, 0.3)
        self.assertAlmostEqual(vx, 0.5 * (1 - 0.3 / 1.2), places=6)
        self.assertAlmostEqual(wz, 0.3, places=6)

    def test_angle_is_wrapped(self):
        _, wz_full = nl.decide_command(5.0, 5.0, 5.0, 2.0 * math.pi)
        self.assertAlmostEqual(wz_full, 0.0, places=6)


class TestLimitTwist(unittest.TestCase):
    def test_clamped_and_z_zeroed(self):
        vx, vy, vz, wz = nl.limit_twist(3.0, -2.0, 1.5, 0.8, 0.5, 0.6)
        self.assertAlmostEqual(vx, 0.8)
        self.assertAlmostEqual(vy, -0.5)
        self.assertEqual(vz, 0.0)
        self.assertAlmostEqual(wz, 0.6)

    def test_wz_clamped(self):
        _, _, _, wz = nl.limit_twist(0.2, 0.2, 0.3, 0.8, 0.5, 0.6)
        self.assertAlmostEqual(wz, 0.3)


class TestSelectSource(unittest.TestCase):
    def test_move_base_only(self):
        self.assertEqual(nl.select_source(10.0, 9.8, None, 0.5), "move_base")

    def test_fallback_wins(self):
        self.assertEqual(nl.select_source(10.0, 9.85, 9.9, 0.5), "fallback")

    def test_fallback_stale_falls_back_to_move_base(self):
        self.assertEqual(nl.select_source(10.0, 9.8, 9.0, 0.5), "move_base")

    def test_all_stale_gives_none(self):
        self.assertIsNone(nl.select_source(10.0, 9.0, 8.0, 0.5))

    def test_no_sources(self):
        self.assertIsNone(nl.select_source(10.0, None, None, 0.5))


class TestStaticRegression(unittest.TestCase):
    """老师 review 指出的问题，用静态断言钉住，防止以后又改回去。"""

    def _read(self, name):
        with open(os.path.join(SCRIPTS, name), "r", encoding="utf-8") as f:
            return f.read()

    def test_simple_nav_does_not_publish_uav_cmd_vel(self):
        src = self._read("simple_nav.py")
        self.assertNotIn('Publisher("/uav/cmd_vel"', src)

    def test_simple_nav_publishes_dedicated_topic(self):
        src = self._read("simple_nav.py")
        self.assertIn("/simple_nav/cmd_vel", src)

    def test_simple_nav_subscribes_move_base_status(self):
        src = self._read("simple_nav.py")
        self.assertIn("/move_base/status", src)

    def test_simple_nav_reads_goal(self):
        src = self._read("simple_nav.py")
        self.assertIn("/move_base/goal", src)

    def test_nav_relay_is_single_writer(self):
        src = self._read("nav_relay.py")
        self.assertIn('Publisher("/uav/cmd_vel"', src)
        # 两个输入都进仲裁
        self.assertIn('"/cmd_vel"', src)
        self.assertIn('"/simple_nav/cmd_vel"', src)

    def test_launch_starts_fallback_node(self):
        launch = os.path.join(PKG, "launch", "slam_nav.launch")
        with open(launch, "r", encoding="utf-8") as f:
            text = f.read()
        self.assertIn('type="simple_nav.py"', text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
