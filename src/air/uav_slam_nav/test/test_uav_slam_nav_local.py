#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_slam_nav 纯逻辑单元测试。不依赖 ROS，CI 直接 python3 跑。"""
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, PKG)

from uav_slam_nav import nav_logic as L  # noqa: E402


def identity_T():
    return np.eye(4)


class TestClamp(unittest.TestCase):
    def test_basic(self):
        self.assertEqual(L.clamp(5, 0, 3), 3)
        self.assertEqual(L.clamp(-1, 0, 3), 0)
        self.assertEqual(L.clamp(2, 0, 3), 2)

    def test_bad_range(self):
        with self.assertRaises(ValueError):
            L.clamp(1, 3, 0)


class TestNormalize(unittest.TestCase):
    def test_wrap(self):
        self.assertAlmostEqual(L.normalize_angle(3 * math.pi), math.pi)
        self.assertAlmostEqual(L.normalize_angle(-3 * math.pi), -math.pi)
        self.assertAlmostEqual(L.normalize_angle(0.5), 0.5)


class TestProject(unittest.TestCase):
    def _scan_cfg(self):
        return dict(angle_min=-math.pi, angle_max=math.pi, angle_inc=math.radians(1.0),
                    range_min=0.3, range_max=30.0, height_lo=-0.3, height_hi=3.0)

    def test_empty(self):
        self.assertEqual(L.project_points_to_scan(np.zeros((0, 3)), identity_T(), **self._scan_cfg()).size, 0)

    def test_single_point_front(self):
        # 正前方 2m，高度 0
        pts = np.array([[2.0, 0.0, 0.0]])
        r = L.project_points_to_scan(pts, identity_T(), **self._scan_cfg())
        # angle=0 对应的 bin
        idx = int(round((0.0 - (-math.pi)) / math.radians(1.0)))
        self.assertAlmostEqual(r[idx], 2.0, places=3)

    def test_height_filter(self):
        pts = np.array([[2.0, 0.0, 5.0]])  # 太高
        r = L.project_points_to_scan(pts, identity_T(), **self._scan_cfg())
        self.assertEqual(r.size, 0)

    def test_out_of_range(self):
        pts = np.array([[100.0, 0.0, 0.0]])
        r = L.project_points_to_scan(pts, identity_T(), **self._scan_cfg())
        self.assertEqual(r.size, 0)

    def test_nearest_in_bin(self):
        pts = np.array([[2.0, 0.0, 0.0], [5.0, 0.05, 0.0]])
        r = L.project_points_to_scan(pts, identity_T(), **self._scan_cfg())
        idx = int(round((0.0 - (-math.pi)) / math.radians(1.0)))
        self.assertAlmostEqual(r[idx], 2.0, places=3)

    def test_rotation_about_yaw(self):
        # 转 90 度，原来正前方的点跑到正右方
        q = (0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        R = L.quat_to_matrix(q)
        T = np.eye(4)
        T[:3, :3] = R
        pts = np.array([[2.0, 0.0, 0.0]])  # world 系正前方
        r = L.project_points_to_scan(pts, T, **self._scan_cfg())
        # base 系下点在 (0, +2, 0)，即 angle=+pi/2
        idx = int(round((math.pi / 2 - (-math.pi)) / math.radians(1.0)))
        self.assertAlmostEqual(r[idx], 2.0, places=3)


class TestSector(unittest.TestCase):
    def setUp(self):
        self.amin = -math.pi
        self.inc = math.radians(1.0)
        n = int(round((2 * math.pi) / self.inc)) + 1
        self.ranges = np.full(n, 30.0)

    def test_front_clear(self):
        self.ranges[int(round((0.0 - self.amin) / self.inc))] = 1.5
        m = L.front_min_range(self.ranges, self.amin, self.inc, math.radians(60))
        self.assertAlmostEqual(m, 1.5, places=3)

    def test_front_blocked(self):
        self.ranges[int(round((0.0 - self.amin) / self.inc))] = 0.5
        m = L.front_min_range(self.ranges, self.amin, self.inc, math.radians(60))
        self.assertAlmostEqual(m, 0.5, places=3)

    def test_sides(self):
        # 正左方 90 度放一个 1m 的障碍，其余 bin 全是 range_max=30m（无回波）
        self.ranges[int(round((math.pi / 2 - self.amin) / self.inc))] = 1.0
        left, right = L.side_min_range(self.ranges, self.amin, self.inc,
                                       math.radians(60), range_max=30.0)
        self.assertAlmostEqual(left, 1.0, places=3)
        self.assertIsNone(right)


class TestDecide(unittest.TestCase):
    def test_blocked_front_turns_to_clear_side(self):
        # 前方 0.5m 障碍，右边空
        vx, wz = L.decide_command(0.5, 3.0, None, goal_bearing=0.0)
        self.assertEqual(vx, 0.0)
        self.assertNotEqual(wz, 0.0)

    def test_no_goal_stopped(self):
        vx, wz = L.decide_command(None, None, None, goal_bearing=None)
        self.assertEqual((vx, wz), (0.0, 0.0))

    def test_align_only_when_big_error(self):
        vx, wz = L.decide_command(None, None, None, goal_bearing=math.pi / 2)
        self.assertAlmostEqual(vx, 0.0, places=3)  # 误差 > align_slow，只转不走
        self.assertNotEqual(wz, 0.0)

    def test_move_forward_when_aligned(self):
        vx, wz = L.decide_command(None, None, None, goal_bearing=0.05)
        self.assertGreater(vx, 0.0)


class TestLimitAndSelect(unittest.TestCase):
    def test_limit(self):
        vx, vy, vz, wz = L.limit_twist(99, 99, 99, 0.8, 0.5, 0.6)
        self.assertEqual(vx, 0.8)
        self.assertEqual(vy, 0.5)
        self.assertEqual(vz, 0.0)
        self.assertEqual(wz, 0.6)

    def test_select_fallback_priority(self):
        self.assertEqual(L.select_source(10.0, 9.9, 9.8, 0.5), "fallback")
        self.assertEqual(L.select_source(10.0, 9.9, None, 0.5), "nav2")
        self.assertIsNone(L.select_source(10.0, 9.0, None, 0.5))


class TestTransformMsg(unittest.TestCase):
    def test_matrix(self):
        class Q: x=y=z=0.0; w=1.0
        class T:
            x=1.0; y=2.0; z=3.0
        class TF:
            rotation=Q(); translation=T()
        M = L.transform_stamped_to_matrix(TF())
        np.testing.assert_allclose(M[:3, :3], np.eye(3))
        np.testing.assert_allclose(M[:3, 3], [1.0, 2.0, 3.0])


class TestStaticSourceAssertions(unittest.TestCase):
    """钉死上一轮 review 的修复：节点不直接发 /uav/cmd_vel，必须走 tf2。"""
    def _read(self, name):
        path = os.path.join(PKG, "uav_slam_nav", name)
        with open(path, encoding="utf-8") as f:
            return f.read()

    def test_simple_nav_does_not_publish_uav_cmd_vel(self):
        src = self._read("simple_nav_node.py")
        self.assertNotIn('create_publisher(Twist, "/uav/cmd_vel"', src)
        self.assertIn('create_publisher(Twist, "/simple_nav/cmd_vel"', src)

    def test_nav_relay_is_uav_cmd_vel_publisher(self):
        src = self._read("nav_relay_node.py")
        self.assertIn('create_publisher(Twist, "/uav/cmd_vel"', src)

    def test_points_to_scan_uses_tf2(self):
        src = self._read("points_to_scan_node.py")
        self.assertIn("tf2_ros", src)
        self.assertIn("lookup_transform", src)


if __name__ == "__main__":
    unittest.main(verbosity=2)
