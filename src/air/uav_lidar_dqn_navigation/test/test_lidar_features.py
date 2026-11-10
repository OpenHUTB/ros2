#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lidar_features 的单元测试（不依赖 AirSim，可直接用 python3 运行）。"""

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'scripts'))

from lidar_features import (  # noqa: E402
    FOV_DEG, MAX_RANGE, NUM_SECTORS, STATE_DIM, build_state, extract_scan,
    goal_state, min_obstacle_distance, scan_angle_max, sector_distances,
    velocity_state,
)


class TestSectorDistances(unittest.TestCase):
    def test_empty_cloud_is_all_clear(self):
        feats = sector_distances(np.zeros((0, 3), dtype=np.float32))
        self.assertEqual(feats.shape, (NUM_SECTORS,))
        self.assertTrue(np.allclose(feats, 1.0))

    def test_forward_obstacle_hits_middle_sector(self):
        # 正前方 10 m 一个点 -> 正中扇区约 0.25，其余为 1
        pts = np.array([[10.0, 0.0, 0.0]], dtype=np.float32)
        feats = sector_distances(pts)
        mid = NUM_SECTORS // 2
        self.assertAlmostEqual(float(feats[mid]), 10.0 / MAX_RANGE, places=4)
        self.assertEqual(float(np.sum(feats < 1.0)), 1.0)

    def test_points_outside_fov_ignored(self):
        # 正后方（180°）不在 ±90° 视场内
        pts = np.array([[-10.0, 0.0, 0.0]], dtype=np.float32)
        feats = sector_distances(pts)
        self.assertTrue(np.allclose(feats, 1.0))

    def test_nearest_point_wins(self):
        pts = np.array([[8.0, 0.0, 0.0], [3.0, 0.0, 0.0]], dtype=np.float32)
        feats = sector_distances(pts)
        self.assertAlmostEqual(float(feats[NUM_SECTORS // 2]),
                               3.0 / MAX_RANGE, places=4)


class TestGoalAndVelocity(unittest.TestCase):
    def test_goal_directly_ahead(self):
        # 机头朝北（yaw=0），目标在正北 20 m
        g = goal_state((0, 0, -4), (20, 0, -4), 0.0)
        self.assertAlmostEqual(float(g[0]), 0.5, places=4)
        self.assertAlmostEqual(float(g[1]), 0.0, places=4)   # sin(0)
        self.assertAlmostEqual(float(g[2]), 1.0, places=4)   # cos(0)

    def test_goal_behind(self):
        g = goal_state((0, 0, -4), (-20, 0, -4), 0.0)
        self.assertAlmostEqual(float(g[2]), -1.0, places=4)

    def test_velocity_rotation(self):
        # 机头朝北（yaw=0）向北飞 -> 机体系前向分量拉满
        v = velocity_state((10.0, 0.0, 0.0), 0.0)
        self.assertGreater(float(v[0]), 0.99)
        # 机头朝东（NED 下 yaw=pi/2）向东飞 -> 同样应落在机体系前向
        v2 = velocity_state((0.0, 10.0, 0.0), math.pi / 2.0)
        self.assertGreater(float(v2[0]), 0.99)


class TestScanAndState(unittest.TestCase):
    def test_scan_length_and_range(self):
        pts = np.array([[5.0, 0.0, 0.0]], dtype=np.float32)
        ranges = extract_scan(pts, bins=180)
        self.assertEqual(ranges.shape, (180,))
        self.assertAlmostEqual(float(ranges[90]), 5.0, places=3)
        self.assertAlmostEqual(float(ranges[0]), MAX_RANGE, places=3)

    def test_state_dimension(self):
        pts = np.zeros((0, 3), dtype=np.float32)
        s = build_state(pts, (0, 0, -4), (10, 10, -4), 0.3, (1, 0, 0), 2)
        self.assertEqual(s.shape, (STATE_DIM,))
        self.assertEqual(STATE_DIM, 32)

    def test_min_obstacle(self):
        pts = np.array([[2.5, 0.0, 0.0]], dtype=np.float32)
        self.assertAlmostEqual(min_obstacle_distance(pts), 2.5, places=4)

    def test_scan_angle_max_uses_bins_minus_one(self):
        # 180 个 1° 的采样点覆盖 [-90°, +89°]，而不是 [-90°, +90°]
        angle_min = math.radians(-FOV_DEG / 2.0)
        angle_increment = math.radians(FOV_DEG / 180.0)
        angle_max = scan_angle_max(angle_min, angle_increment, 180)
        self.assertAlmostEqual(math.degrees(angle_max), 89.0, places=6)
        self.assertAlmostEqual(
            math.degrees(angle_min + angle_increment * 180), 90.0, places=6)
        points = int(round((angle_max - angle_min) / angle_increment)) + 1
        self.assertEqual(points, 180)


if __name__ == '__main__':
    unittest.main(verbosity=2)
