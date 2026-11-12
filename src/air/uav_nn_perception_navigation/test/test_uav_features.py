#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_features.py 的单元测试：ROS 扫描角度约定与多帧坐标变换。"""

import math
import os
import sys
import unittest

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'scripts'))

import uav_features as uf  # noqa: E402


class TestScanConvention(unittest.TestCase):
    """AirSim 机体系 y 向右，ROS 扫描逆时针为正，必须取负。"""

    def test_right_point_goes_to_negative_angle(self):
        # 机身右侧 (y=+10)，±90° 视场：应落在 angle_min 端（索引 0）
        pts = np.array([[0.0, 10.0, 0.0]], dtype=np.float32)
        ranges = uf.extract_scan(pts, bins=180, fov_deg=180.0)
        self.assertAlmostEqual(float(ranges[0]), 10.0, places=4)
        self.assertAlmostEqual(float(ranges[-1]), uf.MAX_RANGE, places=4)

    def test_left_point_goes_to_positive_angle(self):
        pts = np.array([[0.0, -10.0, 0.0]], dtype=np.float32)
        ranges = uf.extract_scan(pts, bins=180, fov_deg=180.0)
        self.assertAlmostEqual(float(ranges[-1]), 10.0, places=4)
        self.assertAlmostEqual(float(ranges[0]), uf.MAX_RANGE, places=4)

    def test_front_point_stays_in_the_middle(self):
        pts = np.array([[10.0, 0.0, 0.0]], dtype=np.float32)
        ranges = uf.extract_scan(pts, bins=360, fov_deg=360.0)
        idx = int(np.argmin(ranges))
        self.assertLessEqual(abs(idx - 180), 1)

    def test_out_of_range_point_is_ignored(self):
        pts = np.array([[0.0, 10.0, 0.0]], dtype=np.float32)
        ranges = uf.extract_scan(pts, bins=180, fov_deg=180.0,
                                 max_range=5.0)
        self.assertTrue(np.allclose(ranges, 5.0))


class TestFrameTransform(unittest.TestCase):
    def test_rotation_moves_forward_point_to_left(self):
        # 目标机体系相对源转左 90°（NED 下 yaw 逆时针），正前方 5 m
        # 的障碍在新的机体系里应该在左侧 (y=-5)
        pts = np.array([[5.0, 0.0, 0.0]], dtype=np.float32)
        out = uf.transform_points_to_frame(
            pts, (0.0, 0.0, 0.0), 0.0, (0.0, 0.0, 0.0), math.pi / 2.0)
        self.assertAlmostEqual(float(out[0, 0]), 0.0, places=4)
        self.assertAlmostEqual(float(out[0, 1]), -5.0, places=4)

    def test_translation_shifts_point_backwards(self):
        pts = np.array([[5.0, 0.0, 0.0]], dtype=np.float32)
        out = uf.transform_points_to_frame(
            pts, (0.0, 0.0, 0.0), 0.0, (10.0, 0.0, 0.0), 0.0)
        self.assertAlmostEqual(float(out[0, 0]), -5.0, places=4)
        self.assertAlmostEqual(float(out[0, 1]), 0.0, places=4)

    def test_round_trip_returns_original(self):
        rng = np.random.RandomState(0)
        pts = rng.uniform(-10, 10, size=(20, 3)).astype(np.float32)
        src_pos, src_yaw = (2.0, -3.0, -4.0), 0.7
        dst_pos, dst_yaw = (-1.0, 5.0, -4.5), -1.2

        moved = uf.transform_points_to_frame(pts, src_pos, src_yaw,
                                             dst_pos, dst_yaw)
        back = uf.transform_points_to_frame(moved, dst_pos, dst_yaw,
                                            src_pos, src_yaw)
        self.assertTrue(np.allclose(back, pts, atol=1e-4))

    def test_same_pose_is_identity(self):
        pts = np.array([[1.0, 2.0, -3.0]], dtype=np.float32)
        out = uf.transform_points_to_frame(pts, (1, 1, 1), 0.3,
                                           (1, 1, 1), 0.3)
        self.assertTrue(np.allclose(out, pts, atol=1e-5))


if __name__ == '__main__':
    unittest.main()
