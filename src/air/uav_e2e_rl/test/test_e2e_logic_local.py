#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_e2e_rl 纯逻辑单测，不依赖 ROS / cv2。"""
import math
import os
import sys
import struct
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, PKG)

from uav_e2e_rl import e2e_logic as L  # noqa: E402


def make_rgb(w, h, rgb=(128, 64, 32)):
    return bytes(np.tile(np.array(rgb, dtype=np.uint8), (h, w, 1)).tobytes())


class TestPreprocess(unittest.TestCase):
    def test_mono8(self):
        data = bytes(np.full((8, 16), 200, dtype=np.uint8).tobytes())
        out = L.preprocess_image("mono8", data, 16, 8, 16, 8)
        self.assertEqual(out.shape, (8, 16))
        self.assertAlmostEqual(float(out.mean()), 200 / 255, places=2)

    def test_rgb8_to_gray(self):
        out = L.preprocess_image("rgb8", make_rgb(16, 8, (30, 60, 90)), 16, 8, 16, 8)
        self.assertEqual(out.shape, (8, 16))
        self.assertAlmostEqual(float(out.mean()), (30 + 60 + 90) / 3 / 255, places=2)

    def test_resize(self):
        data = bytes(np.full((48, 64), 128, dtype=np.uint8).tobytes())
        out = L.preprocess_image("mono8", data, 64, 48, 16, 8)
        self.assertEqual(out.shape, (8, 16))

    def test_unknown_encoding(self):
        with self.assertRaises(ValueError):
            L.preprocess_image("yuv422", b"", 4, 4, 4, 4)


class TestDepthCollision(unittest.TestCase):
    def test_no_obstacle(self):
        depth = np.full((48, 64), 5.0, dtype=np.float32)
        self.assertFalse(L.is_collision(depth, 0.6))

    def test_obstacle_in_front(self):
        depth = np.full((48, 64), 5.0, dtype=np.float32)
        depth[30:40, 25:40] = 0.3
        self.assertTrue(L.is_collision(depth, 0.6))

    def test_invalid_ignored(self):
        depth = np.full((48, 64), -1.0, dtype=np.float32)
        self.assertEqual(L.min_depth_in_forward_sector(depth), float("inf"))


class TestReward(unittest.TestCase):
    def test_forward_distance(self):
        r = L.compute_reward(forward_dx=1.0, wz=0.0, collided=False)
        self.assertAlmostEqual(r, 0.5, places=3)

    def test_collision_short_circuits(self):
        # 撞车那一步不再叠转向惩罚
        r = L.compute_reward(forward_dx=1.0, wz=1.0, collided=True)
        self.assertAlmostEqual(r, -5.0, places=3)

    def test_yaw_penalty(self):
        r = L.compute_reward(forward_dx=0.0, wz=0.5, collided=False)
        self.assertAlmostEqual(r, -0.05, places=3)


class TestClip(unittest.TestCase):
    def test_clip(self):
        self.assertEqual(L.clip_action(5.0, 9.0), (2.0, 1.0))
        self.assertEqual(L.clip_action(-1.0, -9.0), (0.0, -1.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
