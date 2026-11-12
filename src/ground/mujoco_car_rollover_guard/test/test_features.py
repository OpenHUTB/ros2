# -*- coding: utf-8 -*-
"""特征构造与归一化测试（不依赖 MuJoCo）。"""
import math
import unittest

import numpy as np

from mujoco_car_rollover_guard.config import Config
from mujoco_car_rollover_guard.features import from_imu, normalize, roll_pitch

IDENTITY = (1.0, 0.0, 0.0, 0.0)


def quat_from_roll(deg):
    half = math.radians(deg) / 2.0
    return (math.cos(half), math.sin(half), 0.0, 0.0)


class TestFeatures(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def test_identity_attitude_has_zero_angles(self):
        roll, pitch = roll_pitch(IDENTITY)
        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)

    def test_roll_recovered_from_quaternion(self):
        quat = quat_from_roll(30.0)
        roll, _ = roll_pitch(quat)
        self.assertAlmostEqual(math.degrees(roll), 30.0, places=3)

    def test_feature_vector_dimension(self):
        raw = from_imu(IDENTITY, (0, 0, 0), (0, 0, 9.81), 1.0, 0.3)
        self.assertEqual(raw.size, self.cfg.feature_dim)

    def test_feature_keeps_speed_and_steer(self):
        raw = from_imu(IDENTITY, (0, 0, 0), (0, 0, 9.81), 1.2, -0.4)
        self.assertAlmostEqual(raw[7], 1.2)
        self.assertAlmostEqual(raw[8], -0.4)

    def test_normalize_divides_by_scale(self):
        raw = from_imu(quat_from_roll(30.0), (1.0, 0, 0), (0, 0, 9.81), 1.0, 0.5)
        norm = normalize(raw, self.cfg)
        self.assertAlmostEqual(norm[0], math.radians(30.0) / self.cfg.scales[0])
        self.assertAlmostEqual(norm[8], 0.5)

    def test_normalize_clips_outliers(self):
        raw = np.full(self.cfg.feature_dim, 100.0)
        self.assertTrue(np.all(np.abs(normalize(raw, self.cfg)) <= 3.0))

    def test_normalize_rejects_wrong_dimension(self):
        with self.assertRaises(ValueError):
            normalize(np.zeros(5), self.cfg)


if __name__ == "__main__":
    unittest.main()
