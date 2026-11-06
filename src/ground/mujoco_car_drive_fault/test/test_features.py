# -*- coding: utf-8 -*-
"""特征构造与归一化测试（不依赖 MuJoCo）。"""
import unittest

import numpy as np

from mujoco_car_drive_fault.config import Config
from mujoco_car_drive_fault.features import from_state, normalize


class TestFeatures(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def test_feature_vector_dimension(self):
        raw = from_state(0.0, 0.0, (0, 0, 0), (0, 0, 9.81), 1.0, 20.0, 20.0, 0.0, 0.0)
        self.assertEqual(raw.size, self.cfg.feature_dim)

    def test_keeps_lateral_offset_and_yaw(self):
        raw = from_state(0.35, -0.2, (0, 0, 0.6), (0, 1.2, 9.8), 1.0, 19.0, 21.0, 20.0, 20.0)
        self.assertAlmostEqual(raw[0], 0.35)
        self.assertAlmostEqual(raw[1], -0.2)

    def test_keeps_yaw_rate_and_lateral_accel(self):
        raw = from_state(0.0, 0.0, (0, 0, 0.5), (0, 1.5, 9.8), 1.0, 20.0, 20.0, 20.0, 20.0)
        self.assertAlmostEqual(raw[2], 0.5)
        self.assertAlmostEqual(raw[3], 1.5)

    def test_keeps_wheel_speeds_and_commands(self):
        raw = from_state(0.0, 0.0, (0, 0, 0), (0, 0, 9.81), 1.0, 18.0, 20.0, 20.0, 20.0)
        self.assertAlmostEqual(raw[5], 18.0)
        self.assertAlmostEqual(raw[6], 20.0)
        self.assertAlmostEqual(raw[7], 20.0)
        self.assertAlmostEqual(raw[8], 20.0)

    def test_normalize_divides_by_scale(self):
        raw = from_state(0.5, 0.3, (0, 0, 1.0), (0, 0, 9.81), 1.0, 6.0, 20.0, 20.0, 20.0)
        norm = normalize(raw, self.cfg)
        self.assertAlmostEqual(norm[0], 0.5 / self.cfg.scales[0])
        self.assertAlmostEqual(norm[5], 6.0 / self.cfg.scales[5])

    def test_normalize_clips_outliers(self):
        raw = np.full(self.cfg.feature_dim, 1000.0)
        self.assertTrue(np.all(np.abs(normalize(raw, self.cfg)) <= 3.0))

    def test_normalize_rejects_wrong_dimension(self):
        with self.assertRaises(ValueError):
            normalize(np.zeros(5), self.cfg)


if __name__ == "__main__":
    unittest.main()
