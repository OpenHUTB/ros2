# -*- coding: utf-8 -*-
"""标签生成与配置一致性测试（不依赖 MuJoCo）。"""
import unittest

import numpy as np

from mujoco_car_drive_fault.config import Config, MODELS_DIR
from mujoco_car_drive_fault.scenario import hold_throttle, label_deviate


class TestLabeling(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def test_horizon_window_marked_around_deviation(self):
        feat = np.zeros((100, self.cfg.feature_dim))
        y = label_deviate(feat, True, t_deviate=1.0, cfg=self.cfg)
        last = int(round(1.0 / self.cfg.dt))
        start = last - self.cfg.horizon_steps + 1
        end = min(feat.shape[0] - 1, last + self.cfg.horizon_steps)
        self.assertEqual(int(y.sum()), end - start + 1)
        self.assertEqual(y[start - 1], 0.0)
        self.assertEqual(y[start], 1.0)
        self.assertEqual(y[last], 1.0)
        
        self.assertEqual(y[end], 1.0)

    def test_no_deviation_all_negative(self):
        feat = np.zeros((50, self.cfg.feature_dim))
        y = label_deviate(feat, False, None, self.cfg)
        self.assertEqual(int(y.sum()), 0)

    def test_short_episode_does_not_underflow(self):
        feat = np.zeros((10, self.cfg.feature_dim))
        y = label_deviate(feat, True, 0.1, self.cfg)
        self.assertTrue(y.sum() > 0)

    def test_throttle_increases_when_slower_than_target(self):
        self.assertGreater(hold_throttle(0.4, 1.0, self.cfg), hold_throttle(1.0, 1.0, self.cfg))

    def test_throttle_within_unit_range(self):
        self.assertLessEqual(hold_throttle(0.0, 5.0, self.cfg), 1.0)
        self.assertGreaterEqual(hold_throttle(99.0, 0.0, self.cfg), 0.0)


class TestConfig(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def test_three_fault_scenes_exist(self):
        self.assertEqual(len(self.cfg.scenes), 3)
        for scene, damping in self.cfg.scenes:
            self.assertTrue((MODELS_DIR / scene).exists(), scene)
            self.assertGreater(damping, 0.0)

    def test_fault_severity_increases(self):
        dampings = [d for _, d in self.cfg.scenes]
        self.assertEqual(dampings, sorted(dampings))

    def test_both_fault_sides(self):
        self.assertEqual(set(self.cfg.fault_sides), {"left", "right"})

    def test_control_period_matches_timestep(self):
        self.assertAlmostEqual(self.cfg.steps_per_control * self.cfg.timestep, self.cfg.dt)

    def test_horizon_steps_positive(self):
        self.assertGreaterEqual(self.cfg.horizon_steps, 1)

    def test_feature_scales_match_dimension(self):
        self.assertEqual(len(self.cfg.scales), self.cfg.feature_dim)


if __name__ == "__main__":
    unittest.main()
