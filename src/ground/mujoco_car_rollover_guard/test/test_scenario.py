# -*- coding: utf-8 -*-
"""标签生成与配置一致性测试（不依赖 MuJoCo）。"""
import unittest

import numpy as np

from mujoco_car_rollover_guard.config import Config, MODELS_DIR
from mujoco_car_rollover_guard.scenario import label_rollover


class TestLabeling(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def test_horizon_window_marked_before_rollover(self):
        feat = np.zeros((100, self.cfg.feature_dim))
        y = label_rollover(feat, True, t_roll=1.0, cfg=self.cfg)
        last = int(round(1.0 / self.cfg.dt)) - 1
        self.assertEqual(int(y.sum()), self.cfg.horizon_steps)
        self.assertEqual(y[last], 1.0)
        self.assertEqual(y[last - self.cfg.horizon_steps], 0.0)

    def test_no_rollover_all_negative(self):
        feat = np.zeros((50, self.cfg.feature_dim))
        y = label_rollover(feat, False, None, self.cfg)
        self.assertEqual(int(y.sum()), 0)

    def test_short_episode_does_not_underflow(self):
        feat = np.zeros((10, self.cfg.feature_dim))
        y = label_rollover(feat, True, 0.1, self.cfg)
        self.assertTrue(y.sum() > 0)


class TestConfig(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def test_three_payload_scenes_exist(self):
        self.assertEqual(len(self.cfg.scenes), 3)
        for scene, height in self.cfg.scenes:
            self.assertTrue((MODELS_DIR / scene).exists(), scene)
            self.assertGreater(height, 0.0)

    def test_control_period_matches_timestep(self):
        self.assertAlmostEqual(self.cfg.steps_per_control * self.cfg.timestep, self.cfg.dt)

    def test_horizon_steps_positive(self):
        self.assertGreaterEqual(self.cfg.horizon_steps, 1)

    def test_feature_scales_match_dimension(self):
        self.assertEqual(len(self.cfg.scales), self.cfg.feature_dim)


if __name__ == "__main__":
    unittest.main()
