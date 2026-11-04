# -*- coding: utf-8 -*-
"""MLP 网络单测（不需要 MuJoCo）。

覆盖：输入输出维度、输出范围约束、单样本/批量形状、权重保存加载一致性。
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mujoco_car_nn_avoidance.model import MLP  # noqa: E402


class TestMLPDimensions(unittest.TestCase):
    def setUp(self):
        self.net = MLP(input_dim=26, hidden=32, output_dim=2, seed=42)

    def test_batch_forward_shape(self):
        x = np.zeros((5, 26), dtype=np.float32)
        y = self.net.predict(x)
        self.assertEqual(y.shape, (5, 2))

    def test_single_sample_shape(self):
        x = np.zeros(26, dtype=np.float32)
        y = self.net.predict(x)
        self.assertEqual(y.shape, (2,))

    def test_steer_always_in_range(self):
        rng = np.random.default_rng(0)
        x = rng.normal(size=(64, 26)).astype(np.float32)
        y = self.net.predict(x)
        self.assertTrue(np.all(y[:, 0] >= -1.0))
        self.assertTrue(np.all(y[:, 0] <= 1.0))

    def test_throttle_always_nonnegative(self):
        rng = np.random.default_rng(1)
        x = rng.normal(size=(64, 26)).astype(np.float32) * 10.0
        y = self.net.predict(x)
        self.assertTrue(np.all(y[:, 1] >= 0.0))
        self.assertTrue(np.all(y[:, 1] <= 1.0))

    def test_hidden_weights_are_initialized(self):
        self.assertEqual(self.net.W1.shape, (32, 26))
        self.assertEqual(self.net.W2.shape, (2, 32))
        # He 初始化后隐层权重不应全为 0
        self.assertGreater(np.abs(self.net.W1).sum(), 1e-6)


class TestMLPPersistence(unittest.TestCase):
    def test_save_load_keeps_weights_and_predictions(self):
        net = MLP(input_dim=26, hidden=32, output_dim=2, seed=7)
        rng = np.random.default_rng(2)
        x = rng.normal(size=(4, 26)).astype(np.float32)
        expected = net.predict(x)

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "weights.npz"
            net.save(path)
            restored = MLP.load(path)

        np.testing.assert_allclose(restored.W1, net.W1)
        np.testing.assert_allclose(restored.b2, net.b2)
        np.testing.assert_allclose(restored.predict(x), expected, atol=1e-6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
