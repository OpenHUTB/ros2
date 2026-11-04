# -*- coding: utf-8 -*-
"""风险网络与 AUC 指标测试（不依赖 MuJoCo）。"""
import tempfile
import unittest
from pathlib import Path

import numpy as np

from mujoco_car_rollover_guard.model import RiskMLP, auc_score


def make_toy_data(n=200, dim=9, seed=7):
    rng = np.random.default_rng(seed)
    label = (rng.random(n) < 0.3).astype(np.float64)
    X = rng.normal(size=(n, dim))
    # 让第一维与标签相关，保证网络可学到东西
    X[:, 0] += 2.5 * label
    return X, label


class TestRiskMLP(unittest.TestCase):
    def test_forward_output_shapes(self):
        net = RiskMLP(9, 16, seed=1)
        X = np.zeros((5, 9))
        logit, a1 = net.forward(X)
        self.assertEqual(logit.shape, (5,))
        self.assertEqual(a1.shape, (5, 16))

    def test_probability_range(self):
        net = RiskMLP(9, 16, seed=1)
        p = net.predict_proba(np.zeros((4, 9)))
        self.assertTrue(np.all(p >= 0.0) and np.all(p <= 1.0))

    def test_single_sample_prediction(self):
        net = RiskMLP(9, 16, seed=1)
        p = net.predict_proba(np.zeros(9))
        self.assertIsInstance(p, float)

    def test_training_reduces_loss(self):
        X, y = make_toy_data()
        net = RiskMLP(9, 16, seed=3)
        losses = net.fit(X, y, epochs=120, batch=32, lr=0.02, seed=3)
        self.assertLess(losses[-1], losses[0])

    def test_learned_model_separates_toy_data(self):
        X, y = make_toy_data()
        net = RiskMLP(9, 16, seed=3)
        net.fit(X, y, epochs=150, batch=32, lr=0.02, seed=3)
        acc = float(((net.predict_proba(X) >= 0.5) == (y >= 0.5)).mean())
        self.assertGreater(acc, 0.85)

    def test_save_and_load_roundtrip(self):
        X, y = make_toy_data()
        net = RiskMLP(9, 16, seed=3)
        net.fit(X, y, epochs=80, batch=32, lr=0.02, seed=3)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "w.npz"
            net.save(path)
            loaded = RiskMLP.load(path)
        np.testing.assert_allclose(net.predict_proba(X), loaded.predict_proba(X), rtol=1e-6)


class TestAUC(unittest.TestCase):
    def test_perfect_separation(self):
        self.assertAlmostEqual(auc_score([0.1, 0.2, 0.8, 0.9], [0, 0, 1, 1]), 1.0)

    def test_all_tied_probabilities_is_half(self):
        self.assertAlmostEqual(auc_score([0.5, 0.5, 0.5, 0.5], [0, 0, 1, 1]), 0.5)

    def test_single_class_returns_nan(self):
        self.assertTrue(np.isnan(auc_score([0.2, 0.4], [0, 0])))


if __name__ == "__main__":
    unittest.main()
