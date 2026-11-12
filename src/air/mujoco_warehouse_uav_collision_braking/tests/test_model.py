"""模型形状、范围和可复现性检查。 / Model shape, range and reproducibility checks."""

import tempfile
import unittest
from pathlib import Path

import numpy as np

from warehouse_braking.model import RiskSpeedMLP


class ModelTests(unittest.TestCase):
    def test_speed_regression_updates_with_batch_mean_gradient(self):
        x = np.zeros((40, 20), dtype=np.float32)
        target = np.full(40, 2.5, dtype=np.float32)
        risk = np.zeros(40, dtype=np.int64)
        model = RiskSpeedMLP(seed=9)
        before = float(model.predict(x[:1])[1][0])
        model.fit(x, target, risk, epochs=60, learning_rate=0.05)
        after = float(model.predict(x[:1])[1][0])
        self.assertGreater(after, before)

    def test_fit_and_round_trip(self):
        rng = np.random.default_rng(4)
        x = rng.normal(size=(60, 20)).astype(np.float32)
        risk = (x[:, 0] > 0).astype(np.int64)
        target = np.clip(1.0 - np.maximum(x[:, 0], 0), 0, 3).astype(np.float32)
        model = RiskSpeedMLP(seed=7)
        history = model.fit(x, target, risk, epochs=4)
        self.assertEqual(len(history), 4)
        probability, speed = model.predict(x[:5])
        self.assertEqual(probability.shape, (5, 3))
        self.assertEqual(speed.shape, (5,))
        self.assertTrue(np.allclose(probability.sum(1), 1.0))
        self.assertTrue(np.all((speed >= 0) & (speed <= 3)))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "model.npz"
            model.save(path)
            restored = RiskSpeedMLP.load(path)
            np.testing.assert_allclose(restored.predict(x[:5])[0], probability)


if __name__ == "__main__":
    unittest.main()
