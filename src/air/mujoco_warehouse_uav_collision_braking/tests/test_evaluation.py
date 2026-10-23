"""评估结果结构检查。 / Evaluation result structure checks."""

import tempfile
import unittest
from pathlib import Path

import numpy as np

from warehouse_braking.evaluate_model import evaluate
from warehouse_braking.model import RiskSpeedMLP


class EvaluationTests(unittest.TestCase):
    def test_metrics_are_bounded_and_saved(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            rng = np.random.default_rng(2)
            x = rng.normal(size=(18, 20)).astype(np.float32)
            target = np.ones(18, dtype=np.float32)
            risk = np.arange(18) % 3
            np.savez(root / "test.npz", x=x, target_speed=target, risk=risk)
            model_dir = root / "model"
            model_dir.mkdir()
            model = RiskSpeedMLP(seed=2)
            model.fit(x, target, risk, epochs=2)
            model.save(model_dir / "risk_speed_mlp.npz")
            result = evaluate(root, model_dir)
            self.assertEqual(result["test_samples"], 18)
            self.assertTrue(0 <= result["risk_accuracy"] <= 1)
            self.assertGreaterEqual(result["speed_mae_mps"], 0)
            self.assertEqual(np.asarray(result["confusion_matrix"]).shape, (3, 3))
            self.assertTrue((model_dir / "test_metrics.json").exists())


if __name__ == "__main__":
    unittest.main()
