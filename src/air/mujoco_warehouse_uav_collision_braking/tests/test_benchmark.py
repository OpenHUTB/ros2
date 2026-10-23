"""基准输出结构检查。 / Benchmark output structure checks."""

import tempfile
import unittest
from pathlib import Path

import numpy as np

from warehouse_braking.benchmark import benchmark
from warehouse_braking.model import RiskSpeedMLP


class BenchmarkTests(unittest.TestCase):
    def test_all_scenarios_are_reported(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            model = RiskSpeedMLP(seed=9)
            model.fit(np.zeros((12, 20), dtype=np.float32), np.ones(12), np.zeros(12, dtype=np.int64), epochs=1)
            model_path = root / "model.npz"
            model.save(model_path)
            result = benchmark(model_path, root / "benchmark.json")
            self.assertEqual(set(result), {"static", "crossing", "narrow"})
            for values in result.values():
                self.assertIn("collision", values)
                self.assertGreater(values["steps"], 0)
                self.assertTrue(0 <= values["risk_agreement"] <= 1)


if __name__ == "__main__":
    unittest.main()
