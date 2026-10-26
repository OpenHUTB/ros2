"""数据边界与轨迹隔离检查。 / Dataset boundary and episode isolation checks."""

import tempfile
import unittest
from pathlib import Path

import numpy as np

from warehouse_braking.dataset import audit, collect, features


class DatasetTests(unittest.TestCase):
    def test_feature_history_is_causal_and_resets(self):
        current = np.ones(9)
        previous = np.ones(9) * 1.1
        self.assertTrue(np.all(features(current, None, 1, 2)[9:18] == 0))
        np.testing.assert_allclose(features(current, previous, 1, 2)[9:18], 5.0)

    def test_seed_reproduces_arrays_and_splits_are_disjoint(self):
        with tempfile.TemporaryDirectory() as first, tempfile.TemporaryDirectory() as second:
            a = collect(first, episodes=15, duration=0.1)
            b = collect(second, episodes=15, duration=0.1)
            self.assertEqual(a, b)
            self.assertEqual(sum(v["episodes"] for v in audit(first).values()), 15)
            scenarios = {e["scenario"] for e in a["episodes"] if e["split"] == "test"}
            self.assertEqual(scenarios, {"static", "crossing", "narrow"})

    def test_audit_rejects_episode_leakage(self):
        with tempfile.TemporaryDirectory() as folder:
            collect(folder, episodes=15, duration=0.1)
            path = Path(folder) / "test.npz"
            with np.load(path) as data:
                arrays = {key: data[key].copy() for key in data.files}
            arrays["episode"][:] = 0
            np.savez_compressed(path, **arrays)
            with self.assertRaisesRegex(ValueError, "leakage"):
                audit(folder)


if __name__ == "__main__":
    unittest.main()
