"""Contract tests for the telemetry-imputation module."""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from telemetry_imputation import data, gen, sim
from telemetry_imputation.model import TORCH_AVAILABLE

if TORCH_AVAILABLE:
    from telemetry_imputation.model import Imputer


class Simulator(unittest.TestCase):
    def test_tracks_reference_path(self):
        rng = np.random.default_rng(0)
        points = sim.random_path(rng, n_points=2, radius=3.0, height=(2.0, 3.0))
        rows = sim.simulate(points, rng)
        path, _, _ = sim.waypoint_path(points)
        errors = [np.linalg.norm(path(row[0])[0] - row[1:4]) for row in rows]
        self.assertLess(float(np.mean(errors)), 0.20)
        self.assertLess(float(np.max(errors)), 0.60)

    def test_records_at_twenty_hertz(self):
        rng = np.random.default_rng(1)
        rows = sim.simulate(sim.random_path(rng, n_points=2), rng)
        dt = np.diff(rows[:, 0])
        self.assertTrue(np.allclose(dt, 0.05, atol=1e-6))


class Masks(unittest.TestCase):
    def test_mask_is_deterministic(self):
        state = np.arange(200 * 6, dtype=np.float32).reshape(200, 6)
        thrust = np.ones(200, np.float32)
        t = np.arange(200) * 0.05
        a = data.make_windows(state, thrust, t, np.random.default_rng(7), gap=(5, 5))
        b = data.make_windows(state, thrust, t, np.random.default_rng(7), gap=(5, 5))
        self.assertTrue(np.array_equal(a[2], b[2]))

    def test_gap_length_matches_request(self):
        state = np.arange(200 * 6, dtype=np.float32).reshape(200, 6)
        thrust = np.ones(200, np.float32)
        t = np.arange(200) * 0.05
        xs, ys, ms, gs = data.make_windows(state, thrust, t, np.random.default_rng(3), gap=(6, 6))
        self.assertTrue(np.all(gs == 6))
        self.assertTrue(np.all(ms.sum(axis=1) == data.WINDOW - 6))

    def test_masked_input_is_zeroed(self):
        state = np.arange(200 * 6, dtype=np.float32).reshape(200, 6)
        thrust = np.ones(200, np.float32)
        t = np.arange(200) * 0.05
        xs, _, ms, _ = data.make_windows(state, thrust, t, np.random.default_rng(5), gap=(4, 4))
        for k in range(len(xs)):
            self.assertTrue(np.all(xs[k][~ms[k], :6] == 0))


class Baselines(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(11)
        self.y = rng.normal(size=(3, data.WINDOW, 6)).astype(np.float32)
        self.ms = np.ones((3, data.WINDOW), bool)
        self.ms[:, 10:14] = False

    def test_zoh_reproduces_observed_points(self):
        out = data.baselines(self.y, self.ms, 'zoh')
        self.assertTrue(np.allclose(out[self.ms], self.y[self.ms]))

    def test_zoh_holds_last_observation_inside_gap(self):
        out = data.baselines(self.y, self.ms, 'zoh')
        for k in range(10, 14):
            self.assertTrue(np.allclose(out[:, k], self.y[:, 9]))

    def test_normalizer_uses_only_observed_entries(self):
        corrupt = self.y.copy()
        corrupt[~self.ms] = 1e6
        mu_a, sd_a = data.normalizer(None, self.y, self.ms)
        mu_b, sd_b = data.normalizer(None, corrupt, self.ms)
        self.assertTrue(np.allclose(mu_a, mu_b))
        self.assertTrue(np.allclose(sd_a, sd_b))


@unittest.skipUnless(TORCH_AVAILABLE, 'torch not installed')
class ModelShape(unittest.TestCase):
    def test_output_matches_window_length(self):
        import torch
        net = Imputer(24)
        x = torch.zeros(2, data.WINDOW, 9)
        self.assertEqual(tuple(net(x).shape), (2, data.WINDOW, 6))

    def test_causal_by_construction(self):
        """The GRU is unidirectional: perturbing a later step cannot change an earlier output."""
        import torch
        net = Imputer(16)
        net.eval()
        x = torch.zeros(1, data.WINDOW, 9)
        with torch.no_grad():
            a = net(x)
            y = x.clone(); y[0, 30, 0] = 5.0
            b = net(y)
        self.assertTrue(torch.allclose(a[0, :30], b[0, :30], atol=1e-6))


class CorpusSplit(unittest.TestCase):
    def test_splits_are_disjoint(self):
        with tempfile.TemporaryDirectory() as tmp:
            manifest = gen.generate(tmp, n_flights=8)
            seeds = {}
            for item in manifest:
                seeds.setdefault(item['split'], set()).add(item['seed'])
            self.assertEqual(len(seeds['train'] & seeds['test']), 0)


class NumpyBackend(unittest.TestCase):
    """The NumPy fallback must agree with torch, so results are checkable without it."""

    def test_numpy_forward_matches_torch(self):
        try:
            import torch
        except ImportError:
            self.skipTest('torch not installed')
        from telemetry_imputation import numpy_infer
        net = Imputer(24)
        net.eval()
        x = np.random.default_rng(3).normal(size=(3, data.WINDOW, 9)).astype(np.float32)
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            numpy_infer.export_weights(net, folder)
            weights = np.load(str(folder / 'weights.npz'))
            weights = {k: weights[k].astype(np.float64) for k in weights.files}
        with torch.no_grad():
            expected = net(torch.tensor(x)).numpy()
        got = numpy_infer.forward(weights, x.astype(np.float64))
        self.assertLess(float(np.abs(got - expected).max()), 1e-5)

    def test_shipped_weights_exist(self):
        folder = Path(__file__).resolve().parent.parent / 'models/imputer_42'
        self.assertTrue((folder / 'weights.npz').is_file())
        weights, cfg = numpy_infer_load(folder)
        self.assertEqual(weights['weight_hh_l0'].shape[1], cfg['hidden'])

    def test_shipped_numpy_eval_matches_reported_metrics(self):
        """Reproduce the headline numbers through the NumPy path only."""
        from telemetry_imputation import data, evaluate, numpy_infer
        folder = Path(__file__).resolve().parent.parent
        weights, cfg = numpy_infer.load(folder / 'models/imputer_42')
        xs, ys, ms, _ = data.dataset(folder / 'flights', 'test', np.random.default_rng(909))
        pred = numpy_infer.predict(weights, cfg, xs, ys, ms)
        pos, vel = evaluate.rmse(pred, ys, ms)
        self.assertAlmostEqual(pos, 0.168, delta=1e-3)
        self.assertAlmostEqual(vel, 0.250, delta=1e-3)


def numpy_infer_load(folder):
    from telemetry_imputation import numpy_infer
    return numpy_infer.load(folder)


if __name__ == '__main__':
    unittest.main()
