"""Unit tests for the RTK ambiguity-resolution module (NumPy only)."""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from rtk_ambiguity_resolution.core import (
    L1_WAVELENGTH_M,
    dd_operator,
    decorrelate,
    fix,
    float_solution,
    geometry_matrix,
    search_integers,
)
from rtk_ambiguity_resolution.headless import evaluate_block, run_experiment, summarize, window_blocks
from rtk_ambiguity_resolution.scene import (
    RtkConfig,
    Satellite,
    simulate,
    simulate_epoch,
    stack_epochs,
    true_ambiguity,
)

DESIGN = np.array(
    [[0.30, 0.50, 0.80], [-0.20, 0.70, 0.60], [0.10, -0.60, 0.80], [-0.50, -0.30, 0.80]]
)
BASELINE = np.array([0.80, 0.60, 0.10])
AMBIGUITY = np.array([10, 20, 30, 40])


class GeometryTests(unittest.TestCase):
    def test_satellite_unit_vector_is_normalised(self):
        vector = Satellite(30.0, 45.0).unit_vector()
        self.assertAlmostEqual(float(np.linalg.norm(vector)), 1.0, places=12)
        self.assertGreater(vector[2], 0.0)

    def test_dd_operator_shape_and_reference(self):
        operator = dd_operator(4)
        self.assertEqual(operator.shape, (3, 4))
        np.testing.assert_array_equal(operator.sum(axis=1), np.zeros(3))

    def test_geometry_matrix_shape(self):
        satellites = tuple(Satellite(az, 45.0) for az in (0.0, 90.0, 180.0, 270.0, 45.0))
        design = geometry_matrix(satellites)
        self.assertEqual(design.shape, (4, 3))
        self.assertTrue(np.all(np.isfinite(design)))


class FloatSolutionTests(unittest.TestCase):
    def test_exact_data_recovers_baseline_and_ambiguity(self):
        phase = (DESIGN @ BASELINE) / L1_WAVELENGTH_M + AMBIGUITY
        solution = float_solution(DESIGN, phase, DESIGN @ BASELINE)
        np.testing.assert_allclose(solution.baseline, BASELINE, atol=1e-9)
        np.testing.assert_allclose(solution.ambiguity, AMBIGUITY, atol=1e-9)

    def test_shape_mismatch_is_rejected(self):
        with self.assertRaises(ValueError):
            float_solution(DESIGN, np.zeros(3), np.zeros(3))

    def test_code_noise_limits_the_single_epoch_ambiguity(self):
        # One epoch is code-limited: the ambiguity cannot be better than the
        # baseline knowledge the code provides, so sigma_code drives Q_N.
        phase = (DESIGN @ BASELINE) / L1_WAVELENGTH_M + AMBIGUITY
        good = float_solution(DESIGN, phase, DESIGN @ BASELINE, sigma_code_m=0.3048)
        poor = float_solution(DESIGN, phase, DESIGN @ BASELINE, sigma_code_m=3.048)
        ratio = float(np.trace(poor.cov_nn)) / float(np.trace(good.cov_nn))
        self.assertGreater(ratio, 50.0)


class LambdaTests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(0)
        phase = (DESIGN @ BASELINE) / L1_WAVELENGTH_M + AMBIGUITY
        self.solution = float_solution(
            DESIGN, phase + rng.normal(0, 0.01, 4), DESIGN @ BASELINE
        )

    def test_transform_is_unimodular_and_integral(self):
        transform = decorrelate(self.solution.cov_nn)
        self.assertTrue(np.issubdtype(transform.dtype, np.integer))
        self.assertEqual(abs(int(round(np.linalg.det(transform.astype(float))))), 1)

    def test_decorrelation_reduces_the_condition_number(self):
        transform = decorrelate(self.solution.cov_nn)
        before = float(np.linalg.cond(self.solution.cov_nn))
        after = float(np.linalg.cond(transform @ self.solution.cov_nn @ transform.T))
        self.assertLess(after, before)

    def test_search_returns_sorted_candidates(self):
        covariance = np.diag([0.01, 0.02])
        candidates = search_integers(covariance, np.array([0.2, -0.3]))
        self.assertGreater(len(candidates), 2)
        costs = [cost for cost, _ in candidates]
        self.assertEqual(costs, sorted(costs))


class FixTests(unittest.TestCase):
    def _solution(self, sigma=0.01, seed=0):
        rng = np.random.default_rng(seed)
        phase = (DESIGN @ BASELINE) / L1_WAVELENGTH_M + AMBIGUITY
        return float_solution(
            DESIGN, phase + rng.normal(0, sigma, 4), DESIGN @ BASELINE
        )

    def test_clean_epoch_is_fixed_to_millimetres(self):
        solution = self._solution(0.01)
        result = fix(solution, decorrelate(solution.cov_nn))
        np.testing.assert_array_equal(result.integer_ambiguity, AMBIGUITY)
        self.assertLess(float(np.linalg.norm(result.baseline - BASELINE)), 0.005)
        self.assertTrue(result.fixed)

    def test_large_noise_is_not_fixed(self):
        solution = self._solution(0.6, seed=3)
        result = fix(solution, decorrelate(solution.cov_nn), ratio_threshold=3.0)
        self.assertLess(result.ratio, 20.0)

    def test_multi_epoch_block_fixes_to_millimetres(self):
        config = RtkConfig(epochs=20, epochs_per_fix=20, seed=0)
        block = stack_epochs(simulate(config))
        solution = float_solution(
            block["design"],
            block["phase_cycles"],
            block["code_m"],
            sigma_phase_cycles=config.sigma_phase_cycles,
            ambiguity_count=len(block["ambiguity"]),
        )
        result = fix(solution, decorrelate(solution.cov_nn))
        self.assertLess(float(np.linalg.norm(result.baseline - block["baseline"])), 0.01)


class SceneTests(unittest.TestCase):
    def test_ambiguity_is_integer_double_difference(self):
        config = RtkConfig()
        ambiguity = true_ambiguity(config)
        self.assertEqual(ambiguity.dtype.kind, "i")
        np.testing.assert_allclose(ambiguity, np.rint(ambiguity))

    def test_simulation_is_deterministic(self):
        config = RtkConfig(epochs=40, epochs_per_fix=20, seed=5)
        first = simulate(config)
        second = simulate(config)
        for left, right in zip(first, second, ):
            np.testing.assert_array_equal(left["phase_cycles"], right["phase_cycles"])
            np.testing.assert_array_equal(left["code_m"], right["code_m"])

    def test_stacking_repeats_the_same_ambiguity(self):
        config = RtkConfig(epochs=40, epochs_per_fix=20, seed=0)
        epoch = simulate_epoch(config, AMBIGUITY, np.random.default_rng(0))
        block = stack_epochs([epoch, epoch, epoch])
        self.assertEqual(block["phase_cycles"].size, 12)
        np.testing.assert_array_equal(block["ambiguity"], AMBIGUITY)


class ExperimentTests(unittest.TestCase):
    def test_summary_reports_rates(self):
        config = RtkConfig(epochs=80, epochs_per_fix=20, seed=0)
        epochs = simulate(config)
        rows = [evaluate_block(block, config.sigma_phase_cycles) for block in window_blocks(epochs, 20)]
        summary = summarize(rows)
        self.assertEqual(summary["attempts"], 4)
        self.assertLessEqual(summary["success_rate_pct"], 100.0)
        self.assertGreaterEqual(summary["fixed_rmse_mm"], 0.0)

    def test_headless_writes_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            summary = run_experiment(out_dir=Path(directory), make_plots=False)
            stored = json.loads(
                (Path(directory) / "summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(stored["module"], "rtk_ambiguity_resolution")
            self.assertGreater(stored["main"]["float_rmse_mm"], stored["main"]["fixed_rmse_mm"])
            self.assertEqual(stored["decorrelation"]["determinant"] in (-1, 1), True)


if __name__ == "__main__":
    unittest.main()
