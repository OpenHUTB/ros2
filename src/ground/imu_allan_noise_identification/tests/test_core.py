"""Unit tests for the Allan-variance identification module.

Only NumPy is required: no ROS, no matplotlib, no external data.
"""

import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from imu_allan_noise_identification.core import (
    BIAS_INSTABILITY_FACTOR,
    NoiseModel,
    allan_deviation,
    cluster_means,
    fit_noise_model,
    slope,
)
from imu_allan_noise_identification.filters import TiltFilter, run_filter
from imu_allan_noise_identification.headless import run_experiment
from imu_allan_noise_identification.scene import (
    ImuConfig,
    simulate_static,
    simulate_tilt_sweep,
)

TRUTH = NoiseModel(arw=0.30, bias_instability=3.00)


def _white_series(count: int, dt: float, arw: float, seed: int = 0):
    model = NoiseModel(arw=arw, bias_instability=0.0)
    rate_variance, _ = model.sample_noise(dt)
    return np.random.default_rng(seed).normal(0.0, math.sqrt(rate_variance), count)


class AllanTests(unittest.TestCase):
    def test_white_noise_slope_is_minus_one_half(self):
        dt = 0.01
        rates = _white_series(20000, dt, 0.3)
        taus, adev = allan_deviation(rates, dt)
        self.assertAlmostEqual(slope(taus, adev), -0.5, delta=0.06)

    def test_cluster_means_matches_a_manual_average(self):
        rates = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
        np.testing.assert_allclose(cluster_means(rates, 2), [1.5, 2.5, 3.5, 4.5])

    def test_allan_rejects_bad_input(self):
        with self.assertRaises(ValueError):
            allan_deviation([1.0, 2.0], 0.01)
        with self.assertRaises(ValueError):
            allan_deviation([1.0, 2.0, 3.0, 4.0], 0.0)

    def test_model_curve_is_reproduced_exactly(self):
        taus = np.logspace(0.0, 3.0, 20)
        model = NoiseModel(arw=0.25, bias_instability=2.0)
        fitted = fit_noise_model(taus, model.adev(taus))
        self.assertAlmostEqual(fitted.arw, model.arw, places=3)
        self.assertAlmostEqual(fitted.bias_instability, model.bias_instability, places=3)

    def test_conversions_to_filter_units(self):
        model = NoiseModel(arw=0.3, bias_instability=3.0)
        self.assertAlmostEqual(model.angle_noise_density, 0.3 / 60.0, places=9)
        rate_variance, bias_variance = model.sample_noise(0.01)
        self.assertAlmostEqual(rate_variance, (0.3 / 60.0) ** 2 / 0.01, places=9)
        self.assertAlmostEqual(bias_variance, model.bias_walk_density**2 * 0.01, places=12)

    def test_bias_instability_floor_constant(self):
        self.assertAlmostEqual(BIAS_INSTABILITY_FACTOR, 0.6643, places=4)


class IdentificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = ImuConfig(static_duration_s=3600.0, noise=TRUTH, seed=0)
        cls.config = config
        cls.simulation = simulate_static(config)
        cls.taus, cls.adev = allan_deviation(
            cls.simulation["rate_deg_s"], cls.simulation["dt"]
        )
        cls.model = fit_noise_model(cls.taus, cls.adev)

    def test_angle_random_walk_within_three_percent(self):
        self.assertLess(abs(self.model.arw - TRUTH.arw) / TRUTH.arw, 0.03)

    def test_bias_instability_within_twenty_five_percent(self):
        self.assertLess(
            abs(self.model.bias_instability - TRUTH.bias_instability)
            / TRUTH.bias_instability,
            0.25,
        )

    def test_short_log_is_worse_for_bias_instability(self):
        short = simulate_static(ImuConfig(static_duration_s=600.0, noise=TRUTH, seed=0))
        taus, adev = allan_deviation(short["rate_deg_s"], short["dt"])
        model = fit_noise_model(taus, adev)
        short_error = abs(model.bias_instability - TRUTH.bias_instability)
        long_error = abs(self.model.bias_instability - TRUTH.bias_instability)
        self.assertGreater(short_error, long_error)

    def test_simulation_is_deterministic(self):
        config = ImuConfig(static_duration_s=60.0, noise=TRUTH, seed=7)
        first = simulate_static(config)
        second = simulate_static(config)
        np.testing.assert_array_equal(first["rate_deg_s"], second["rate_deg_s"])


class FilterTests(unittest.TestCase):
    def setUp(self):
        self.config = ImuConfig(sweep_duration_s=120.0, noise=TRUTH, seed=3)
        self.sweep = simulate_tilt_sweep(self.config)

    def test_predict_integrates_the_gyro(self):
        filter_ = TiltFilter.create(TRUTH, 0.35, initial_tilt_deg=0.0)
        filter_.predict(1.0, 0.5)
        self.assertAlmostEqual(filter_.state[0], 0.5, places=9)

    def test_update_reduces_angle_variance(self):
        filter_ = TiltFilter.create(TRUTH, 0.35, initial_tilt_deg=0.0)
        before = float(filter_.covariance[0, 0])
        filter_.update(0.1)
        self.assertLess(float(filter_.covariance[0, 0]), before)

    def test_tuned_filter_is_consistent(self):
        result = run_filter(self.sweep, TRUTH)
        self.assertLess(result["nees_mean"], 4.0)
        self.assertLess(result["rmse_deg"], 0.05)

    def test_over_optimistic_model_breaks_consistency(self):
        naive = NoiseModel(arw=0.075, bias_instability=0.75)
        tuned = run_filter(self.sweep, TRUTH)
        optimistic = run_filter(self.sweep, naive)
        self.assertGreater(optimistic["nees_mean"], 3.0 * tuned["nees_mean"])


class IntegrationTests(unittest.TestCase):
    def test_headless_writes_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            summary = run_experiment(
                out_dir=Path(directory),
                make_plots=False,
                static_s=300.0,
                sweep_s=60.0,
            )
            stored = json.loads(
                (Path(directory) / "summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(stored["module"], "imu_allan_noise_identification")
            self.assertIn("allan_tuned", summary["filters"])
            self.assertLess(summary["filters"]["allan_tuned"]["rmse_deg"], 0.1)


if __name__ == "__main__":
    unittest.main()
