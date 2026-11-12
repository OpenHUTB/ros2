"""Unit tests for the visual-inertial time-offset module (NumPy only)."""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from vio_time_offset_estimation.core import (
    align,
    cross_correlation,
    estimate_offset,
    residual_rms,
)
from vio_time_offset_estimation.headless import run_experiment
from vio_time_offset_estimation.scene import SignalConfig, simulate_pair


class CorrelationTests(unittest.TestCase):
    def test_peak_is_at_the_known_lag(self):
        rng = np.random.default_rng(0)
        signal = rng.normal(size=400)
        lag = 7
        delayed = np.concatenate([np.zeros(lag), signal[:-lag]])
        values = cross_correlation(signal, delayed, 20)
        self.assertEqual(int(np.argmax(values)), 20 + lag)

    def test_constant_signal_is_rejected(self):
        with self.assertRaises(ValueError):
            cross_correlation(np.ones(100), np.ones(100), 5)

    def test_length_mismatch_is_rejected(self):
        with self.assertRaises(ValueError):
            cross_correlation(np.arange(10.0), np.arange(9.0), 2)

    def test_lag_limit_is_validated(self):
        with self.assertRaises(ValueError):
            cross_correlation(np.arange(10.0), np.arange(10.0), 10)


class OffsetTests(unittest.TestCase):
    def test_exact_recovery_without_noise(self):
        config = SignalConfig(offset_s=0.025, gyro_noise=0.0, visual_noise=0.0,
                              gyro_bias=0.0, visual_gain=1.0)
        pair = simulate_pair(config)
        estimate = estimate_offset(pair["gyro"], pair["visual"], pair["dt"])
        self.assertAlmostEqual(estimate.offset_s, config.offset_s, delta=pair["dt"])

    def test_negative_offset_is_recovered(self):
        config = SignalConfig(offset_s=-0.030, gyro_noise=0.0, visual_noise=0.0,
                              gyro_bias=0.0, visual_gain=1.0)
        pair = simulate_pair(config)
        estimate = estimate_offset(pair["gyro"], pair["visual"], pair["dt"])
        self.assertAlmostEqual(estimate.offset_s, config.offset_s, delta=pair["dt"])

    def test_noise_degrades_but_stays_small(self):
        errors = []
        for seed in range(20):
            config = SignalConfig(offset_s=0.025, seed=seed)
            pair = simulate_pair(config)
            estimate = estimate_offset(pair["gyro"], pair["visual"], pair["dt"])
            errors.append(abs(estimate.offset_s - config.offset_s))
        self.assertLess(float(np.mean(errors)), 0.003)

    def test_alignment_trims_the_overlap(self):
        gyro = np.arange(100.0)
        visual = np.arange(100.0) * 0.5
        left, right = align(gyro, visual, 0.05, 0.01)
        self.assertEqual(left.size, right.size)
        self.assertEqual(left.size, 95)

    def test_residual_drops_after_compensation(self):
        config = SignalConfig(offset_s=0.02, seed=3)
        pair = simulate_pair(config)
        before = residual_rms(pair["gyro"], pair["visual"], 0.0, pair["dt"], config.visual_gain)
        after = residual_rms(pair["gyro"], pair["visual"], 0.02, pair["dt"], config.visual_gain)
        self.assertLess(after, before)


class SceneTests(unittest.TestCase):
    def test_simulation_is_deterministic(self):
        first = simulate_pair(SignalConfig(seed=9))
        second = simulate_pair(SignalConfig(seed=9))
        np.testing.assert_array_equal(first["gyro"], second["gyro"])
        np.testing.assert_array_equal(first["visual"], second["visual"])

    def test_offset_beyond_the_limit_is_rejected(self):
        with self.assertRaises(ValueError):
            SignalConfig(offset_s=0.5).validate()


class ExperimentTests(unittest.TestCase):
    def test_headless_writes_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            summary = run_experiment(
                out_dir=Path(directory), make_plots=False, trials=15
            )
            stored = json.loads(
                (Path(directory) / "summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(stored["module"], "vio_time_offset_estimation")
            self.assertLess(summary["offset_error_ms"]["rmse"], 5.0)
            self.assertGreater(summary["residual"]["improvement_pct"], 0.0)


if __name__ == "__main__":
    unittest.main()
