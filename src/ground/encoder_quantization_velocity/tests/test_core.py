"""Unit tests for the encoder quantisation module (NumPy only)."""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from encoder_quantization_velocity.core import (
    EncoderSpec,
    alpha_beta,
    kalman_velocity,
    moving_average,
    raw_difference,
)
from encoder_quantization_velocity.headless import run_experiment
from encoder_quantization_velocity.scene import ProfileConfig, measure, trapezoid


class EncoderTests(unittest.TestCase):
    def test_quantisation_error_is_bounded_by_half_a_step(self):
        spec = EncoderSpec(counts_per_revolution=1024)
        positions = np.linspace(0.0, 2.0, 500)
        quantised = spec.quantise(positions)
        self.assertLessEqual(float(np.max(np.abs(quantised - positions))), spec.step_rad / 2 + 1e-12)

    def test_step_scales_with_resolution(self):
        coarse = EncoderSpec(counts_per_revolution=1024).step_rad
        fine = EncoderSpec(counts_per_revolution=4096).step_rad
        self.assertAlmostEqual(coarse / fine, 4.0, places=9)

    def test_low_resolution_is_rejected(self):
        with self.assertRaises(ValueError):
            EncoderSpec(counts_per_revolution=8).validate()


class EstimatorTests(unittest.TestCase):
    def test_raw_difference_of_a_ramp_is_constant(self):
        dt = 0.001
        velocity = 3.0
        position = velocity * np.arange(100) * dt
        estimate = raw_difference(position, dt)
        np.testing.assert_allclose(estimate[1:], velocity, atol=1e-9)

    def test_moving_average_reduces_noise(self):
        rng = np.random.default_rng(0)
        signal = rng.normal(size=2000)
        smoothed = moving_average(signal, 9)
        self.assertLess(float(np.std(smoothed)), float(np.std(signal)))
        self.assertAlmostEqual(float(np.mean(smoothed)), 0.0, delta=0.05)

    def test_alpha_beta_converges_to_constant_velocity(self):
        dt = 0.001
        velocity = 2.0
        position = velocity * np.arange(600) * dt
        estimate = alpha_beta(position, dt)
        self.assertAlmostEqual(float(np.mean(estimate[-50:])), velocity, delta=0.2)

    def test_kalman_tracks_a_ramp(self):
        dt = 0.001
        velocity = 4.0
        position = velocity * np.arange(800) * dt
        estimate = kalman_velocity(position, dt, process_sigma=30.0, measurement_sigma=1e-3)
        self.assertAlmostEqual(float(np.mean(estimate[-50:])), velocity, delta=0.3)

    def test_bad_gains_are_rejected(self):
        with self.assertRaises(ValueError):
            alpha_beta(np.zeros(10), 0.001, alpha=0.0)
        with self.assertRaises(ValueError):
            moving_average(np.zeros(10), 0)


class ExperimentTests(unittest.TestCase):
    def test_trapezoid_profile_is_consistent(self):
        profile = trapezoid(ProfileConfig(duration_s=0.5))
        velocity = profile["velocity_rad_s"]
        self.assertGreaterEqual(float(np.min(velocity)), -1e-9)
        self.assertLessEqual(float(np.max(velocity)), ProfileConfig().peak_velocity + 1e-9)
        position = profile["position_rad"]
        self.assertTrue(np.all(np.diff(position) >= -1e-9))

    def test_measurement_is_deterministic(self):
        profile = trapezoid(ProfileConfig(duration_s=0.3))
        first = measure(profile, 4096, seed=3)
        second = measure(profile, 4096, seed=3)
        np.testing.assert_array_equal(first["position_rad"], second["position_rad"])

    def test_headless_writes_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            summary = run_experiment(out_dir=Path(directory), make_plots=False)
            stored = json.loads(
                (Path(directory) / "summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(stored["module"], "encoder_quantization_velocity")
            coarse = summary["resolutions"]["1024"]["raw"]["rmse_rad_s"]
            fine = summary["resolutions"]["16384"]["raw"]["rmse_rad_s"]
            self.assertGreater(coarse, 3.0 * fine)


if __name__ == "__main__":
    unittest.main()
