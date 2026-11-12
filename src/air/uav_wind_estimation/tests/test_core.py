"""Unit tests for the wind-estimation module (NumPy only)."""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from uav_wind_estimation.core import (
    airspeed_estimate,
    estimate_wind,
    fisher_information,
    observability,
    reciprocal_estimate,
    residual_vector,
)
from uav_wind_estimation.headless import run_experiment
from uav_wind_estimation.scene import WindConfig, headings_for, simulate_maneuver


class ObservabilityTests(unittest.TestCase):
    def test_hover_is_not_observable(self):
        result = observability(headings_for("hover"))
        self.assertFalse(result["observable"])

    def test_circle_is_observable(self):
        result = observability(headings_for("circle"))
        self.assertTrue(result["observable"])
        self.assertLess(result["condition"], 2.0)

    def test_information_matrix_scales_with_noise(self):
        quiet = fisher_information(headings_for("circle"), 0.1)
        noisy = fisher_information(headings_for("circle"), 0.2)
        self.assertGreater(float(np.trace(quiet)), 3.0 * float(np.trace(noisy)))


class EstimatorTests(unittest.TestCase):
    def test_noiseless_circle_recovers_the_wind(self):
        config = WindConfig(gps_noise_m_s=0.0, airspeed_noise_m_s=0.0,
                            heading_noise_deg=0.0, airspeed_variation_m_s=0.0)
        data = simulate_maneuver(config, "circle")
        estimate = estimate_wind(data["ground_velocity"], data["headings"])
        np.testing.assert_allclose(estimate.wind, config.wind, atol=1e-6)

    def test_hover_is_rejected(self):
        data = simulate_maneuver(WindConfig(), "hover")
        with self.assertRaises(ValueError):
            estimate_wind(data["ground_velocity"], data["headings"])

    def test_circle_estimate_is_accurate_with_noise(self):
        errors = []
        for seed in range(10):
            data = simulate_maneuver(WindConfig(seed=seed), "circle")
            estimate = estimate_wind(data["ground_velocity"], data["headings"])
            errors.append(float(np.linalg.norm(estimate.wind - data["wind"])))
        self.assertLess(float(np.mean(errors)), 0.15)

    def test_residual_is_small_at_the_true_wind(self):
        data = simulate_maneuver(WindConfig(gps_noise_m_s=0.0, airspeed_noise_m_s=0.0,
                                            heading_noise_deg=0.0,
                                            airspeed_variation_m_s=0.0), "circle")
        residual = residual_vector(data["wind"], data["ground_velocity"], data["headings"])
        self.assertLess(float(np.max(np.abs(residual))), 1e-6)

    def test_airspeed_matches_the_pitot(self):
        data = simulate_maneuver(WindConfig(gps_noise_m_s=0.0, airspeed_noise_m_s=0.0,
                                            heading_noise_deg=0.0,
                                            airspeed_variation_m_s=0.0), "circle")
        speeds = airspeed_estimate(data["wind"], data["ground_velocity"], data["headings"])
        np.testing.assert_allclose(speeds, data["airspeed"], atol=1e-6)

    def test_reciprocal_average_cancels_the_airspeed(self):
        config = WindConfig(gps_noise_m_s=0.0, airspeed_noise_m_s=0.0,
                            heading_noise_deg=0.0, airspeed_variation_m_s=0.0)
        data = simulate_maneuver(config, "reciprocal")
        estimate = reciprocal_estimate(data["ground_velocity"], data["headings"])
        np.testing.assert_allclose(estimate, config.wind, atol=1e-6)


class SceneTests(unittest.TestCase):
    def test_maneuver_headings_have_unit_length(self):
        for maneuver in ("hover", "straight", "reciprocal", "circle"):
            headings = headings_for(maneuver)
            np.testing.assert_allclose(np.linalg.norm(headings, axis=1), 1.0, atol=1e-12)

    def test_simulation_is_deterministic(self):
        first = simulate_maneuver(WindConfig(seed=5), "circle")
        second = simulate_maneuver(WindConfig(seed=5), "circle")
        np.testing.assert_array_equal(first["ground_velocity"], second["ground_velocity"])

    def test_headless_writes_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            summary = run_experiment(out_dir=Path(directory), make_plots=False, trials=6)
            stored = json.loads(
                (Path(directory) / "summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(stored["module"], "uav_wind_estimation")
            self.assertTrue(summary["maneuvers"]["circle"]["observability"]["observable"])
            self.assertLess(summary["maneuvers"]["circle"]["ls_speed_rmse_m_s"], 0.2)


if __name__ == "__main__":
    unittest.main()
