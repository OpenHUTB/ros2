"""Unit tests for the magnetometer calibration module (NumPy only)."""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from mag_hard_soft_iron_calibration.core import (
    HeadingFilter,
    fit_ellipsoid,
    fit_hard_iron,
    magnetic_heading,
    tilt_angles,
)
from mag_hard_soft_iron_calibration.headless import _heading_errors, run_experiment
from mag_hard_soft_iron_calibration.scene import (
    MagConfig,
    rotation_matrix,
    simulate_calibration,
    simulate_heading,
)


class CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = MagConfig(seed=0)
        cls.data = simulate_calibration(cls.config)
        cls.fit = fit_ellipsoid(cls.data["measured_ut"], cls.config.field_norm_ut)

    def test_sphere_fit_lands_near_the_hard_iron_offset(self):
        offset = fit_hard_iron(self.data["measured_ut"])
        self.assertLess(float(np.linalg.norm(offset - self.config.hard_iron)), 15.0)

    def test_ellipsoid_recovers_the_offset(self):
        error = float(np.linalg.norm(self.fit.offset - self.config.hard_iron))
        self.assertLess(error, 1.0)

    def test_ellipsoid_recovers_the_soft_iron_matrix(self):
        truth = np.linalg.inv(self.config.soft_matrix)
        relative = float(np.linalg.norm(self.fit.matrix - truth) / np.linalg.norm(truth))
        self.assertLess(relative, 0.05)

    def test_calibrated_norm_matches_the_field(self):
        corrected = self.fit.apply(self.data["measured_ut"])
        norms = np.linalg.norm(corrected, axis=1)
        self.assertLess(float(np.std(norms)), 1.0)
        self.assertLess(float(self.fit.residual_rms), 1.5 * self.config.noise_ut)

    def test_apply_is_identity_for_a_perfectly_calibrated_set(self):
        config = MagConfig(
            hard_iron_ut=(0.0, 0.0, 0.0),
            soft_diagonal=(1.0, 1.0, 1.0),
            soft_cross=0.0,
            noise_ut=0.0,
        )
        data = simulate_calibration(config)
        fitted = fit_ellipsoid(data["measured_ut"], config.field_norm_ut)
        np.testing.assert_allclose(fitted.offset, 0.0, atol=1e-6)
        np.testing.assert_allclose(fitted.matrix, np.eye(3), atol=1e-3)

    def test_too_few_samples_are_rejected(self):
        with self.assertRaises(ValueError):
            fit_ellipsoid(np.zeros((5, 3)))


class HeadingTests(unittest.TestCase):
    def test_tilt_angles_of_level_gravity_are_zero(self):
        roll, pitch = tilt_angles(np.array([[0.0, 0.0, 9.81]]))
        self.assertAlmostEqual(float(roll[0]), 0.0, places=9)
        self.assertAlmostEqual(float(pitch[0]), 0.0, places=9)

    def test_heading_of_a_northward_field_is_zero(self):
        heading = magnetic_heading(np.array([[48.0, 0.0, 0.0]]))
        self.assertAlmostEqual(float(heading[0]), 0.0, places=9)

    def test_heading_of_an_eastward_field_is_minus_half_pi(self):
        heading = magnetic_heading(np.array([[0.0, 48.0, 0.0]]))
        self.assertAlmostEqual(float(heading[0]), -np.pi / 2.0, places=9)

    def test_rotation_matrix_matches_the_heading_convention(self):
        yaw = np.deg2rad(30.0)
        matrix = rotation_matrix(yaw, 0.0, 0.0)[0]
        field = matrix @ np.array([48.0, 0.0, 0.0])
        self.assertAlmostEqual(float(magnetic_heading(field[None, :])[0]), -yaw, places=6)

    def test_heading_error_wraps_across_pi(self):
        result = _heading_errors(np.array([np.pi - 0.01]), np.array([-np.pi + 0.01]))
        self.assertLess(result["rmse_deg"], 2.0)
        self.assertGreater(result["rmse_deg"], 0.5)

    def test_filter_converges_to_a_constant_heading(self):
        filter_ = HeadingFilter.create(0.0, initial_sigma=np.deg2rad(20.0))
        for _ in range(50):
            filter_.predict(0.0, 0.02)
            filter_.update(1.0)
        self.assertAlmostEqual(filter_.state, 1.0, places=2)

    def test_filter_rejects_a_bad_initial_guess(self):
        filter_ = HeadingFilter.create(np.pi, initial_sigma=np.deg2rad(50.0))
        for _ in range(80):
            filter_.predict(0.0, 0.02)
            filter_.update(0.0)
        self.assertLess(abs(float(np.arctan2(np.sin(filter_.state), np.cos(filter_.state)))), 0.2)


class ExperimentTests(unittest.TestCase):
    def test_heading_simulation_is_deterministic(self):
        config = MagConfig(heading_duration_s=5.0, seed=3)
        first = simulate_heading(config)
        second = simulate_heading(config)
        np.testing.assert_array_equal(first["measured_ut"], second["measured_ut"])

    def test_headless_writes_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            summary = run_experiment(out_dir=Path(directory), make_plots=False)
            stored = json.loads(
                (Path(directory) / "summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(stored["module"], "mag_hard_soft_iron_calibration")
            self.assertLess(
                summary["heading"]["fused"]["rmse_deg"],
                summary["heading"]["raw"]["rmse_deg"],
            )
            self.assertLess(
                summary["heading"]["ellipsoid_calibrated"]["rmse_deg"],
                summary["heading"]["hard_iron_only"]["rmse_deg"],
            )


if __name__ == "__main__":
    unittest.main()
