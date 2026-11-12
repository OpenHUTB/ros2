"""Unit tests for the joint-friction module (NumPy only)."""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from joint_friction_identification.core import (
    FrictionModel,
    identify_friction,
    sweep_torque,
    torque_error,
)
from joint_friction_identification.headless import run_experiment
from joint_friction_identification.scene import JointConfig, simulate_sweeps, simulate_tracking

TRUTH = FrictionModel(coulomb=0.45, viscous=0.08, static=0.65, stribeck_velocity=0.20)


class ModelTests(unittest.TestCase):
    def test_torque_is_odd_in_velocity(self):
        model = FrictionModel()
        speeds = np.array([0.1, 0.5, 1.0])
        np.testing.assert_allclose(model.torque(-speeds), -model.torque(speeds), atol=1e-12)

    def test_zero_velocity_has_zero_friction(self):
        self.assertAlmostEqual(float(FrictionModel().torque(0.0)), 0.0, places=12)

    def test_static_must_not_be_below_coulomb(self):
        with self.assertRaises(ValueError):
            FrictionModel(coulomb=0.5, static=0.4).validate()

    def test_negative_coefficients_are_rejected(self):
        with self.assertRaises(ValueError):
            FrictionModel(coulomb=-0.1).validate()


class IdentificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = JointConfig(torque_noise=0.0)
        cls.config = config
        cls.sweeps = simulate_sweeps(config, TRUTH)
        cls.model, cls.info = identify_friction(cls.sweeps["speeds"], cls.sweeps["torques"])

    def test_coulomb_and_static_are_accurate(self):
        self.assertLess(abs(self.model.coulomb - TRUTH.coulomb) / TRUTH.coulomb, 0.05)
        self.assertLess(abs(self.model.static - TRUTH.static) / TRUTH.static, 0.05)

    def test_viscous_and_stribeck_are_reasonable(self):
        self.assertLess(abs(self.model.viscous - TRUTH.viscous) / TRUTH.viscous, 0.25)
        self.assertLess(
            abs(self.model.stribeck_velocity - TRUTH.stribeck_velocity)
            / TRUTH.stribeck_velocity,
            0.25,
        )

    def test_residual_is_near_zero_without_noise(self):
        self.assertLess(self.info["residual_rms"], 1e-3)

    def test_too_few_points_are_rejected(self):
        with self.assertRaises(ValueError):
            identify_friction(np.array([0.1, 0.2]), np.array([1.0, 1.1]))

    def test_zero_speed_is_rejected(self):
        with self.assertRaises(ValueError):
            identify_friction(np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5]),
                              np.zeros(6))

    def test_error_report_is_relative(self):
        errors = torque_error(TRUTH, TRUTH)
        for value in errors.values():
            self.assertAlmostEqual(value, 0.0, places=9)


class TrackingTests(unittest.TestCase):
    def test_feed_forward_reduces_tracking_error(self):
        config = JointConfig(sine_duration_s=3.0)
        model, _ = identify_friction(
            *simulate_sweeps(config, TRUTH).values()
        )
        baseline = simulate_tracking(config, TRUTH)
        compensated = simulate_tracking(config, TRUTH, model=model)
        self.assertLess(compensated["rms_error_rad"], baseline["rms_error_rad"])

    def test_simulation_is_deterministic(self):
        config = JointConfig(sine_duration_s=1.0)
        first = simulate_tracking(config, TRUTH)
        second = simulate_tracking(config, TRUTH)
        np.testing.assert_array_equal(first["position"], second["position"])


class ExperimentTests(unittest.TestCase):
    def test_headless_writes_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            summary = run_experiment(out_dir=Path(directory), make_plots=False)
            stored = json.loads(
                (Path(directory) / "summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(stored["module"], "joint_friction_identification")
            self.assertGreater(summary["tracking"]["improvement_pct"], 0.0)
            self.assertLess(abs(summary["identification"]["error_pct"]["coulomb_pct"]), 10.0)


if __name__ == "__main__":
    unittest.main()
