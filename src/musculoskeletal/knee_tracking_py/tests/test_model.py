"""刚体 + 拮抗肌模型测试。"""

from __future__ import annotations

import math
import unittest

import numpy as np

from src.config import JointConfig, MuscleConfig
from src.model import KneeModel, KneeState


def make_model() -> KneeModel:
    return KneeModel(
        JointConfig(
            inertia=0.06,
            damping=0.5,
            gravity_torque=4.5,
            angle_min_deg=-15.0,
            angle_max_deg=90.0,
        ),
        MuscleConfig(
            max_force=500.0,
            moment_arm=0.035,
            tau_act=0.02,
            tau_deact=0.05,
            force_length_strength=0.0,
            optimal_angle_deg=30.0,
        ),
    )


class TestModelBasics(unittest.TestCase):
    def setUp(self) -> None:
        self.model = make_model()

    def test_max_muscle_torque(self) -> None:
        self.assertAlmostEqual(self.model.max_muscle_torque, 17.5, places=12)

    def test_gravity_zero_at_full_extension(self) -> None:
        self.assertAlmostEqual(self.model.gravity_torque(0.0), 0.0, places=12)

    def test_gravity_opposes_flexion(self) -> None:
        tau = self.model.gravity_torque(math.radians(50.0))
        self.assertAlmostEqual(tau, -4.5 * math.sin(math.radians(50.0)), places=12)
        self.assertAlmostEqual(tau, -3.447, places=3)

    def test_gravity_assists_flexion_when_hyperextended(self) -> None:
        self.assertGreater(self.model.gravity_torque(math.radians(-10.0)), 0.0)

    def test_flexor_torque_positive(self) -> None:
        state = KneeState(math.radians(30.0), 0.0, 0.5, 0.0)
        self.assertAlmostEqual(self.model.muscle_torque(state), 8.75, places=12)

    def test_extensor_torque_negative(self) -> None:
        state = KneeState(math.radians(30.0), 0.0, 0.0, 0.5)
        self.assertAlmostEqual(self.model.muscle_torque(state), -8.75, places=12)

    def test_equal_activation_cancels(self) -> None:
        state = KneeState(0.0, 0.0, 0.4, 0.4)
        self.assertAlmostEqual(self.model.muscle_torque(state), 0.0, places=12)

    def test_force_length_disabled_is_one(self) -> None:
        self.assertEqual(self.model.force_length_factor(0.0), 1.0)
        self.assertEqual(self.model.force_length_factor(math.radians(70.0)), 1.0)


class TestActivationDynamics(unittest.TestCase):
    def setUp(self) -> None:
        self.model = make_model()

    def test_activation_rises_with_tau_act(self) -> None:
        state = KneeState(0.0, 0.0, 0.0, 0.0)
        new = self.model.step(state, 1.0, 0.0, 0.02)
        self.assertGreater(new.flex, 0.60)
        self.assertLess(new.flex, 0.66)
        self.assertGreater(new.flex, 0.0)
        self.assertLessEqual(new.flex, 1.0)

    def test_deactivation_is_slower(self) -> None:
        state = KneeState(0.0, 0.0, 1.0, 0.0)
        after_short = self.model.step(state, 0.0, 0.0, 0.02)
        self.assertGreater(after_short.flex, 0.65)
        after_long = self.model.step(state, 0.0, 0.0, 0.1)
        self.assertLess(after_long.flex, 0.2)

    def test_activation_stays_in_unit_interval(self) -> None:
        state = KneeState(0.0, 0.0, 0.0, 0.0)
        for _ in range(500):
            state = self.model.step(state, 1.0, 0.0, 0.005)
        self.assertLessEqual(state.flex, 1.0)
        self.assertGreaterEqual(state.flex, 0.0)


class TestLimitsAndStability(unittest.TestCase):
    def setUp(self) -> None:
        self.model = make_model()

    def test_upper_limit_clamped(self) -> None:
        state = KneeState(math.radians(89.9), 5.0, 1.0, 0.0)
        new = self.model.step(state, 1.0, 0.0, 0.005)
        self.assertLessEqual(new.theta, self.model.angle_max + 1e-12)
        self.assertAlmostEqual(new.theta, self.model.angle_max, places=9)
        self.assertEqual(new.omega, 0.0)

    def test_lower_limit_clamped(self) -> None:
        state = KneeState(math.radians(-14.9), -5.0, 0.0, 1.0)
        new = self.model.step(state, 0.0, 1.0, 0.005)
        self.assertGreaterEqual(new.theta, self.model.angle_min - 1e-12)
        self.assertAlmostEqual(new.theta, self.model.angle_min, places=9)
        self.assertEqual(new.omega, 0.0)

    def test_no_nan_in_long_run(self) -> None:
        state = KneeState(0.0, 0.0, 0.0, 0.0)
        for _ in range(2000):
            state = self.model.step(state, 1.0, 0.0, 0.005)
            self.assertTrue(math.isfinite(state.theta))
            self.assertTrue(math.isfinite(state.omega))
            self.assertTrue(math.isfinite(state.flex))
        self.assertLessEqual(state.theta, self.model.angle_max + 1e-12)

    def test_damping_dissipates_energy(self) -> None:
        state = KneeState(math.radians(30.0), 2.0, 0.0, 0.0)
        energies = []
        for _ in range(200):
            energies.append(0.5 * self.model.joint.inertia * state.omega**2)
            state = self.model.step(state, 0.0, 0.0, 0.005)
        self.assertLess(state.omega, 2.0)
        self.assertLess(energies[-1], energies[0])

    def test_invalid_dt_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.model.step(KneeState(0.0, 0.0, 0.0, 0.0), 0.0, 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
