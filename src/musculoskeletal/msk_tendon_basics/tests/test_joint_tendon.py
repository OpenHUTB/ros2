# -*- coding: utf-8 -*-
"""关节与释放实验测试。"""

from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from experiments import (  # noqa: E402
    isometric_sweep,
    release_experiment,
    release_pair,
    tendon_stiffness_sweep,
)
from joint_tendon import JointParams, TendonJoint, static_torque  # noqa: E402
from muscle_tendon import TendonParams  # noqa: E402


class StaticJointTest(unittest.TestCase):
    def setUp(self):
        self.jp = JointParams()
        self.mp = TendonParams()

    def test_symmetric_activation_gives_zero_torque(self):
        tau = static_torque(self.jp, 0.0, 0.3, 0.3, self.mp)
        self.assertAlmostEqual(tau, 0.0, places=9)

    def test_flexor_pulls_positive(self):
        tau = static_torque(self.jp, 0.0, 0.8, 0.1, self.mp)
        self.assertGreater(tau, 0.0)

    def test_no_activation_no_torque(self):
        tau = static_torque(self.jp, 0.0, 0.0, 0.0, self.mp)
        self.assertAlmostEqual(tau, 0.0, places=9)


class JointStepTest(unittest.TestCase):
    def test_activation_bounded_and_angle_moves(self):
        joint = TendonJoint(JointParams(), TendonParams())
        for _ in range(200):
            state = joint.step(0.8, 0.05, 1e-3)
        self.assertGreater(state["theta"], 0.0)
        self.assertGreaterEqual(state["a_flex"], 0.0)
        self.assertLessEqual(state["a_flex"], 1.0)

    def test_invalid_dt(self):
        joint = TendonJoint()
        with self.assertRaises(ValueError):
            joint.step(0.5, 0.5, 0.0)


class SweepTest(unittest.TestCase):
    def test_isometric_series_lengths(self):
        result = isometric_sweep(n=6)
        self.assertEqual(len(result["acts"]), 6)
        self.assertEqual(len(result["compliant"]["force"]), 6)
        self.assertLess(result["compliant"]["force"][-1],
                        result["rigid"]["force"][-1])

    def test_stiffness_tradeoff_is_monotonic(self):
        result = tendon_stiffness_sweep()
        for prev, cur in zip(result["force"], result["force"][1:]):
            self.assertLess(prev, cur)
        for prev, cur in zip(result["energy"], result["energy"][1:]):
            self.assertGreater(prev, cur)


class ReleaseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mp = TendonParams()
        cls.jp = JointParams()
        cls.runs = release_pair(cls.mp, cls.jp, hold_time=0.3,
                                duration=0.9, dt=2e-3)

    def test_energy_is_stored_at_hold(self):
        m = self.runs["compliant"]["metrics"]
        self.assertGreater(m["hold_energy"], 0.01)

    def test_energy_drops_after_release(self):
        m = self.runs["compliant"]["metrics"]
        self.assertLess(m["min_energy_after_release"], m["hold_energy"])

    def test_joint_moves(self):
        h = self.runs["compliant"]["history"]
        self.assertGreater(max(h["theta"]), h["theta"][0])

    def test_rigid_has_no_tendon_energy(self):
        m = self.runs["rigid"]["metrics"]
        self.assertLess(m["hold_energy"], 1e-6)

    def test_duration_validation(self):
        with self.assertRaises(ValueError):
            release_experiment(self.mp, self.jp, duration=0.3, hold_time=0.3)


if __name__ == "__main__":
    unittest.main()
