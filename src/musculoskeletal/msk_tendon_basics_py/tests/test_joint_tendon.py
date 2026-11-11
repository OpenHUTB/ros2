"""拮抗关节几何、静力学与松手释放测试。"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.joint_tendon import (  # noqa: E402
    AntagonistJoint,
    JointParams,
    ReleaseResult,
)
from scripts.muscle_tendon import MuscleTendonParams  # noqa: E402


class TestJointGeometryAndStatics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p = MuscleTendonParams()
        cls.jp = JointParams.from_muscle_params(cls.p)
        cls.joint = AntagonistJoint(cls.p, cls.jp)

    def test_mtu_lengths_neutral(self):
        lf, le = self.joint.mtu_lengths(0.0)
        self.assertAlmostEqual(lf, self.jp.rest_mtu, places=12)
        self.assertAlmostEqual(le, self.jp.rest_mtu, places=12)

    def test_mtu_lengths_move_oppositely(self):
        theta = 0.2
        lf, le = self.joint.mtu_lengths(theta)
        self.assertAlmostEqual(lf, self.jp.rest_mtu - self.jp.moment_arm * theta, places=12)
        self.assertAlmostEqual(le, self.jp.rest_mtu + self.jp.moment_arm * theta, places=12)
        self.assertLess(lf, le)

    def test_flexor_activation_gives_positive_torque(self):
        torque = self.joint.torque(0.0, 0.0, 0.9, 0.05)
        self.assertGreater(torque, 0.0)

    def test_load_spring_has_zero_torque_at_rest_angle(self):
        torque = self.joint.torque(self.jp.load_rest_angle, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(torque, 0.0, places=12)

    def test_damping_opposes_motion(self):
        positive = self.joint.torque(0.05, 1.0, 0.0, 0.0)
        self.assertLess(positive, 0.0)


class TestReleaseExperiment(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p = MuscleTendonParams()
        cls.jp = JointParams.from_muscle_params(cls.p)
        cls.elastic = AntagonistJoint(cls.p, cls.jp).simulate_release(
            hold_time=0.4, duration=1.0, dt=2e-3,
            flex_command=0.9, ext_command=0.05,
        )
        cls.rigid = AntagonistJoint(
            cls.p, cls.jp, tendon_stiffness=math.inf
        ).simulate_release(
            hold_time=0.4, duration=1.0, dt=2e-3,
            flex_command=0.9, ext_command=0.05,
        )

    def test_result_type_and_length(self):
        self.assertIsInstance(self.elastic, ReleaseResult)
        self.assertGreater(len(self.elastic), 100)
        self.assertEqual(len(self.elastic), len(self.rigid))

    def test_hold_angle_is_constant(self):
        hold = self.elastic.phase == "hold"
        self.assertTrue(np.allclose(self.elastic.theta[hold], self.jp.release_angle))
        self.assertTrue(np.allclose(self.elastic.omega[hold], 0.0))

    def test_activation_reaches_commands(self):
        hold = self.elastic.phase == "hold"
        self.assertAlmostEqual(self.elastic.flex_activation[hold][-1], 0.9, places=9)
        self.assertAlmostEqual(self.elastic.ext_activation[hold][-1], 0.05, places=9)

    def test_pre_release_metrics(self):
        self.assertAlmostEqual(self.elastic.pre_force, 89.5, delta=0.2)
        self.assertAlmostEqual(self.elastic.pre_energy, 0.134, delta=0.0015)

    def test_release_peak_angle(self):
        self.assertAlmostEqual(self.elastic.peak_angle, 0.58, delta=0.01)
        self.assertGreater(self.elastic.peak_time, 0.4)

    def test_rigid_release_peak_angle(self):
        self.assertAlmostEqual(self.rigid.peak_angle, 0.60, delta=0.01)

    def test_elastic_peak_is_below_rigid(self):
        self.assertLess(self.elastic.peak_angle, self.rigid.peak_angle)

    def test_energy_drops_after_release(self):
        self.assertLess(self.elastic.min_energy, self.elastic.pre_energy)
        self.assertAlmostEqual(self.elastic.min_energy, 0.061, delta=0.002)

    def test_release_fraction(self):
        self.assertAlmostEqual(self.elastic.release_fraction, 0.54, delta=0.02)
        self.assertGreater(self.elastic.release_fraction, 0.0)
        self.assertLess(self.elastic.release_fraction, 1.0)

    def test_rigid_tendon_has_no_energy(self):
        self.assertTrue(np.allclose(self.rigid.flex_energy, 0.0))
        self.assertEqual(self.rigid.pre_energy, 0.0)

    def test_no_nan_in_release(self):
        for arr in (
            self.elastic.theta,
            self.elastic.omega,
            self.elastic.flex_force,
            self.elastic.ext_force,
            self.elastic.flex_energy,
        ):
            self.assertTrue(np.all(np.isfinite(arr)))

    def test_invalid_time_arguments(self):
        joint = AntagonistJoint(self.p, self.jp)
        with self.assertRaises(ValueError):
            joint.simulate_release(dt=0.0)
        with self.assertRaises(ValueError):
            joint.simulate_release(duration=0.0)


if __name__ == "__main__":
    unittest.main()
