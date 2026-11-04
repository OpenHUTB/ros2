#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""核心单元测试（不依赖 ROS master/display）。"""
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src"))

from uav_wind_aware_delivery.ballistics import (  # noqa: E402
    release_point, simulate_drop,
)
from uav_wind_aware_delivery.control import GeometricController  # noqa: E402
from uav_wind_aware_delivery.mathutil import quat_to_euler  # noqa: E402
from uav_wind_aware_delivery.mission import DeliveryMission, MissionConfig  # noqa: E402
from uav_wind_aware_delivery.nn import ImpactMLP  # noqa: E402
from uav_wind_aware_delivery.sim import DeliverySim, SimConfig  # noqa: E402
from uav_wind_aware_delivery.state import State  # noqa: E402


class TestCore(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = DeliverySim(SimConfig(wind=np.zeros(3), substeps=2))
        cls.ctrl = GeometricController()
        cls.model_path = os.path.join(ROOT, "models", "impact_mlp.npz")

    def test_01_model_mass(self):
        self.assertAlmostEqual(self.sim.uav_mass, 0.972, places=3)
        self.assertAlmostEqual(self.sim.payload_mass, 0.20, places=3)
        self.assertAlmostEqual(self.sim.mass, 1.172, places=3)

    def test_02_euler_zero(self):
        self.assertTrue(np.allclose(quat_to_euler(np.array([1.0, 0, 0, 0])), 0.0))

    def test_03_hover_stable(self):
        sim = DeliverySim(SimConfig(wind=np.zeros(3), substeps=2))
        ctrl = GeometricController()
        sp = np.array([0.0, 0.0, 1.8])
        for _ in range(300):
            ctrl.act(sim, sp, np.zeros(3))
        self.assertLess(float(np.linalg.norm(sim.state().pos - sp)), 0.10)

    def test_04_wind_hold(self):
        sim = DeliverySim(SimConfig(wind=np.array([3.0, -1.5, 0.0]), substeps=2))
        ctrl = GeometricController()
        sp = np.array([0.0, 0.0, 2.0])
        for _ in range(500):
            ctrl.act(sim, sp, np.zeros(3))
        self.assertLess(float(np.linalg.norm(sim.state().pos[:2] - sp[:2])), 0.20)

    def test_05_release_once(self):
        sim = DeliverySim(SimConfig(wind=np.zeros(3), substeps=2))
        self.assertFalse(sim.released)
        self.assertTrue(sim.release())
        self.assertTrue(sim.released)
        self.assertFalse(sim.release())

    def test_06_payload_drop(self):
        sim = DeliverySim(SimConfig(wind=np.zeros(3), substeps=2))
        sim.release()
        for _ in range(250):
            sim.step(sim.hover_force(), np.zeros(3))
        st = sim.state()
        self.assertLess(st.payload_pos[2], 0.15)
        self.assertTrue(st.landed)

    def test_07_mass_drop(self):
        sim = DeliverySim(SimConfig(wind=np.zeros(3), substeps=2))
        before = sim.mass
        sim.release()
        self.assertAlmostEqual(before - sim.mass, 0.20, places=3)

    def test_08_zero_wind_drop(self):
        off = simulate_drop(3.0, 0, 0, 0, np.zeros(2))
        self.assertLess(float(np.linalg.norm(off)), 0.02)

    def test_09_wind_drift_sign(self):
        off = simulate_drop(3.0, 0, 0, 0, np.array([4.0, 0.0]))
        self.assertGreater(float(off[0]), 0.5)

    def test_10_release_point(self):
        target = np.array([5.0, 2.0])
        wind = np.array([3.0, 1.0])
        rel = release_point(target, 3.0, wind)
        landing = rel + simulate_drop(3.0, 0, 0, 0, wind)
        self.assertLess(float(np.linalg.norm(landing - target)), 0.06)

    def test_11_mlp_exists(self):
        self.assertTrue(os.path.exists(self.model_path))

    def test_12_mlp_predict(self):
        model = ImpactMLP.load(self.model_path)
        p = model.predict(np.array([3.0, 0, 0, 0, 3.0, 1.0]))
        self.assertEqual(p.shape, (2,))
        self.assertTrue(np.all(np.isfinite(p)))

    def test_13_mlp_accuracy(self):
        model = ImpactMLP.load(self.model_path)
        X = np.array([
            [3.0, 0, 0, 0, 3.0, 1.0],
            [3.0, 0, 0, 0, 0.0, 0.0],
            [2.5, 0.2, -0.1, 0, -2.0, 1.0],
        ])
        y = np.array([simulate_drop(3.0, 0, 0, 0, np.array([3.0, 1.0])),
                      simulate_drop(3.0, 0, 0, 0, np.zeros(2)),
                      simulate_drop(2.5, 0.2, -0.1, 0, np.array([-2.0, 1.0]))])
        rmse = float(np.sqrt(np.mean((model.predict(X) - y) ** 2)))
        self.assertLess(rmse, 0.05)

    def test_14_state_vector_roundtrip(self):
        st = self.sim.state()
        st2 = State.from_vector(st.as_vector())
        self.assertAlmostEqual(st.t, st2.t)
        self.assertTrue(np.allclose(st.pos, st2.pos))
        self.assertTrue(np.allclose(st.quat, st2.quat))
        self.assertEqual(st.released, st2.released)

    def test_15_mission_release_geometry(self):
        m = DeliveryMission(MissionConfig(target=np.array([5.0, 2.0, 0.0])),
                            model_path=self.model_path)
        rel = m.compute_release_point(np.array([3.0, 1.0, 0.0]))
        self.assertLess(rel[0], 5.0)
        self.assertLess(rel[1], 2.0)
        self.assertGreater(float(np.linalg.norm(rel - np.array([5.0, 2.0]))), 0.5)


if __name__ == "__main__":
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(TestCore)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    print(f"\n{result.testsRun} tests, failures={len(result.failures)}, errors={len(result.errors)}")
    sys.exit(0 if result.wasSuccessful() else 1)
