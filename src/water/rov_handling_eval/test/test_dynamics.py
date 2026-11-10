# -*- coding: utf-8 -*-
"""水动力与评测指标的单测（不需要 ROS / MuJoCo，numpy 即可）。

    python3 -m unittest discover -s test -p "test_*.py" -v
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from rov_handling_eval.dynamics import (G, VehicleParams, buoyancy_wrench,  # noqa: E402
                                        current_authority, drag_force,
                                        drag_torque, velocity_control)


def rotx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


class TestStatics(unittest.TestCase):
    def setUp(self):
        self.p = VehicleParams()

    def test_net_buoyancy(self):
        """默认参数下应约有 +2.7 N 正浮力（≈ +0.27 kg）。"""
        self.assertAlmostEqual(self.p.net_buoyancy(), 2.70, delta=0.05)

    def test_terminal_speed_matches_formula(self):
        vt = self.p.terminal_speed(2)
        self.assertAlmostEqual(vt, 0.206, delta=0.005)

    def test_ballast_balances(self):
        """加 +0.27 kg 配重后净浮力接近 0。"""
        self.p.ballast = 0.27
        self.assertLess(abs(self.p.net_buoyancy()), 0.1)

    def test_ballast_sign(self):
        """配重越大越沉：净浮力单调递减。"""
        base = self.p.net_buoyancy()
        self.p.ballast = 1.0
        self.assertLess(self.p.net_buoyancy(), base)


class TestDrag(unittest.TestCase):
    def test_direction(self):
        f = drag_force(np.array([1.0, 0.0, 0.0]), np.array([10.0, 20.0, 30.0]))
        self.assertLess(f[0], 0.0)
        self.assertAlmostEqual(f[1], 0.0)
        self.assertAlmostEqual(f[2], 0.0)

    def test_quadratic(self):
        """阻力随速度平方增长：v 翻倍 → 力变 4 倍。"""
        k = np.array([10.0, 10.0, 10.0])
        f1 = abs(drag_force(np.array([1.0, 0, 0]), k)[0])
        f2 = abs(drag_force(np.array([2.0, 0, 0]), k)[0])
        self.assertAlmostEqual(f2 / f1, 4.0, places=9)

    def test_anisotropy(self):
        """同速下升沉阻力应大于纵荡（迎水面积不同）。"""
        k = VehicleParams().k()
        self.assertGreater(k[2], k[0])
        self.assertGreater(k[1], k[0])

    def test_torque_sign(self):
        tau = drag_torque(np.array([0.5, 0.0, 0.0]), (1.0, 1.0, 1.0))
        self.assertLess(tau[0], 0.0)


class TestBuoyancyWrench(unittest.TestCase):
    def test_no_torque_when_upright(self):
        """浮心在正上方且不倾斜时，恢复力矩为 0。"""
        _, tau = buoyancy_wrench(VehicleParams(), np.eye(3))
        np.testing.assert_allclose(tau, 0.0, atol=1e-12)

    def test_restoring_torque_when_tilted(self):
        """绕 x 倾斜后，浮力应产生反向（恢复）力矩。"""
        p = VehicleParams()
        R = rotx(np.radians(10.0))
        _, tau = buoyancy_wrench(p, R)
        self.assertLess(tau[0], 0.0, "倾斜 +10° 时应出现负向恢复力矩")
        self.assertAlmostEqual(abs(tau[0]), p.buoyancy() * 0.03 * np.sin(np.radians(10.0)),
                               places=6)


class TestControl(unittest.TestCase):
    def test_clip(self):
        p = VehicleParams()
        f, tau = velocity_control(np.full(6, 99.0), np.zeros(3), np.zeros(3), p)
        np.testing.assert_allclose(f, p.max_force)
        np.testing.assert_allclose(tau, p.max_torque)

    def test_zero_error_zero_force(self):
        p = VehicleParams()
        cmd = np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0])
        f, _ = velocity_control(cmd, cmd[:3], np.zeros(3), p)
        np.testing.assert_allclose(f, 0.0, atol=1e-12)


class TestAuthority(unittest.TestCase):
    def test_still_water(self):
        eta = current_authority(VehicleParams(), 0.0)
        np.testing.assert_allclose(eta, 1.0)

    def test_documented_values(self):
        """文档表 6.3 的 η：0.5 m/s 横流时 η_y≈0.11。"""
        eta = current_authority(VehicleParams(), 0.5)
        self.assertAlmostEqual(eta[1], 0.108, delta=0.005)
        self.assertAlmostEqual(eta[0], 0.670, delta=0.005)
        self.assertAlmostEqual(eta[2], 0.0, delta=1e-6)   # 升沉已无力可用

    def test_monotonic(self):
        prev = None
        for v in (0.0, 0.1, 0.2, 0.3, 0.5):
            eta = current_authority(VehicleParams(), v)
            if prev is not None:
                self.assertTrue(np.all(eta <= prev + 1e-12))
            prev = eta


class TestParams(unittest.TestCase):
    def test_g(self):
        self.assertAlmostEqual(G, 9.81)


if __name__ == "__main__":
    unittest.main(verbosity=2)
