# -*- coding: utf-8 -*-
"""悬停可行域计算的单元测试（纯 numpy，不需要 ROS）。

    python -m unittest discover -s test -p "test_*.py" -v
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from uav_rotor_failure_env.core import (G, max_hover, max_hover_zero_yaw,  # noqa: E402
                                        hexa, nominal_mass_limit, octo,
                                        quad_x, spin_rate_for, wrench_rank)


def moments(cfg, f):
    pos = cfg.positions()
    return (float(np.dot(pos[:, 1], f)), float(np.dot(-pos[:, 0], f)))


class TestNominal(unittest.TestCase):
    def test_all_full_thrust(self):
        """正常状态下，所有桨满推力即可零力矩悬停。"""
        for cfg in (quad_x(), hexa(), octo()):
            r = max_hover(cfg)
            self.assertAlmostEqual(r["margin"], 1.0, places=9)
            self.assertTrue(r["yaw_free"])
            np.testing.assert_allclose(r["thrusts"], cfg.thrust_max, atol=1e-9)

    def test_mass_limit(self):
        cfg = quad_x()
        self.assertAlmostEqual(max_hover(cfg)["mass_limit"],
                               cfg.nominal_thrust() / G, places=9)
        self.assertAlmostEqual(nominal_mass_limit(cfg), cfg.nominal_thrust() / G)

    def test_moments_are_zero(self):
        """解出来的桨推力必须满足 Mx = My = 0。"""
        for cfg in (quad_x(), hexa(), octo()):
            r = max_hover(cfg, {0})
            mx, my = moments(cfg, np.array(r["thrusts"]))
            self.assertAlmostEqual(mx, 0.0, places=9)
            self.assertAlmostEqual(my, 0.0, places=9)


class TestSingleFailure(unittest.TestCase):
    def test_quad_halves(self):
        """四旋翼单桨失效：只剩一对对角桨，推力掉到 50%。"""
        cfg = quad_x()
        for k in range(4):
            r = max_hover(cfg, {k})
            self.assertAlmostEqual(r["margin"], 0.5, places=6)
            self.assertFalse(r["yaw_free"], "四旋翼单桨失效必须自旋才能零偏航")

    def test_quad_zero_yaw_infeasible(self):
        """四旋翼单桨失效后，不允许自旋就一点推力都没有（经典结论）。"""
        cfg = quad_x()
        for k in range(4):
            z = max_hover_zero_yaw(cfg, {k})
            self.assertAlmostEqual(z["margin"], 0.0, places=6)

    def test_hexa_third_lost(self):
        """六旋翼单桨失效：4/6 = 66.7%，且可以不转。"""
        cfg = hexa()
        r = max_hover(cfg, {0})
        self.assertAlmostEqual(r["margin"], 4.0 / 6.0, places=6)
        self.assertTrue(r["yaw_free"])
        self.assertAlmostEqual(max_hover_zero_yaw(cfg, {0})["margin"], 4.0 / 6.0, places=6)

    def test_octo_quarter_lost(self):
        """八旋翼单桨失效：6/8 = 75%（可自旋）；零偏航略低。"""
        cfg = octo()
        r = max_hover(cfg, {0})
        self.assertAlmostEqual(r["margin"], 6.0 / 8.0, places=6)
        z = max_hover_zero_yaw(cfg, {0})
        self.assertLess(z["margin"], r["margin"])
        self.assertGreater(z["margin"], 0.6)

    def test_two_failures_quad_dead(self):
        """四旋翼掉两个桨（相邻）后无法零力矩悬停。"""
        cfg = quad_x()
        self.assertAlmostEqual(max_hover(cfg, {0, 1})["margin"], 0.0, places=6)

    def test_bounds_respected(self):
        cfg = hexa()
        f = np.array(max_hover(cfg, {2})["thrusts"])
        self.assertTrue(np.all(f >= -1e-9))
        self.assertTrue(np.all(f <= cfg.thrust_max + 1e-9))
        self.assertAlmostEqual(f[2], 0.0, places=9)

    def test_bad_index_raises(self):
        with self.assertRaises(ValueError):
            max_hover(quad_x(), {9})


class TestHelpers(unittest.TestCase):
    def test_spin_rate(self):
        self.assertAlmostEqual(spin_rate_for(0.4, 0.004), 10.0, places=6)
        self.assertGreater(spin_rate_for(0.8, 0.004), spin_rate_for(0.4, 0.004))
        with self.assertRaises(ValueError):
            spin_rate_for(0.4, 0.0)

    def test_rank_drops_for_quad(self):
        cfg = quad_x()
        self.assertEqual(wrench_rank(cfg), 4)
        self.assertLess(wrench_rank(cfg, {0}), 4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
