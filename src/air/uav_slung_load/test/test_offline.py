#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线自检：模型结构、质量刷新、三种控制律的指标对比、可重复性。

不需要 ROS、不需要显示器；pytest 或 ``python3 test/test_offline.py -v`` 均可。
"""

import os
import sys
import unittest

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "scripts"))

from controllers import CONTROLLERS  # noqa: E402
from sim_core import SlungSim, run_episode, summarize  # noqa: E402

TARGET = [2.0, 0.0, 1.5]
SECONDS = 12.0
STEP_TIME = 1.0


def _metrics(controller, mass=0.3):
    sim = SlungSim(payload_mass=mass)
    log = run_episode(controller, TARGET, SECONDS, payload_mass=mass, sim=sim,
                      step_time=STEP_TIME)
    return summarize(log, settle_window=2.0, t_start=STEP_TIME)


class TestModel(unittest.TestCase):
    def test_structure(self):
        sim = SlungSim(payload_mass=0.3)
        # freejoint + 两个摆铰 = 3 个关节；6 + 2 = 8 个自由度；4 个推力执行器
        self.assertEqual(sim.model.njnt, 3)
        self.assertEqual(sim.model.nv, 8)
        self.assertEqual(sim.n_u, 4)
        self.assertAlmostEqual(float(sim.model.body_mass[sim.payload_id]), 0.3, places=6)

    def test_payload_mass_refresh(self):
        sim = SlungSim(payload_mass=0.3)
        m_small = sim.total_mass
        sim.reset(payload_mass=0.5)
        self.assertGreater(sim.total_mass, m_small)
        self.assertAlmostEqual(sim.total_mass - m_small, 0.2, places=6)
        # 惯量必须随质量同步重算（实心球 I = 0.4·m·r²），否则不同负载的对比不是同一套物理
        expected = 0.4 * 0.5 * 0.06 ** 2
        self.assertAlmostEqual(float(sim.model.body_inertia[sim.payload_id][0]),
                               expected, places=8)

    def test_swing_period(self):
        # 摆长 0.5 m，理论周期 2π√(L/g) ≈ 1.42 s
        self.assertAlmostEqual(float(np.pi * 2 * np.sqrt(0.5 / 9.81)), 1.42, delta=0.01)


class TestControllers(unittest.TestCase):
    def test_all_controllers_bounded(self):
        for name in ("pd", "zv", "swing"):
            stats = _metrics(CONTROLLERS[name]())
            for key in ("swing_rms_deg", "pos_final_m", "thrust_max_n"):
                self.assertTrue(np.isfinite(stats[key]), "%s 的 %s 非有限值" % (name, key))
            self.assertLess(stats["pos_final_m"], 0.10, "%s 末端误差过大" % name)
            self.assertLessEqual(stats["thrust_max_n"], 6.0 + 1e-6, "%s 推力超限" % name)

    def test_swing_feedback_reduces_swing(self):
        pd = _metrics(CONTROLLERS["pd"]())
        swing = _metrics(CONTROLLERS["swing"]())
        self.assertLess(swing["swing_rms_deg"], pd["swing_rms_deg"])
        self.assertLess(swing["residual_deg"], pd["residual_deg"])
        self.assertLess(swing["residual_deg"], 0.5)

    def test_zv_reduces_residual(self):
        pd = _metrics(CONTROLLERS["pd"]())
        zv = _metrics(CONTROLLERS["zv"]())
        self.assertLess(zv["residual_deg"], pd["residual_deg"])

    def test_repeatable(self):
        a = _metrics(CONTROLLERS["swing"]())
        b = _metrics(CONTROLLERS["swing"]())
        self.assertAlmostEqual(a["residual_deg"], b["residual_deg"], places=6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
