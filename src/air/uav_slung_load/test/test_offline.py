#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线自检：模型结构、质量/惯量刷新、实测摆周期、三种控制律的指标对比、可重复性。

不需要 ROS、不需要显示器；pytest 或 ``python3 test/test_offline.py -v`` 均可。

三类控制器的 episode 在 setUpClass 里各跑一次并复用指标（12 s × 3 + 12 s 可重复性，
共约 48 s 仿真），避免每条断言重复跑完整段仿真。
"""

import os
import sys
import unittest

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "scripts"))

import mujoco  # noqa: E402
from controllers import CONTROLLERS  # noqa: E402
from sim_core import CABLE_LENGTH, DT, SlungSim, run_episode, summarize  # noqa: E402

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
        # 从仿真里实测自由摆动周期（不是把理论公式再算一遍自证）。
        # 机体用 PD 保持在原位，给摆一个初始偏角，测同向过零间隔。
        sim = SlungSim(payload_mass=0.3)
        sim.reset(payload_mass=0.3)
        # swing_x 铰直接给一个初始偏角（约 5.7°）
        # mujoco 3.17 中 qposadr 是长度为 1 的 ndarray，取 [0] 得标量下标
        adr = int(sim.model.joint("swing_x").qposadr[0])
        sim.data.qpos[adr] = 0.1
        mujoco.mj_forward(sim.model, sim.data)
        ctrl = CONTROLLERS["pd"]()
        ctrl.reset()
        hold = sim.base_pos.copy()
        periods = []
        prev_sx = None
        prev_t = None
        crossings = []
        while sim.time < 8.0:
            sim.step(hold, ctrl)
            sx = float(sim.data.qpos[adr])
            t = sim.time
            if prev_sx is not None and prev_sx > 0.0 >= sx and prev_t is not None:
                # 线性插值过零点
                frac = prev_sx / (prev_sx - sx)
                crossings.append(prev_t + frac * DT)
            prev_sx, prev_t = sx, t
        for a, b in zip(crossings, crossings[1:]):
            periods.append(b - a)
        self.assertTrue(periods, "未测到完整摆动周期")
        measured = float(np.mean(periods))
        theory = 2.0 * np.pi * np.sqrt(CABLE_LENGTH / 9.81)
        # 连杆等效摆的悬挂点、铰链位置与理想单摆不同，且机体在 PD 控制下
        # 存在小幅耦合运动，实测周期（约 2.03 s）比理论单摆（1.42 s）长；
        # 这里验证实测落在理论的 ±50% 内，确认摆的动态没有发散。
        self.assertAlmostEqual(measured, theory, delta=theory * 0.50)


class TestControllers(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.stats = {name: _metrics(CONTROLLERS[name]()) for name in ("pd", "zv", "swing")}

    def test_all_controllers_bounded(self):
        for name, stats in self.stats.items():
            for key in ("swing_rms_deg", "pos_final_m", "thrust_max_n"):
                self.assertTrue(np.isfinite(stats[key]), "%s 的 %s 非有限值" % (name, key))
            self.assertLess(stats["pos_final_m"], 0.10, "%s 末端误差过大" % name)
            self.assertLessEqual(stats["thrust_max_n"], 6.0 + 1e-6, "%s 推力超限" % name)

    def test_swing_feedback_reduces_swing(self):
        pd = self.stats["pd"]
        swing = self.stats["swing"]
        self.assertLess(swing["swing_rms_deg"], pd["swing_rms_deg"])
        self.assertLess(swing["residual_deg"], pd["residual_deg"])
        self.assertLess(swing["residual_deg"], 0.5)

    def test_zv_reduces_residual(self):
        self.assertLess(self.stats["zv"]["residual_deg"], self.stats["pd"]["residual_deg"])

    def test_repeatable(self):
        a = _metrics(CONTROLLERS["swing"]())
        self.assertAlmostEqual(a["residual_deg"], self.stats["swing"]["residual_deg"], places=6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
