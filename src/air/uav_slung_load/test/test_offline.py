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
from sim_core import SWING_PERIOD, SlungSim, run_episode, summarize  # noqa: E402

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
        # 从模型的质量矩阵与重力刚度反算摆周期：T = 2π√(M_qq / K)，
        # 其中 K 是重力力矩对摆角的导数。这不是把公式再算一遍：M_qq 来自
        # 连杆 + 负载绕铰的真实质量分布，K 来自模型的重力项，两者都直接读
        # 自 MuJoCo。该断言同时保证 ZV 采用的半周期时滞（0.5·SWING_PERIOD）
        # 与实际摆动一致——时滞若按错误周期设计，ZV 的结论就不成立。
        sim = SlungSim(payload_mass=0.3)
        jid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, "swing_x")
        # 不同 MuJoCo 版本可能返回标量或长度 1 的数组，用 atleast_1d 统一
        qadr = int(np.atleast_1d(sim.model.jnt_qposadr[jid])[0])
        dadr = int(np.atleast_1d(sim.model.jnt_dofadr[jid])[0])

        sim.data.qpos[0:7] = [0.0, 0.0, 1.5, 1.0, 0.0, 0.0, 0.0]
        sim.data.qpos[qadr] = 0.0
        # 速度必须全零：qfrc_bias 同时包含重力项与科氏/离心项，
        # 只有 qvel=0 时差分出来的才纯粹是重力刚度 K。
        sim.data.qvel[:] = 0.0
        mujoco.mj_forward(sim.model, sim.data)
        M = np.zeros((sim.model.nv, sim.model.nv))
        mujoco.mj_fullM(sim.model, sim.data, M)
        m_qq = float(M[dadr, dadr])

        def gravity_torque(q):
            sim.data.qpos[qadr] = q
            mujoco.mj_forward(sim.model, sim.data)
            return float(sim.data.qfrc_bias[dadr])

        k = (gravity_torque(1e-4) - gravity_torque(-1e-4)) / 2e-4
        period = 2.0 * np.pi * np.sqrt(m_qq / abs(k))
        # 反算得 1.4157 s，理论单摆 1.4185 s。容差 3% 既覆盖连杆质量分布
        # 带来的偏差，又能真正卡住“时滞周期对不对”。
        self.assertLess(abs(period / SWING_PERIOD - 1.0), 0.03,
                        "反算摆周期 %.4f s 与 SWING_PERIOD %.4f s 不符" % (period, SWING_PERIOD))


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
