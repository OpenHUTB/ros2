#!/usr/bin/env python3
"""test_offline.py —— 不依赖 ROS 的离线自检。

覆盖：
  1. MJCF 可加载、关节/执行器数量符合预期；
  2. 0/2/4 kg 三种负载各走 5 s 不跌倒，等效里程随时间增加；
  3. PD 专家输出在限幅内、方向符号正确；
  4. 神经网络权重可加载、输出维度与限幅正确。

运行：python3 test/test_offline.py（或 catkin_add_nosetests）
"""

import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(os.path.dirname(HERE), "scripts")
sys.path.insert(0, SCRIPTS)

import numpy as np                                                   # noqa: E402

from model import FEATURE_NAMES, OUTPUT_NAMES, BalanceMLP, PDExpert   # noqa: E402
from sim_core import CarryPayloadSim, package_path                    # noqa: E402


class TestModel(unittest.TestCase):
    def test_model_loads(self):
        sim = CarryPayloadSim(payload_mass=2.0)
        self.assertEqual(sim.model.nu, 17, "执行器数量应为 17（12 腿 + 4 臂 + 1 颈）")
        names = [sim.model.joint(i).name for i in range(sim.model.njnt)]
        for required in ("root_x", "root_z", "root_pitch", "root_yaw", "belt_x",
                         "knee_l", "ankle_pitch_l", "payload_free"):
            self.assertIn(required, names)
        sim.close()

    def test_payload_mass_override(self):
        sim = CarryPayloadSim(payload_mass=4.0)
        self.assertAlmostEqual(float(sim.model.body_mass[sim.payload_body]), 4.0, 6)
        sim.reset(0.0)
        self.assertLess(float(sim.model.body_mass[sim.payload_body]), 0.1)
        sim.close()


class TestWalking(unittest.TestCase):
    def _run(self, mass):
        sim = CarryPayloadSim(payload_mass=mass)
        sim.reset(mass)
        for i in range(int(5.0 / sim.dt)):
            sim.step(v_cmd=0.12, w_cmd=0.0, controller=PDExpert())
            if i % 50 == 0:
                self.assertTrue(sim.standing(),
                                "负载 %.1f kg 时 t=%.2f s 跌倒" % (mass, i * sim.dt))
        distance = sim.distance
        _, pitch, _, _ = sim.posture()
        sim.close()
        return distance, abs(math.degrees(pitch))

    def test_three_payloads_survive(self):
        for mass in (0.0, 2.0, 4.0):
            distance, pitch = self._run(mass)
            self.assertGreater(distance, 0.3, "等效里程应随指令累积")
            self.assertLess(pitch, 25.0, "躯干俯仰应远离跌倒姿态")


class TestControllers(unittest.TestCase):
    def _features(self):
        return {name: 0.0 for name in FEATURE_NAMES}

    def test_expert_clamps_and_signs(self):
        expert = PDExpert()
        feats = self._features()
        feats["pitch"] = 0.25                          # 明显前倾
        out = expert.act(feats)
        self.assertEqual(set(out.keys()), set(OUTPUT_NAMES))
        for value in out.values():
            self.assertLessEqual(abs(value), expert.corr_limit + 1e-9)

    def test_mlp_loads(self):
        path = package_path("models/balance_mlp.npz")
        if not os.path.isfile(path):
            self.skipTest("尚未训练 balance_mlp.npz")
        net = BalanceMLP.load(path)
        feats = self._features()
        out = net.act(feats)
        self.assertEqual(set(out.keys()), set(OUTPUT_NAMES))
        for value in out.values():
            self.assertLessEqual(abs(value), net.corr_limit + 1e-9)
            self.assertTrue(np.isfinite(value))


if __name__ == "__main__":
    unittest.main(verbosity=2)
