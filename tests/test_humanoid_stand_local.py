# -*- coding: utf-8 -*-
"""人形机器人站立示例的离线测试：纯控制器逻辑 + 无头物理冒烟.

    python3 tests/test_humanoid_stand_local.py
"""
from __future__ import annotations

import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "humanoid", "mujoco_demo")
sys.path.insert(0, PKG)

import controller as ctrl  # noqa: E402


class PdTorqueTests(unittest.TestCase):
    def test_pulls_towards_target(self):
        tau = ctrl.pd_torque([0.0], [0.0], [0.5], [100.0], [10.0])
        self.assertAlmostEqual(tau[0], 50.0)

    def test_damping_opposes_velocity(self):
        tau = ctrl.pd_torque([0.0], [1.0], [0.0], [100.0], [10.0])
        self.assertAlmostEqual(tau[0], -10.0)

    def test_bias_feedforward_added(self):
        tau = ctrl.pd_torque([0.0], [0.0], [0.0], [100.0], [10.0], bias=[3.5])
        self.assertAlmostEqual(tau[0], 3.5)

    def test_length_mismatch_raises(self):
        with self.assertRaises(ValueError):
            ctrl.pd_torque([0.0], [0.0], [], [1.0], [1.0])

    def test_saturate(self):
        self.assertEqual(ctrl.saturate([5.0, -300.0], 200.0), [5.0, -200.0])


class TiltTests(unittest.TestCase):
    def test_upright_quaternion(self):
        self.assertAlmostEqual(ctrl.tilt_deg((1.0, 0.0, 0.0, 0.0)), 0.0, places=6)

    def test_ninety_degrees(self):
        q = (math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0)
        self.assertAlmostEqual(ctrl.tilt_deg(q), 90.0, places=3)

    def test_is_standing(self):
        self.assertTrue(ctrl.is_standing(1.04, 1.04))
        self.assertFalse(ctrl.is_standing(0.5, 1.04))
        tilted = (math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0)
        self.assertFalse(ctrl.is_standing(1.04, 1.04, quat_wxyz=tilted))


try:
    import mujoco  # noqa: F401
    HAVE_MUJOCO = True
except Exception:  # noqa: BLE001
    HAVE_MUJOCO = False


@unittest.skipUnless(HAVE_MUJOCO, "mujoco 未安装")
class HeadlessStandTests(unittest.TestCase):
    def test_stands_for_five_seconds(self):
        import humanoid_stand
        stats = humanoid_stand.run(headless=True, seconds=5.0)
        self.assertTrue(stats["upright"], "5 s 内应保持站立：%s" % stats)
        self.assertGreater(stats["min_height"], stats["base_height"] * 0.9)


if __name__ == "__main__":
    unittest.main()
