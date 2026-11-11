# -*- coding: utf-8 -*-
"""mujoco_shooting 的离线测试：纯控制逻辑 + 无头物理冒烟.

纯逻辑部分不依赖 mujoco；无头部分在装了 mujoco 的机器上才跑，
所以本机和 CI 都能用同一条命令：

    python3 tests/test_mujoco_shooting_local.py
"""
from __future__ import annotations

import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "ground", "mujoco_shooting")
sys.path.insert(0, PKG)

import controller as ctrl  # noqa: E402


class GeometryTests(unittest.TestCase):
    def test_wrap_pi(self):
        self.assertAlmostEqual(ctrl.wrap_pi(3 * math.pi), math.pi, places=9)
        self.assertAlmostEqual(ctrl.wrap_pi(-3 * math.pi), math.pi, places=9)
        self.assertAlmostEqual(ctrl.wrap_pi(0.3), 0.3, places=9)

    def test_aim_error(self):
        self.assertAlmostEqual(ctrl.aim_error(0.0, (1.0, 0.0)), 0.0, places=9)
        self.assertAlmostEqual(ctrl.aim_error(0.0, (0.0, 1.0)), math.pi / 2, places=9)


class TargetTests(unittest.TestCase):
    def _qpos(self, angles):
        qpos = [0.0] * 12
        for i, angle in enumerate(angles):
            qpos[ctrl.TARGET_QPOS0 + i] = angle
        return qpos

    def test_down_flags(self):
        flags = ctrl.down_flags(self._qpos([0.9, 0, 0, 0, 0, 0]))
        self.assertEqual(flags, [True, False, False, False, False, False])
        self.assertFalse(ctrl.all_down(self._qpos([0.9, 0, 0, 0, 0, 0])))

    def test_all_down(self):
        qpos = self._qpos([1.0] * 6)
        self.assertTrue(ctrl.all_down(qpos))
        self.assertEqual(ctrl.select_target(0.0, down=ctrl.down_flags(qpos)), -1)

    def test_select_nearest_standing(self):
        down = [False] * 6
        self.assertEqual(ctrl.select_target(0.0, down=down), 1, "车头正前方是 1 号靶")
        down[1] = True
        # 中间近靶打掉后，正前方（角度 0）还剩远处的 4 号靶，角度差最小
        self.assertEqual(ctrl.select_target(0.0, down=down), 4)

    def test_select_skips_down(self):
        down = [True, True, True, True, False, True]
        self.assertEqual(ctrl.select_target(0.0, down=down), 4)


class TurretTests(unittest.TestCase):
    def test_command_sign(self):
        self.assertGreater(ctrl.turret_command(0.0, 0.0, (0.0, 1.0)), 0.0)
        self.assertLess(ctrl.turret_command(0.0, 0.0, (0.0, -1.0)), 0.0)

    def test_aligned(self):
        self.assertTrue(ctrl.is_aligned(0.0, 0.0, (1.0, 0.0)))
        self.assertFalse(ctrl.is_aligned(0.2, 0.0, (1.0, 0.0)))
        self.assertFalse(ctrl.is_aligned(0.0, 0.5, (1.0, 0.0)))


class BallisticTests(unittest.TestCase):
    def test_flight_hits_target(self):
        """算出的初速度按重力积分，飞行时间到时应正好落在目标点."""
        muzzle = ctrl.muzzle_position(0.0)
        target = (2.0, 0.3)
        vx, vy, vz, dist = ctrl.launch_velocity(muzzle, target, flight_time=0.45)
        t = 0.45
        x = muzzle[0] + vx * t
        y = muzzle[1] + vy * t
        z = ctrl.MUZZLE_Z + vz * t - 0.5 * ctrl.GRAVITY * t * t
        self.assertAlmostEqual(math.hypot(x - target[0], y - target[1]), 0.0, places=6)
        self.assertAlmostEqual(z, ctrl.TARGET_Z, places=6)
        self.assertGreater(dist, 0.0)

    def test_boost_scales_horizontal_speed(self):
        muzzle = ctrl.muzzle_position(0.0)
        vx1, vy1, _, _ = ctrl.launch_velocity(muzzle, (2.0, 0.0), boost=1.0)
        vx2, vy2, _, _ = ctrl.launch_velocity(muzzle, (2.0, 0.0), boost=1.25)
        self.assertAlmostEqual(math.hypot(vx2, vy2) / math.hypot(vx1, vy1), 1.25, places=6)

    def test_distance_scales_speed(self):
        near = ctrl.launch_velocity(ctrl.muzzle_position(0.0), (1.0, 0.0))[0]
        far = ctrl.launch_velocity(ctrl.muzzle_position(0.0), (3.0, 0.0))[0]
        self.assertGreater(far, near, "越远力度越大")


try:
    import mujoco  # noqa: F401
    HAVE_MUJOCO = True
except Exception:  # noqa: BLE001
    HAVE_MUJOCO = False


@unittest.skipUnless(HAVE_MUJOCO, "mujoco 未安装")
class HeadlessSimTests(unittest.TestCase):
    def test_knocks_all_targets_down(self):
        import main
        stats = main.run_headless(max_steps=12000)
        self.assertTrue(stats["all_down"], "12 发内应击倒全部靶子：%s" % stats)
        self.assertGreater(stats["shots"], 0)
        self.assertEqual(stats["down"], stats["targets"])


if __name__ == "__main__":
    unittest.main()
