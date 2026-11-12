"""core.py 的离线单元测试。"""

import math
import unittest

from rov_thruster_allocation_ls import core


class TestTam(unittest.TestCase):
    def test_shape(self):
        tam = core.build_tam(core.THRUSTERS)
        self.assertEqual(len(tam), 6)
        self.assertEqual(len(tam[0]), len(core.THRUSTERS))

    def test_vertical_thrusters_drive_heave(self):
        tam = core.build_tam(core.THRUSTERS)
        self.assertAlmostEqual(tam[2][0], 1.0)
        self.assertAlmostEqual(tam[0][0], 0.0)

    def test_roll_coupling_of_vertical_thrusters(self):
        tam = core.build_tam(core.THRUSTERS)
        # 右上垂推产生 -y 位置上的 z 推力 -> 绕 x 轴力矩
        self.assertNotEqual(tam[3][0], 0.0)


class TestAllocate(unittest.TestCase):
    def setUp(self):
        self.tam = core.build_tam(core.THRUSTERS)

    def test_heave_only(self):
        thrust, achieved, sat, util = core.allocate([0, 0, 8, 0, 0, 0], self.tam, 10.0)
        self.assertFalse(sat)
        self.assertAlmostEqual(achieved[2], 8.0, places=3)
        for i in range(4):
            self.assertAlmostEqual(thrust[i], 2.0, places=3)
        for i in range(4, 8):
            self.assertAlmostEqual(thrust[i], 0.0, places=3)

    def test_yaw_uses_horizontal_thrusters(self):
        thrust, achieved, _, _ = core.allocate([0, 0, 0, 0, 0, 2.0], self.tam, 10.0)
        self.assertAlmostEqual(achieved[5], 2.0, places=3)
        self.assertGreater(abs(thrust[4]) + abs(thrust[5]), 0.0)

    def test_saturation_scales_all(self):
        thrust, achieved, sat, util = core.allocate([0, 0, 100, 0, 0, 0], self.tam, 10.0)
        self.assertTrue(sat)
        self.assertAlmostEqual(max(abs(t) for t in thrust), 10.0, places=6)
        self.assertAlmostEqual(util, 1.0, places=6)
        self.assertLess(achieved[2], 100.0)

    def test_fault_mask_zeroes_column(self):
        masked = core.fault_mask(self.tam, [0])
        self.assertEqual(masked[2][0], 0.0)
        thrust, _, _, _ = core.allocate([0, 0, 8, 0, 0, 0], masked, 10.0)
        self.assertAlmostEqual(thrust[0], 0.0, places=6)
        self.assertGreater(thrust[1], 0.0)

    def test_zero_wrench(self):
        thrust, achieved, sat, util = core.allocate([0] * 6, self.tam, 10.0)
        self.assertFalse(sat)
        self.assertAlmostEqual(util, 0.0)
        self.assertAlmostEqual(core.wrench_norm(achieved), 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
