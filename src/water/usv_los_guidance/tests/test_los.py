# -*- coding: utf-8 -*-
"""LOS 制导模块的单元测试（只用标准库）。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from los_model import los_guidance, simulate, wrap  # noqa: E402


class TestLOS(unittest.TestCase):
    def test_wrap(self):
        # 3.2 > pi 应被卷绕到 (-pi, pi]
        self.assertAlmostEqual(wrap(2.5), 2.5)
        self.assertAlmostEqual(wrap(4.0), 4.0 - 2 * 3.141592653589793, places=4)
        self.assertAlmostEqual(wrap(-4.0), -4.0 + 2 * 3.141592653589793, places=4)

    def test_guidance_on_path(self):
        # 在路径线上（y_e=0）时，期望航向应等于路径切线角
        chi_p = los_guidance((5.0, 0.0), (0.0, 0.0), (10.0, 0.0), 3.0)[0]
        self.assertAlmostEqual(chi_p, 0.0, places=6)

    def test_guidance_pulls_back(self):
        # 在路径下方（y_e>0 左正？此处 dy>0 -> y_e>0）期望航向应偏向路径
        chi_d, y_e = los_guidance((5.0, 3.0), (0.0, 0.0), (10.0, 0.0), 3.0)
        self.assertGreater(y_e, 0.0)
        self.assertLess(chi_d, 0.0)  # 向下（负向）偏回路径

    def test_converges_to_straight_path(self):
        wps = [(0.0, 0.0), (20.0, 0.0)]
        traj, cte, _ = simulate(wps, (0.0, 4.0), dt=0.05)
        self.assertLess(abs(cte[-1]), 0.5)  # 横漂收敛
        self.assertLess(abs(traj[-1][1]), 0.5)

    def test_reaches_final_waypoint(self):
        wps = [(0.0, 0.0), (12.0, 0.0), (12.0, 8.0)]
        traj, _, _ = simulate(wps, (0.0, 3.0), dt=0.05)
        x, y = traj[-1]
        self.assertLess(abs(x - 12.0), 1.0)
        self.assertLess(abs(y - 8.0), 1.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
