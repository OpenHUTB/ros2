# -*- coding: utf-8 -*-
"""滑模控制模块的单元测试（只用标准库）。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from smc_model import (  # noqa: E402
    simulate_smc,
    simulate_pd,
    mean_abs_error,
    steady_error,
    sat,
)


class TestSMC(unittest.TestCase):
    def test_boundary_layer_bounded(self):
        for z in [-100, -1, 0, 0.3, 5, 100]:
            v = sat(z, 0.08)
            self.assertGreaterEqual(v, -1.0)
            self.assertLessEqual(v, 1.0)

    def test_smc_tracks_reference(self):
        ts, xs, rs = simulate_smc()
        self.assertAlmostEqual(len(xs), len(rs))
        # 稳态（t>=6s）平均误差应很小（边界层量级）
        self.assertLess(steady_error(xs, rs), 0.06)

    def test_smc_beats_pd(self):
        _, xs_smc, rs = simulate_smc()
        _, xs_pd, _ = simulate_pd()
        self.assertLess(mean_abs_error(xs_smc, rs), mean_abs_error(xs_pd, rs))

    def test_disturbance_is_rejected(self):
        # SMC 全程误差应远小于 PD 的稳态（说明扰动被抑制）
        _, xs_smc, rs = simulate_smc()
        _, xs_pd, _ = simulate_pd()
        self.assertLess(mean_abs_error(xs_smc, rs), steady_error(xs_pd, rs))


if __name__ == "__main__":
    unittest.main(verbosity=2)
