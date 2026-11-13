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
    boundary_residual,
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

    # ---------- tanh 边界层下的常值扰动残差 ----------
    def test_constant_disturbance_residual_matches_analytic(self):
        """常值扰动 d=2 下，tanh 边界层不给出零误差，而是解析残差

            s* = phi*atanh(d/K),  e* = s*/lam = 0.08*atanh(0.4)/3 ≈ 0.0113。

        实测稳态误差必须与解析值一致（差 < 0.002），验证文档 4.2 节的
        边界层分析；这也锁定「残差随 phi 增大、随 K 增大而减小」的规律。
        """
        K, phi, lam, d0 = 5.0, 0.08, 3.0, 2.0
        pred = boundary_residual(d0, K, phi, lam)
        self.assertAlmostEqual(pred, 0.0113, delta=0.0002)
        _, xs, rs = simulate_smc(K=K, phi=phi, lam=lam, disturbance_fn=lambda t: d0)
        self.assertAlmostEqual(steady_error(xs, rs), pred, delta=0.002)

    def test_residual_scales_with_boundary_layer(self):
        """残差与 phi 成正比：phi 减半，残差约减半。"""
        d0 = 2.0
        _, xb, rb = simulate_smc(phi=0.08, disturbance_fn=lambda t: d0)
        _, xh, rh = simulate_smc(phi=0.04, disturbance_fn=lambda t: d0)
        self.assertAlmostEqual(steady_error(xb, rb), 2.0 * steady_error(xh, rh),
                               delta=0.004)

    def test_pd_constant_disturbance_steady_error(self):
        """对照：PD 在常值扰动下的稳态误差 = d/Kp = 0.25（无法消除）。"""
        _, xs, rs = simulate_pd(Kp=8.0, disturbance_fn=lambda t: 2.0)
        self.assertAlmostEqual(steady_error(xs, rs), 2.0 / 8.0, delta=0.005)


if __name__ == "__main__":
    unittest.main(verbosity=2)
