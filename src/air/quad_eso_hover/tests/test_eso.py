"""四旋翼 PD + ESO 模块的单元测试（仅标准库 unittest）。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import eso

REF = 1.0
DT = 0.001
T = 12.0


def ref_fn(t):
    return REF


def dist_fn(t):
    return 0.0 if t < 4.0 else 4.0


class TestESO(unittest.TestCase):
    def setUp(self):
        self.plant = eso.Plant(m=1.0, g=9.81)
        self.wc = 2.5
        self.kp = self.wc * self.wc
        self.kd = 2.0 * self.wc
        self.w0 = 15.0
        self.ts, self.zp, _, _ = eso.simulate_pd(
            self.plant, ref_fn, dist_fn, self.kp, self.kd, T, DT)
        (self.ts2, self.ze, _, _,
         self.z3, self.ft) = eso.simulate_eso(
            self.plant, ref_fn, dist_fn, self.kp, self.kd, self.w0, T, DT)

    def test_pd_has_steady_offset(self):
        # PD-only 在常值扰动下存在稳态偏差 ≈ -d/(m kp) = -0.64
        self.assertGreater(abs(REF - self.zp[-1]), 0.4)

    def test_eso_rejects_disturbance(self):
        # ESO 前馈补偿后稳态误差应接近 0
        self.assertLess(abs(REF - self.ze[-1]), 0.02)

    def test_eso_better_than_pd(self):
        self.assertLess(abs(REF - self.ze[-1]), abs(REF - self.zp[-1]))

    def test_eso_estimates_total_disturbance(self):
        # 收敛后 ESO 估计应贴合真实总扰动 f
        err = 0.0
        cnt = 0
        for k, t in enumerate(self.ts2):
            if t > 8.0:
                err += abs(self.z3[k] - self.ft[k])
                cnt += 1
        self.assertLess(err / max(cnt, 1), 0.05)

    def test_eso_estimates_gravity_before_disturbance(self):
        # 扰动前总扰动 f=-g；ESO 应在 t=3.5 时已估计到 -9.81
        k = int(3.5 / DT)
        self.assertAlmostEqual(self.z3[k], -self.plant.g, delta=0.15)

    def test_eso_tracks_reference_before_disturbance(self):
        k = int(3.5 / DT)
        self.assertAlmostEqual(self.ze[k], REF, delta=0.05)


if __name__ == "__main__":
    unittest.main()
