"""IDM 跟车模块的单元测试（仅标准库 unittest）。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import idm as idm_mod


def const_lead(v):
    return lambda t: v


class TestIDM(unittest.TestCase):
    def setUp(self):
        # 期望速度 20 m/s，巡航 15 m/s
        self.model = idm_mod.IDM(v0=20.0, T=1.5, a=2.0, b=3.0, s0=2.0)

    def test_free_road_reaches_v0(self):
        # 前车远快且远去 -> 自由流，自车应收敛到期望速度 v0
        lead = const_lead(50.0)
        ts, gaps, ve, vl = idm_mod.simulate_single(
            self.model, lead, T=80.0, dt=0.01, ego_v0=0.0, gap0=300.0)
        self.assertAlmostEqual(ve[-1], self.model.v0, delta=0.5)

    def test_steady_following_gap(self):
        # 从 15 m/s 的平衡间距出发，间距与速度应基本保持
        gap_eq = self.model.equilibrium_gap(15.0)
        ts, gaps, ve, vl = idm_mod.simulate_single(
            self.model, const_lead(15.0), T=40.0, dt=0.01, ego_v0=15.0, gap0=gap_eq)
        self.assertAlmostEqual(gaps[-1], gap_eq, delta=1.0)
        self.assertAlmostEqual(ve[-1], 15.0, delta=0.3)

    def test_equilibrium_is_attracting(self):
        # 从偏近/偏远两种初始间距出发，最终收敛到同一平衡间距
        gap_eq = self.model.equilibrium_gap(15.0)
        _, g1, _, _ = idm_mod.simulate_single(
            self.model, const_lead(15.0), T=60.0, dt=0.01, ego_v0=15.0, gap0=gap_eq - 8)
        _, g2, _, _ = idm_mod.simulate_single(
            self.model, const_lead(15.0), T=60.0, dt=0.01, ego_v0=15.0, gap0=gap_eq + 8)
        self.assertAlmostEqual(g1[-1], g2[-1], delta=1.5)

    def test_accel_upper_bound(self):
        # IDM 正向加速度上界为 a（自由流项<=1, 交互项>=0）
        for v in [0.0, 5.0, 15.0, 25.0]:
            for gap in [2.0, 10.0, 100.0, 1e6]:
                a = self.model.accel(v, 15.0, gap)
                self.assertLessEqual(a, self.model.a + 1e-9)

    def test_no_collision_single(self):
        # 前车急减速，自车不得追上（间距保持为正）
        def lead(t):
            return 15.0 if t < 5 else 3.0
        ts, gaps, ve, vl = idm_mod.simulate_single(
            self.model, lead, T=40.0, dt=0.01, ego_v0=15.0, gap0=25.0)
        self.assertGreater(min(gaps), 0.0)

    def test_platoon_stays_together(self):
        # 车队经历领航脉冲后，各车最终速度回到 15 m/s 附近
        def pulse(t):
            import math
            if t < 5 or t > 15:
                return 15.0
            return 15.0 - 5.0 * (0.5 - 0.5 * math.cos(2.0 * math.pi * (t - 5.0) / 10.0))
        gap0 = self.model.equilibrium_gap(15.0)
        ts, speeds = idm_mod.simulate_platoon(
            self.model, pulse, n_follow=5, T=60.0, dt=0.01, v_init=15.0, gap_init=gap0)
        final = [speeds[i][-1] for i in range(1, 6)]
        self.assertLess(max(final) - min(final), 0.2)
        for f in final:
            self.assertAlmostEqual(f, 15.0, delta=0.3)


if __name__ == "__main__":
    unittest.main()
