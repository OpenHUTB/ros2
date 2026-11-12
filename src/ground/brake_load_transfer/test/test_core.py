# -*- coding: utf-8 -*-
"""载重偏置制动模型的单元测试（纯 numpy，不需要 ROS）。

    python -m unittest discover -s test -p "test_*.py" -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from brake_load_transfer.core import (G, VehicleSpec, axle_brake_force,  # noqa: E402
                                      compare_with_ideal, dynamic_axle_loads,
                                      ideal_stopping_distance, simulate_braking,
                                      simulate_lateral, static_axle_loads,
                                      sweep_bias)


class TestStatics(unittest.TestCase):
    def setUp(self):
        self.s = VehicleSpec()

    def test_axle_loads_sum_to_weight(self):
        fz_f, fz_r = static_axle_loads(self.s)
        self.assertAlmostEqual(fz_f + fz_r, self.s.total_mass() * G, places=6)

    def test_static_distribution(self):
        """质心距前轴 1.2 m / 轴距 2.7 m → 前轴约占 55.6%。"""
        fz_f, fz_r = static_axle_loads(self.s)
        self.assertAlmostEqual(fz_f / (fz_f + fz_r), 1.5 / 2.7, places=6)

    def test_payload_moves_com(self):
        """载重后移，合成质心后移，前轴载荷下降。"""
        base_f, _ = static_axle_loads(self.s)
        s2 = VehicleSpec(payload=300.0, payload_x=2.4)
        self.assertGreater(s2.com_position(), self.s.com_position())
        f2, _ = static_axle_loads(s2)
        self.assertGreater(f2, base_f)      # 总质量变大，前轴仍然变大
        s3 = VehicleSpec(payload=300.0, payload_x=0.2)
        # 同样 300 kg，压在车头时前轴分摊更多
        self.assertGreater(static_axle_loads(s3)[0] / s3.total_mass(),
                           static_axle_loads(s2)[0] / s2.total_mass())


class TestLoadTransfer(unittest.TestCase):
    def test_transfer_magnitude(self):
        """动态转移量 = m·a·h/L。"""
        s = VehicleSpec()
        fz_f0, fz_r0 = static_axle_loads(s)
        fz_f, fz_r = dynamic_axle_loads(s, 5.0)
        expect = s.total_mass() * 5.0 * s.com_height / s.wheelbase
        self.assertAlmostEqual(fz_f - fz_f0, expect, places=6)
        self.assertAlmostEqual(fz_r0 - fz_r, expect, places=6)

    def test_rear_never_negative(self):
        s = VehicleSpec(com_height=1.2)
        _, fz_r = dynamic_axle_loads(s, 20.0)
        self.assertGreaterEqual(fz_r, 0.0)

    def test_no_transfer_without_decel(self):
        s = VehicleSpec()
        self.assertAlmostEqual(dynamic_axle_loads(s, 0.0)[0],
                               static_axle_loads(s)[0], places=9)


class TestBrakeForce(unittest.TestCase):
    def test_bias_split(self):
        s = VehicleSpec()
        fz_f, fz_r = static_axle_loads(s)
        fx_f, fx_r, _, _ = axle_brake_force(s, fz_f, fz_r, 4000.0)
        self.assertAlmostEqual(fx_f, 0.62 * 4000.0, places=6)
        self.assertAlmostEqual(fx_r, 0.38 * 4000.0, places=6)

    def test_cap_by_friction(self):
        s = VehicleSpec()
        fz_f, fz_r = 1000.0, 1000.0
        fx_f, _, lf, _ = axle_brake_force(s, fz_f, fz_r, 100000.0)
        self.assertAlmostEqual(fx_f, s.mu * 1000.0, places=6)
        self.assertTrue(lf)


class TestStopping(unittest.TestCase):
    def test_ideal_distance(self):
        """v²/(2μg)：100 km/h、μ=0.9 → 43.7 m。"""
        s = VehicleSpec()
        self.assertAlmostEqual(ideal_stopping_distance(s), 27.78 ** 2 / (2 * 0.9 * G),
                               places=6)

    def test_best_bias_approaches_ideal(self):
        """把 bias 调到最优时，制动距离应非常接近理想 EBD（≤2%）。"""
        s = VehicleSpec(payload=300.0, payload_x=2.4)
        rows = sweep_bias(s, biases=[i / 100 for i in range(50, 81, 2)],
                          payload=300.0, payload_x=2.4)
        best = min(r["distance_m"] for r in rows)
        ideal = ideal_stopping_distance(s)
        self.assertLess((best - ideal) / ideal, 0.02)

    def test_payload_to_rear_shortens_with_fixed_bias(self):
        """固定 β=0.62 时，载重后移反而更短——因为后轴载荷变大、能多出力。"""
        s = VehicleSpec(payload=300.0, payload_x=0.3)
        d_front = simulate_braking(s)["distance_m"]
        s2 = VehicleSpec(payload=300.0, payload_x=2.4)
        d_rear = simulate_braking(s2)["distance_m"]
        self.assertLess(d_rear, d_front)

    def test_peak_decel_bounded(self):
        """峰值减速度不可能超过 μg（加上空气阻力的小量）。"""
        s = VehicleSpec()
        r = simulate_braking(s)
        self.assertLess(r["peak_decel"], s.mu * G + 0.5)

    def test_repeatable(self):
        s = VehicleSpec(payload=200.0, payload_x=1.8)
        self.assertAlmostEqual(simulate_braking(s)["distance_m"],
                               simulate_braking(s)["distance_m"], places=9)

    def test_compare_with_ideal_fields(self):
        c = compare_with_ideal(VehicleSpec())
        self.assertIn("gap_pct", c)
        self.assertGreater(c["gap_pct"], 0.0)


class TestLockOrder(unittest.TestCase):
    """review 指出的 bug 的回归测试：抱死顺序必须来自自洽迭代的稳态，而不是第一步瞬态。"""

    def test_lock_order_is_rear_at_full_braking(self):
        """满制动、默认 β=0.62：前轴动态载荷约 73%，后轴被过制动 → 后轴先饱和。"""
        self.assertEqual(simulate_braking(VehicleSpec())["first_lock"], "rear")

    def test_lock_order_front_when_bias_high(self):
        """β 提到 0.70 时前轴先饱和，抱死顺序应翻转。"""
        spec = VehicleSpec(payload=300.0, payload_x=2.4, brake_bias=0.70)
        self.assertEqual(simulate_braking(spec)["first_lock"], "front")

    def test_rear_lift_flag_false_for_normal_spec(self):
        self.assertFalse(simulate_braking(VehicleSpec())["rear_lift"])

    def test_rear_lift_flag_true_when_com_high(self):
        """质心极高时后轴载荷被压到 0，rear_lift 必须置位。"""
        self.assertTrue(simulate_braking(VehicleSpec(com_height=3.0))["rear_lift"])

    def test_distance_matches_self_consistent_iteration(self):
        """review 给出的自洽迭代基准值：空载 48.05 m、载重 300 kg @2.4 m 45.03 m（v0=27.78）。"""
        self.assertAlmostEqual(simulate_braking(VehicleSpec())["distance_m"], 48.05, delta=0.05)
        spec = VehicleSpec(payload=300.0, payload_x=2.4)
        self.assertAlmostEqual(simulate_braking(spec)["distance_m"], 45.03, delta=0.05)

class TestLateral(unittest.TestCase):
    def test_symmetric_no_drift(self):
        """左右对称时不应有任何跑偏——这是模型的对称性检查。"""
        r = simulate_lateral(VehicleSpec(payload=300.0, payload_x=1.35))
        self.assertAlmostEqual(r["yaw_deg"], 0.0, places=9)
        self.assertAlmostEqual(r["drift_m"], 0.0, places=9)

    def test_monotonic_in_offset(self):
        yaws = []
        for dy in (0.05, 0.10, 0.20):
            s = VehicleSpec(payload=300.0, payload_x=1.35, payload_y=dy)
            yaws.append(abs(simulate_lateral(s)["yaw_deg"]))
        self.assertLess(yaws[0], yaws[1])
        self.assertLess(yaws[1], yaws[2])

    def test_offset_direction(self):
        """载重偏右与偏左，横摆方向相反。"""
        right = simulate_lateral(VehicleSpec(payload=300.0, payload_y=0.2))
        left = simulate_lateral(VehicleSpec(payload=300.0, payload_y=-0.2))
        self.assertLess(right["yaw_deg"] * left["yaw_deg"], 0.0)
        self.assertLess(abs(right["yaw_deg"] + left["yaw_deg"]), 1e-6)

    def test_brake_force_difference_grows_with_offset(self):
        """左右制动力差随横向偏置增大（不要求严格线性：抱死时刻会变）。"""
        s1 = simulate_lateral(VehicleSpec(payload=300.0, payload_y=0.1))
        s2 = simulate_lateral(VehicleSpec(payload=300.0, payload_y=0.2))
        self.assertGreater(abs(s2["brake_force_diff"]), 1.5 * abs(s1["brake_force_diff"]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
