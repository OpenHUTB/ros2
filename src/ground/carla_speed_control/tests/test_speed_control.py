#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``speed_control`` 的单元测试（纯 numpy，不需要 ROS / CARLA）。

覆盖范围：
* 限速公式与解析解对照、边界情况（直道、曲率为零、非法参数）
* 曲率剖面生成（峰值不被削减、斜坡过渡）
* 前瞻窗口取最大值（相对单点插值的优势）
* 预瞄制动的匀减速公式
* jerk 限制是链路的最后一步
* PID 行为：纯比例、积分抗饱和、零 dt 保护、复位
* 端到端仿真：横向加速度不超限、jerk 不超限、速度跟踪误差
"""

from __future__ import annotations

import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

from speed_control import (  # noqa: E402
    SpeedController,
    SpeedControllerConfig,
    VehicleLimits,
    apply_jerk_limit,
    curvature_profile_s_curve,
    find_brake_point,
    lateral_accel_at,
    lookahead_distance,
    max_curvature_ahead,
    required_decel,
    simulate_speed_control,
    speed_limit_from_curvature,
    speed_limit_profile,
)


class TestSpeedLimitFormula(unittest.TestCase):
    """限速公式：v_max = sqrt(a_lat_max / |κ|) · η"""

    def test_matches_analytic_formula(self):
        """与解析公式对照（安全系数取 1.0 时严格相等）。"""
        for radius in (20.0, 40.0, 100.0, 250.0):
            kappa = 1.0 / radius
            v = speed_limit_from_curvature(
                kappa, max_lateral_accel=3.0, min_speed=0.0, safety_factor=1.0
            )
            self.assertAlmostEqual(v, math.sqrt(3.0 * radius), places=9)

    def test_result_respects_lateral_limit(self):
        """把算出的限速代回 a_lat = v²κ，不应超过设定值。"""
        for kappa in (0.005, 0.01, 0.02, 0.05, 0.1):
            v = speed_limit_from_curvature(kappa, max_lateral_accel=3.0, min_speed=0.0)
            self.assertLessEqual(lateral_accel_at(v, kappa), 3.0 + 1e-9)

    def test_safety_factor_scales_speed(self):
        """安全系数应线性缩放限速。"""
        kappa = 1.0 / 40.0
        v1 = speed_limit_from_curvature(kappa, 3.0, 0.0, safety_factor=1.0)
        v2 = speed_limit_from_curvature(kappa, 3.0, 0.0, safety_factor=0.5)
        self.assertAlmostEqual(v2, 0.5 * v1, places=9)

    def test_straight_road_is_infinite(self):
        """直道（曲率为 0）不构成限速。"""
        self.assertEqual(
            speed_limit_from_curvature(0.0, 3.0, 1.0), float("inf")
        )
        self.assertEqual(
            speed_limit_from_curvature(1e-12, 3.0, 1.0), float("inf")
        )

    def test_curvature_sign_is_ignored(self):
        """左右转（曲率符号）不影响限速大小。"""
        v_left = speed_limit_from_curvature(+0.025, 3.0, 1.0)
        v_right = speed_limit_from_curvature(-0.025, 3.0, 1.0)
        self.assertAlmostEqual(v_left, v_right, places=12)

    def test_min_speed_floor_protects_against_tiny_limit(self):
        """曲率异常大时限速被 min_speed 兜住，不会压到接近 0。"""
        v = speed_limit_from_curvature(10.0, 3.0, min_speed=2.0)
        self.assertAlmostEqual(v, 2.0, places=9)

    def test_infinite_curvature_returns_min_speed(self):
        v = speed_limit_from_curvature(float("inf"), 3.0, min_speed=1.5)
        self.assertAlmostEqual(v, 1.5, places=9)

    def test_rejects_invalid_parameters(self):
        with self.assertRaises(ValueError):
            speed_limit_from_curvature(0.01, max_lateral_accel=0.0)
        with self.assertRaises(ValueError):
            speed_limit_from_curvature(0.01, safety_factor=0.0)
        with self.assertRaises(ValueError):
            speed_limit_from_curvature(0.01, safety_factor=1.5)


class TestLateralAccel(unittest.TestCase):
    """横向加速度计算。"""

    def test_reference_values(self):
        # 10 m/s 过 40 m 半径弯道：a = 100/40 = 2.5
        self.assertAlmostEqual(lateral_accel_at(10.0, 1.0 / 40.0), 2.5, places=9)

    def test_sign_of_curvature_irrelevant(self):
        a = lateral_accel_at(10.0, 1.0 / 40.0)
        b = lateral_accel_at(10.0, -1.0 / 40.0)
        self.assertAlmostEqual(a, b, places=12)

    def test_quadratic_in_speed(self):
        a1 = lateral_accel_at(10.0, 0.01)
        a2 = lateral_accel_at(20.0, 0.01)
        self.assertAlmostEqual(a2, 4.0 * a1, places=9)


class TestLookahead(unittest.TestCase):
    """前瞻距离换算。"""

    def test_basic(self):
        self.assertAlmostEqual(lookahead_distance(10.0, 1.5), 15.0, places=9)

    def test_zero_and_negative(self):
        self.assertAlmostEqual(lookahead_distance(10.0, 0.0), 0.0, places=9)
        self.assertAlmostEqual(lookahead_distance(-5.0, 1.5), 0.0, places=9)


class TestMaxCurvatureAhead(unittest.TestCase):
    """前瞻窗口取最大曲率。"""

    def setUp(self):
        # 0..100 m 直道，100..110 m 出现弯道
        self.s = np.arange(0.0, 201.0, 1.0)
        self.kappa = np.zeros_like(self.s)
        self.kappa[100:110] = 0.025

    def test_finds_peak_inside_window(self):
        """窗口包含弯道时应返回峰值曲率。"""
        k = max_curvature_ahead(self.s, self.kappa, distance=90.0, lookahead=30.0)
        self.assertAlmostEqual(abs(k), 0.025, places=9)

    def test_returns_zero_when_window_is_straight(self):
        k = max_curvature_ahead(self.s, self.kappa, distance=0.0, lookahead=50.0)
        self.assertAlmostEqual(k, 0.0, places=9)

    def test_zero_lookahead_returns_point_value(self):
        """前瞻为 0 时退化为单点取值。"""
        k = max_curvature_ahead(self.s, self.kappa, distance=105.0, lookahead=0.0)
        self.assertAlmostEqual(k, 0.025, places=9)

    def test_better_than_single_point_interpolation(self):
        """关键对比：窗口最大值 >= 单点插值，因此限速不会偏大。

        这正是早期版本横向加速度超标的根因：单点插值漏掉了窗口内的峰值。
        """
        d = 95.0
        la = 10.0
        k_window = max_curvature_ahead(self.s, self.kappa, d, la)
        k_point = float(np.interp(d + la, self.s, self.kappa))
        self.assertGreaterEqual(abs(k_window), abs(k_point))
        v_window = speed_limit_from_curvature(k_window, 3.0, 0.0)
        v_point = speed_limit_from_curvature(k_point, 3.0, 0.0)
        # 窗口法给出的限速必须不高于单点法（更保守）
        self.assertLessEqual(v_window, v_point + 1e-9)


class TestCurvatureProfile(unittest.TestCase):
    """曲率剖面生成。"""

    def test_profile_length(self):
        s, k = curvature_profile_s_curve(total_length=400.0)
        self.assertAlmostEqual(s[-1], 400.0, places=6)
        self.assertEqual(s.size, k.size)

    def test_peak_is_not_eroded_by_ramps(self):
        """弯道中段的曲率峰值必须等于 1/R。

        早期版本对整条剖面做滑动平均，把峰值从 0.025 削到 0.021，
        导致按单点前瞻算出的限速偏大。这是必须守住的回归点。
        """
        radius = 40.0
        s, k = curvature_profile_s_curve(curve_radius=radius)
        self.assertAlmostEqual(np.abs(k).max(), 1.0 / radius, places=6)

    def test_both_directions_present(self):
        """应同时包含左弯（正）与右弯（负）。"""
        s, k = curvature_profile_s_curve()
        self.assertGreater(k.max(), 0.0)
        self.assertLess(k.min(), 0.0)

    def test_transitions_are_gradual(self):
        """入弯处应存在介于 0 与峰值之间的过渡点（模拟缓和曲线）。"""
        s, k = curvature_profile_s_curve()
        peak = np.abs(k).max()
        mid = np.abs(k[(np.abs(k) > 1e-4) & (np.abs(k) < peak - 1e-4)])
        self.assertGreater(mid.size, 0, "未找到过渡点")

    def test_rejects_bad_parameters(self):
        with self.assertRaises(ValueError):
            curvature_profile_s_curve(curve_radius=0.0)
        with self.assertRaises(ValueError):
            curvature_profile_s_curve(curve_length=-1.0)


class TestSpeedLimitProfile(unittest.TestCase):
    """限速剖面。"""

    def test_limit_never_exceeds_cruise(self):
        s, k = curvature_profile_s_curve()
        v = speed_limit_profile(s, k, cruise_speed=12.0)
        self.assertLessEqual(v.max(), 12.0 + 1e-9)

    def test_limit_drops_before_curve_entry(self):
        """限速应"提前"下降：弯道之前的点限速就已低于巡航速度。"""
        s, k = curvature_profile_s_curve(
            straight_length=100.0, curve_radius=40.0, curve_length=120.0
        )
        v = speed_limit_profile(s, k, cruise_speed=12.0, lookahead_distance_m=60.0)
        # 弯道起点约在 s=100，检查 s=50 处（弯前 50 m）限速已开始下降
        v_at_50 = float(np.interp(50.0, s, v))
        self.assertLess(v_at_50, 12.0 - 1e-6)

    def test_curve_section_limit_matches_formula(self):
        s, k = curvature_profile_s_curve(curve_radius=40.0)
        v = speed_limit_profile(s, k, cruise_speed=12.0)
        v_expected = speed_limit_from_curvature(1.0 / 40.0, 3.0, 1.0)
        # 弯道中段（s=150 附近）限速应等于该曲率的限速
        self.assertAlmostEqual(float(np.interp(150.0, s, v)), v_expected, places=6)


class TestAchievableProfile(unittest.TestCase):
    """物理可达性约束（双向扫描）。

    这组测试针对一个真实出现过的缺陷：双向扫描算完写进了局部变量 v，
    函数却仍然返回几何限速 out，导致扫描完全无效——没有任何报错，
    只能靠检查输出曲线发现。因此这里专门验证"扫描确实作用到了返回值"。
    """

    def setUp(self):
        self.s, self.k = curvature_profile_s_curve()
        self.cfg = SpeedControllerConfig()

    def test_achievable_is_not_higher_than_geometric(self):
        """可达性约束只会降低限速，不会提高。"""
        v_geo = speed_limit_profile(self.s, self.k, self.cfg, 12.0,
                                    enforce_achievable=False)
        v_ach = speed_limit_profile(self.s, self.k, self.cfg, 12.0,
                                    enforce_achievable=True)
        big = v_ach > v_geo + 1e-9
        self.assertFalse(big.any(),
                         f"有 {int(big.sum())} 个点的可达限速高于几何限速")

    def test_achievable_actually_differs_somewhere(self):
        """回归点：若扫描结果没被返回，两个剖面会完全相同。

        本场景（55 m 直道夹在两个 40 m 半径弯之间）必然产生差异——
        几何限速在直道中段会跳回巡航速度，而物理上无法在这么短的
        直道上加速上去再刹回来。
        """
        v_geo = speed_limit_profile(self.s, self.k, self.cfg, 12.0,
                                    enforce_achievable=False)
        v_ach = speed_limit_profile(self.s, self.k, self.cfg, 12.0,
                                    enforce_achievable=True)
        diff = np.abs(v_geo - v_ach)
        self.assertGreater(float(diff.max()), 0.1,
                           "可达性扫描未生效（两个剖面几乎相同）")

    def test_no_unreachable_spike_in_straight(self):
        """弯之间的直道上不得出现"限速跳到巡航速度再掉回"的尖峰。

        判据：任意点的限速不得超过"以最大减速度在剩余距离内降到该点
        之后最低限速"所允许的值。这里用更直接的检查——直道中段若限速
        明显高于两侧弯道限速，则说明剖面不可实现。
        """
        v = speed_limit_profile(self.s, self.k, self.cfg, 12.0)
        v_curve = speed_limit_from_curvature(0.025, 3.0, 1.0)
        # 55 m 直道位于 s=175..225 附近，其间限速不应超过弯道限速太多
        mid = (self.s > 180.0) & (self.s < 220.0)
        self.assertTrue(mid.any())
        excess = float(v[mid].max() - v_curve)
        # 允许 1 m/s 的余量（受 max_accel 与直道长度约束）
        self.assertLess(excess, 1.0,
                        f"直道中段限速超出弯道限速 {excess:.2f} m/s，剖面不可实现")

    def test_satisfies_acceleration_limit(self):
        """相邻点的限速增长率不得超过 max_accel 允许的范围。"""
        v = speed_limit_profile(self.s, self.k, self.cfg, 12.0)
        lim = self.cfg.limits
        n = len(v)
        for i in range(n - 1):
            ds = self.s[i + 1] - self.s[i]
            if ds <= 0:
                continue
            v_max_allowed = math.sqrt(v[i] ** 2 + 2.0 * lim.max_decel * ds)
            self.assertLessEqual(
                v[i + 1], v_max_allowed + 1e-6,
                f"s={self.s[i+1]:.0f} 处限速 {v[i+1]:.3f} 超出减速能力"
                f"（上限 {v_max_allowed:.3f}）"
            )

    def test_satisfies_deceleration_limit(self):
        """反向扫描：限速下降不得要求超过 max_decel 的减速度。

        等价地，从后往前看，限速的回升也不得超过 max_accel 允许的范围。
        """
        v = speed_limit_profile(self.s, self.k, self.cfg, 12.0)
        lim = self.cfg.limits
        for i in range(len(v) - 2, -1, -1):
            ds = self.s[i + 1] - self.s[i]
            if ds <= 0:
                continue
            v_max_allowed = math.sqrt(v[i + 1] ** 2 + 2.0 * lim.max_accel * ds)
            self.assertLessEqual(
                v[i], v_max_allowed + 1e-6,
                f"s={self.s[i]:.0f} 处限速 {v[i]:.3f} 超出加速能力"
                f"（上限 {v_max_allowed:.3f}）"
            )

    def test_can_be_disabled(self):
        """关闭开关时应返回纯几何限速。"""
        v_off = speed_limit_profile(self.s, self.k, self.cfg, 12.0,
                                    enforce_achievable=False)
        v_point = np.array([
            speed_limit_from_curvature(kk, self.cfg.limits.max_lateral_accel,
                                       self.cfg.limits.min_speed)
            for kk in self.k
        ])
        # 逐点几何限速的窗口最小值，应等于关闭开关的结果
        self.assertAlmostEqual(float(v_off.min()),
                               float(min(v_point.min(), 12.0)), places=6)


class TestRequiredDecel(unittest.TestCase):
    """预瞄制动的匀减速公式。"""

    def test_matches_kinematics(self):
        """v² = v₀² + 2ad 反解。"""
        v0, v1, d = 12.0, 10.0, 50.0
        a = required_decel(v0, v1, d, max_decel=10.0)
        self.assertAlmostEqual(a, (v1 ** 2 - v0 ** 2) / (2.0 * d), places=9)
        # 代回检验：以该减速度行驶 d 米后速度应为 v1
        v_end = math.sqrt(max(0.0, v0 ** 2 + 2.0 * a * d))
        self.assertAlmostEqual(v_end, v1, places=6)

    def test_no_braking_when_not_needed(self):
        self.assertAlmostEqual(required_decel(10.0, 12.0, 50.0, 6.0), 0.0, places=9)
        self.assertAlmostEqual(required_decel(10.0, 10.0, 50.0, 6.0), 0.0, places=9)

    def test_zero_distance_needs_no_action(self):
        self.assertAlmostEqual(required_decel(12.0, 5.0, 0.0, 6.0), 0.0, places=9)

    def test_clamped_to_braking_capability(self):
        """需求超过制动能力时截断到 -max_decel（物理限制）。"""
        a = required_decel(12.0, 1.0, 0.5, max_decel=6.0)
        self.assertAlmostEqual(a, -6.0, places=9)


class TestFindBrakePoint(unittest.TestCase):
    """预瞄制动的目标点搜索。"""

    def test_finds_first_lower_limit(self):
        s = np.arange(0.0, 101.0, 1.0)
        v = np.full_like(s, 12.0)
        v[50:] = 8.0
        bp = find_brake_point(s, v, distance=0.0, v_now=12.0)
        self.assertIsNotNone(bp)
        dist, v_future = bp
        self.assertAlmostEqual(dist, 50.0, places=6)
        self.assertAlmostEqual(v_future, 8.0, places=6)

    def test_returns_none_when_no_lower_limit(self):
        s = np.arange(0.0, 101.0, 1.0)
        v = np.full_like(s, 12.0)
        self.assertIsNone(find_brake_point(s, v, 0.0, 12.0))

    def test_ignores_points_beyond_horizon(self):
        s = np.arange(0.0, 501.0, 1.0)
        v = np.full_like(s, 12.0)
        v[300:] = 8.0
        self.assertIsNone(find_brake_point(s, v, 0.0, 12.0, horizon=100.0))


class TestJerkLimit(unittest.TestCase):
    """jerk 限制。"""

    def test_limits_rate_of_change(self):
        out = apply_jerk_limit(accel_cmd=6.0, prev_accel=0.0, dt=0.05, max_jerk=4.0)
        self.assertAlmostEqual(out, 4.0 * 0.05, places=9)

    def test_passes_through_small_changes(self):
        out = apply_jerk_limit(accel_cmd=0.1, prev_accel=0.0, dt=0.05, max_jerk=4.0)
        self.assertAlmostEqual(out, 0.1, places=9)

    def test_limits_negative_direction_too(self):
        out = apply_jerk_limit(accel_cmd=-6.0, prev_accel=0.0, dt=0.05, max_jerk=4.0)
        self.assertAlmostEqual(out, -0.2, places=9)

    def test_disabled_when_max_jerk_zero(self):
        out = apply_jerk_limit(accel_cmd=6.0, prev_accel=0.0, dt=0.05, max_jerk=0.0)
        self.assertAlmostEqual(out, 6.0, places=9)

    def test_zero_dt_passes_through(self):
        out = apply_jerk_limit(accel_cmd=6.0, prev_accel=0.0, dt=0.0, max_jerk=4.0)
        self.assertAlmostEqual(out, 6.0, places=9)


class TestSpeedController(unittest.TestCase):
    """PID 控制器行为。"""

    def test_pure_proportional(self):
        cfg = SpeedControllerConfig(
            kp=2.0, ki=0.0, kd=0.0, kff=0.0,
            limits=VehicleLimits(max_accel=100.0, max_decel=100.0, max_jerk=0.0),
        )
        c = SpeedController(cfg)
        a = c.update(target_speed=10.0, current_speed=8.0, dt=0.1)
        self.assertAlmostEqual(a, 4.0, places=6)

    def test_output_clamped_to_vehicle_limits(self):
        cfg = SpeedControllerConfig(
            kp=100.0, ki=0.0, kd=0.0, kff=0.0,
            limits=VehicleLimits(max_accel=3.0, max_decel=6.0, max_jerk=0.0),
        )
        c = SpeedController(cfg)
        self.assertAlmostEqual(
            c.update(target_speed=50.0, current_speed=0.0, dt=0.1), 3.0, places=6
        )
        c.reset()
        self.assertAlmostEqual(
            c.update(target_speed=0.0, current_speed=50.0, dt=0.1), -6.0, places=6
        )

    def test_integral_accumulates_and_is_clamped(self):
        cfg = SpeedControllerConfig(
            kp=0.0, ki=1.0, kd=0.0, kff=0.0, integral_limit=0.5,
            limits=VehicleLimits(max_accel=100.0, max_decel=100.0, max_jerk=0.0),
        )
        c = SpeedController(cfg)
        for _ in range(100):
            c.update(target_speed=10.0, current_speed=0.0, dt=0.1)
        self.assertAlmostEqual(c.integral, 0.5, places=6)

    def test_zero_dt_does_not_explode(self):
        """同一时间戳连调两次不应产生除零或无穷。"""
        cfg = SpeedControllerConfig(limits=VehicleLimits(max_jerk=0.0))
        c = SpeedController(cfg)
        a1 = c.update(10.0, 0.0, dt=0.1)
        a2 = c.update(10.0, 0.0, dt=0.0)
        self.assertTrue(math.isfinite(a1))
        self.assertTrue(math.isfinite(a2))

    def test_reset_clears_state(self):
        cfg = SpeedControllerConfig(limits=VehicleLimits(max_jerk=0.0))
        c = SpeedController(cfg)
        c.update(10.0, 0.0, dt=0.1)
        c.reset()
        self.assertAlmostEqual(c.integral, 0.0, places=12)
        self.assertAlmostEqual(c.last_accel, 0.0, places=12)

    def test_sync_accel_overrides_internal_state(self):
        """sync_accel 用于在外部安全层介入后同步状态，保证 jerk 限制有效。"""
        cfg = SpeedControllerConfig(limits=VehicleLimits(max_jerk=4.0))
        c = SpeedController(cfg)
        c.sync_accel(-2.0)
        self.assertAlmostEqual(c.last_accel, -2.0, places=9)

    def test_jerk_limit_in_controller(self):
        """控制器内置的 jerk 限制：相邻两拍加速度变化不超过 max_jerk·dt。"""
        cfg = SpeedControllerConfig(
            kp=100.0, ki=0.0, kd=0.0, kff=0.0,
            limits=VehicleLimits(max_accel=10.0, max_decel=10.0, max_jerk=2.0),
        )
        c = SpeedController(cfg)
        a1 = c.update(50.0, 0.0, dt=0.1)
        a2 = c.update(50.0, 0.0, dt=0.1)
        self.assertLessEqual(abs(a2 - a1), 2.0 * 0.1 + 1e-9)

    def test_negative_gains_rejected(self):
        with self.assertRaises(ValueError):
            SpeedController(SpeedControllerConfig(kp=-1.0))

    def test_kff_out_of_range_rejected(self):
        with self.assertRaises(ValueError):
            SpeedController(SpeedControllerConfig(kff=1.5))

    def test_feedforward_is_applied(self):
        cfg = SpeedControllerConfig(
            kp=0.0, ki=0.0, kd=0.0, kff=1.0,
            limits=VehicleLimits(max_accel=100.0, max_decel=100.0, max_jerk=0.0),
        )
        c = SpeedController(cfg)
        a = c.update(target_speed=10.0, current_speed=10.0, dt=0.1, target_accel=1.5)
        self.assertAlmostEqual(a, 1.5, places=6)


class TestVehicleLimitsValidation(unittest.TestCase):
    """载具约束的参数校验。"""

    def test_rejects_non_positive_accel(self):
        with self.assertRaises(ValueError):
            VehicleLimits(max_accel=0.0).validate()
        with self.assertRaises(ValueError):
            VehicleLimits(max_decel=-1.0).validate()
        with self.assertRaises(ValueError):
            VehicleLimits(max_lateral_accel=0.0).validate()

    def test_rejects_negative_jerk_and_speed(self):
        with self.assertRaises(ValueError):
            VehicleLimits(max_jerk=-1.0).validate()
        with self.assertRaises(ValueError):
            VehicleLimits(min_speed=-1.0).validate()


class TestEndToEndSimulation(unittest.TestCase):
    """端到端仿真：所有约束必须同时满足。"""

    @classmethod
    def setUpClass(cls):
        cls.s, cls.k = curvature_profile_s_curve()
        cls.cfg = SpeedControllerConfig()
        cls.result = simulate_speed_control(
            cls.s, cls.k, cls.cfg, cruise_speed=12.0
        )

    def test_lateral_acceleration_within_limit(self):
        """核心指标：横向加速度不得超限。

        这条曾经失败过——早期版本用单点前瞻曲率 + 无预瞄制动，
        最大横向加速度达 3.59 m/s²（限值 3.0）。
        """
        self.assertTrue(
            self.result.within_lateral_limit,
            f"横向加速度 {self.result.max_lateral_accel:.3f} 超过限值 "
            f"{self.cfg.limits.max_lateral_accel}",
        )

    def test_acceleration_within_vehicle_limits(self):
        self.assertLessEqual(
            self.result.max_abs_accel, self.cfg.limits.max_accel + 1e-6
        )

    def test_jerk_within_limit(self):
        """jerk 必须受限——早期安全层绕过 jerk 限制时达 40 m/s³。"""
        self.assertLessEqual(
            self.result.max_abs_jerk, self.cfg.limits.max_jerk + 1e-6,
            f"jerk {self.result.max_abs_jerk:.2f} 超过限值 {self.cfg.limits.max_jerk}",
        )

    def test_no_overspeed_beyond_tolerance(self):
        """不应超出"限速 + 容差"。"""
        self.assertLessEqual(self.result.max_overspeed, 1e-6)

    def test_speed_tracking_error_is_small(self):
        """稳态速度跟踪误差应远小于 1 m/s。"""
        self.assertLess(self.result.max_speed_error, 0.5)

    def test_simulation_completes_the_path(self):
        """仿真应走完全程（而不是中途卡住）。"""
        self.assertGreaterEqual(self.result.trace.s[-1], self.s[-1] - 1e-6)

    def test_slower_than_cruise_because_of_curves(self):
        """平均速度应低于巡航速度——因为弯道处确实减速了。"""
        self.assertLess(self.result.mean_speed, 12.0)

    def test_curve_speed_is_close_to_limit(self):
        """弯道处速度应贴近限速（说明没有过度保守）。"""
        tr = self.result.trace
        in_curve = np.abs(tr.curvature) > 0.02
        self.assertGreater(in_curve.sum(), 10, "仿真中未经过弯道")
        v_curve = tr.v[in_curve]
        v_limit_curve = tr.v_limit[in_curve]
        # 弯道内的平均速度应达到限速的 95% 以上
        ratio = float(np.mean(v_curve / v_limit_curve))
        self.assertGreater(ratio, 0.95, f"弯道速度仅达限速的 {ratio:.3f}")

    def test_worse_without_lookahead_planning(self):
        """关掉预瞄制动后指标应变差——证明预瞄确实起作用。"""
        r_no_plan = simulate_speed_control(
            self.s, self.k, self.cfg, cruise_speed=12.0, plan_ahead=False
        )
        # 不预瞄时要么超速更多，要么横向加速度更大
        worse = (
            r_no_plan.max_overspeed > self.result.max_overspeed + 1e-6
            or r_no_plan.max_lateral_accel > self.result.max_lateral_accel + 1e-6
        )
        self.assertTrue(worse, "预瞄制动未体现出改善")


class TestSimulationInputValidation(unittest.TestCase):
    """仿真的输入检查。"""

    def test_rejects_bad_dt(self):
        s, k = curvature_profile_s_curve(total_length=50.0)
        with self.assertRaises(ValueError):
            simulate_speed_control(s, k, dt=0.0)
        with self.assertRaises(ValueError):
            simulate_speed_control(s, k, dt=-0.1)

    def test_rejects_bad_cruise_speed(self):
        s, k = curvature_profile_s_curve(total_length=50.0)
        with self.assertRaises(ValueError):
            simulate_speed_control(s, k, cruise_speed=0.0)

    def test_rejects_mismatched_profile(self):
        with self.assertRaises(ValueError):
            simulate_speed_control(np.arange(10.0), np.arange(5.0))

    def test_rejects_too_short_profile(self):
        with self.assertRaises(ValueError):
            simulate_speed_control(np.array([0.0]), np.array([0.0]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
