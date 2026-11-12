# -*- coding: utf-8 -*-
"""LIPM、ZMP 与步态求解的单元测试。

断言的是已验证的动力学性质，例如：

* 解析解满足 ẍ = ω²(x − p)
* 静止时 ZMP 等于质心投影
* 加速时 ZMP 落在质心后方
* 开环周期解不稳定（误差按 e^(ωT) 放大）
* 闭环能收敛、ZMP 不越出支撑多边形
"""

from __future__ import annotations

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

from dcm import (  # noqa: E402
    dcm,
    dcm_reference,
    summarise_track,
    track,
    zmp_feedback,
)
from footstep import (  # noqa: E402
    Footstep,
    GaitPlan,
    stability_margin,
    support_polygon,
)
from gait import (  # noqa: E402
    build_symmetric_plan,
    integrate,
    solve_periodic,
    simulate,
    summarise,
)
from lipm import (  # noqa: E402
    GRAVITY,
    LIPMParams,
    capture_point,
    com_trajectory,
    omega,
    stop_position,
    time_to_stop,
    zmp_from_state,
)


# ===========================================================================
class TestLIPMParams(unittest.TestCase):
    def test_default_values(self):
        p = LIPMParams()
        self.assertAlmostEqual(p.z_c, 0.8)
        self.assertAlmostEqual(p.g, GRAVITY)

    def test_omega_from_params(self):
        p = LIPMParams(z_c=0.8, g=9.81)
        self.assertAlmostEqual(p.omega, math.sqrt(9.81 / 0.8), places=12)

    def test_reject_bad_params(self):
        with self.assertRaises(ValueError):
            LIPMParams(z_c=0.0)
        with self.assertRaises(ValueError):
            LIPMParams(z_c=-1.0)
        with self.assertRaises(ValueError):
            LIPMParams(g=0.0)

    def test_omega_function(self):
        self.assertAlmostEqual(omega(0.8), math.sqrt(GRAVITY / 0.8), places=12)
        with self.assertRaises(ValueError):
            omega(0.0)

    def test_higher_com_means_lower_frequency(self):
        self.assertGreater(omega(0.4), omega(0.8))


# ===========================================================================
class TestComTrajectory(unittest.TestCase):
    def test_initial_conditions(self):
        x, v, _a = com_trajectory(0.1, 0.3, 0.0, 0.0, 3.5)
        self.assertAlmostEqual(x, 0.1, places=12)
        self.assertAlmostEqual(v, 0.3, places=12)

    def test_static_equilibrium_stays_put(self):
        """质心正好在 ZMP 正上方且速度为 0 时，应保持不动。"""
        x, v, a = com_trajectory(0.25, 0.0, 0.25, 1.5, 3.5)
        self.assertAlmostEqual(x, 0.25, places=12)
        self.assertAlmostEqual(v, 0.0, places=12)
        self.assertAlmostEqual(a, 0.0, places=12)

    def test_satisfies_dynamics(self):
        """解析解应满足 ẍ = ω²(x − p)。"""
        om, p, x0, v0 = 3.5, 0.1, 0.0, 0.2
        for t in (0.0, 0.1, 0.3, 0.7, 1.2):
            x, _v, a = com_trajectory(x0, v0, p, t, om)
            self.assertAlmostEqual(a, om * om * (x - p), places=9)

    def test_numerical_second_derivative_matches(self):
        """解析加速度应与二阶差分一致（相对误差极小）。"""
        om, p, x0, v0 = 3.5, 0.0, 0.1, 0.4
        t, h = 0.3, 1e-5
        xc, _v, a = com_trajectory(x0, v0, p, t, om)
        xp, _, _ = com_trajectory(x0, v0, p, t + h, om)
        xm, _, _ = com_trajectory(x0, v0, p, t - h, om)
        a_num = (xp - 2.0 * xc + xm) / (h * h)
        self.assertLess(abs(a - a_num) / max(1.0, abs(a)), 1e-5)

    def test_com_ahead_of_zmp_accelerates_forward(self):
        """质心在 ZMP 前方时加速度朝正方向。

        注意这正是倒立摆的不稳定之处：偏得越多，越往前跑，而不是被拉回来。
        """
        x, _v, a = com_trajectory(0.3, 0.0, 0.0, 0.0, 3.5)
        self.assertGreater(x, 0.0)
        self.assertGreater(a, 0.0)

    def test_com_behind_zmp_accelerates_backward(self):
        """质心在 ZMP 后方时加速度朝负方向，同样是在远离。"""
        x, _v, a = com_trajectory(-0.3, 0.0, 0.0, 0.0, 3.5)
        self.assertLess(x, 0.0)
        self.assertLess(a, 0.0)

    def test_reject_negative_time(self):
        with self.assertRaises(ValueError):
            com_trajectory(0.0, 0.0, 0.0, -0.1, 3.5)

    def test_needs_omega_or_params(self):
        with self.assertRaises(ValueError):
            com_trajectory(0.0, 0.0, 0.0, 0.1)


# ===========================================================================
class TestZMP(unittest.TestCase):
    def test_static_zmp_equals_com(self):
        """静止时 ZMP 就是质心水平投影。"""
        self.assertAlmostEqual(zmp_from_state(0.4, 0.0, z_c=0.8), 0.4, places=12)

    def test_zmp_behind_com_when_accelerating_forward(self):
        """向前加速时 ZMP 落在质心后方。"""
        p = zmp_from_state(0.5, 1.0, z_c=0.8)
        self.assertLess(p, 0.5)

    def test_zmp_ahead_of_com_when_decelerating(self):
        p = zmp_from_state(0.5, -1.0, z_c=0.8)
        self.assertGreater(p, 0.5)

    def test_requires_positive_zc(self):
        with self.assertRaises(ValueError):
            zmp_from_state(0.0, 0.0, z_c=0.0)

    def test_zmp_formula_matches_omega_form(self):
        """p = x − ẍ/ω² 与 p = x − ẍ·z_c/g 应一致。"""
        z_c, g, x, a = 0.8, 9.81, 0.3, 1.2
        om = omega(z_c, g)
        self.assertAlmostEqual(x - a / (om * om),
                               zmp_from_state(x, a, z_c, g), places=12)


# ===========================================================================
class TestCapturePoint(unittest.TestCase):
    def test_stationary_is_itself(self):
        self.assertAlmostEqual(capture_point(0.3, 0.0, 3.5), 0.3, places=12)

    def test_moving_forward_has_larger_capture_point(self):
        self.assertGreater(capture_point(0.3, 0.5, 3.5), 0.3)

    def test_stop_position_equals_capture_point(self):
        self.assertAlmostEqual(stop_position(0.2, 0.4, 3.0),
                               capture_point(0.2, 0.4, 3.0), places=12)

    def test_reject_bad_omega(self):
        with self.assertRaises(ValueError):
            capture_point(0.0, 0.0, 0.0)

    def test_time_to_stop_finite_with_zmp_ahead(self):
        """ZMP 在质心前方时，质心在有限时间内停下。

        LIPM 里 ẍ = ω²(x − p)，所以 p > x 才会减速。
        """
        om, x, v = 3.5, 0.0, 0.5
        t = time_to_stop(x, v, om, zmp=0.3)
        self.assertGreater(t, 0.0)
        self.assertTrue(math.isfinite(t))
        # 解析解代回去，末速度应为 0
        _x, v_end, _a = com_trajectory(x, v, 0.3, t, om)
        self.assertAlmostEqual(v_end, 0.0, places=9)

    def test_zmp_behind_com_never_stops(self):
        """ZMP 在质心后方时只会越走越快，没有正的停车时间。"""
        om, x, v = 3.5, 0.0, 0.5
        t = time_to_stop(x, v, om, zmp=-0.3)
        self.assertTrue(t <= 0.0 or not math.isfinite(t))

    def test_time_to_stop_infinite_when_asymptotic(self):
        """ZMP 取停止位置时只能渐近停下，时间趋于无穷。"""
        om, x, v = 3.5, 0.0, 0.5
        self.assertEqual(time_to_stop(x, v, om), float("inf"))

    def test_time_to_stop_infinite_when_at_rest(self):
        self.assertEqual(time_to_stop(0.0, 0.0, 3.5), float("inf"))


# ===========================================================================
class TestFootstep(unittest.TestCase):
    def test_reach_symmetric(self):
        f = Footstep("L", 0.5, 0.0, 1.0, half_length=0.1)
        self.assertAlmostEqual(f.reach[0], 0.4)
        self.assertAlmostEqual(f.reach[1], 0.6)

    def test_covers_interval(self):
        f = Footstep("L", 0.0, 0.2, 0.8)
        self.assertFalse(f.covers(0.1))
        self.assertTrue(f.covers(0.2))
        self.assertTrue(f.covers(0.5))
        self.assertTrue(f.covers(0.8))
        self.assertFalse(f.covers(0.9))

    def test_reject_bad_side(self):
        with self.assertRaises(ValueError):
            Footstep("X", 0.0, 0.0, 1.0)

    def test_reject_bad_interval(self):
        with self.assertRaises(ValueError):
            Footstep("L", 0.0, 1.0, 0.5)

    def test_reject_bad_half_length(self):
        with self.assertRaises(ValueError):
            Footstep("L", 0.0, 0.0, 1.0, half_length=0.0)


# ===========================================================================
class TestGaitPlan(unittest.TestCase):
    def setUp(self):
        self.plan = GaitPlan([
            Footstep("L", 0.0, 0.0, 0.6),
            Footstep("R", 0.35, 0.5, 1.1),
            Footstep("L", 0.70, 1.0, 1.6),
        ])

    def test_single_support_polygon(self):
        lo, hi = self.plan.support_polygon(0.2)
        self.assertAlmostEqual(lo, -0.12)
        self.assertAlmostEqual(hi, 0.12)

    def test_double_support_union(self):
        lo, hi = self.plan.support_polygon(0.55)
        self.assertAlmostEqual(lo, -0.12)
        self.assertAlmostEqual(hi, 0.47)

    def test_phase_kind(self):
        self.assertEqual(self.plan.phase_kind(0.2), "single")
        self.assertEqual(self.plan.phase_kind(0.55), "double")

    def test_zmp_reference_single_is_center(self):
        self.assertAlmostEqual(self.plan.zmp_reference(0.2), 0.0, places=12)

    def test_zmp_reference_double_transitions(self):
        a = self.plan.zmp_reference(0.5)
        b = self.plan.zmp_reference(0.6)
        self.assertAlmostEqual(a, 0.0, places=9)
        self.assertAlmostEqual(b, 0.35, places=9)

    def test_duration(self):
        self.assertAlmostEqual(self.plan.duration, 1.6, places=12)

    def test_reject_empty(self):
        with self.assertRaises(ValueError):
            GaitPlan([])

    def test_support_polygon_wrapper(self):
        self.assertEqual(support_polygon(self.plan, 0.2),
                         self.plan.support_polygon(0.2))


# ===========================================================================
class TestStabilityMargin(unittest.TestCase):
    def test_inside_positive(self):
        self.assertAlmostEqual(stability_margin(0.0, (-0.1, 0.1)), 0.1, places=12)

    def test_at_boundary_zero(self):
        self.assertAlmostEqual(stability_margin(0.1, (-0.1, 0.1)), 0.0, places=12)

    def test_outside_negative(self):
        self.assertLess(stability_margin(0.2, (-0.1, 0.1)), 0.0)

    def test_uses_nearer_edge(self):
        # 距离左边界 0.05，右边界 0.15 → 取 0.05
        self.assertAlmostEqual(stability_margin(-0.05, (-0.1, 0.2)), 0.05,
                               places=12)

    def test_reject_invalid_polygon(self):
        with self.assertRaises(ValueError):
            stability_margin(0.0, (0.1, -0.1))


# ===========================================================================
class TestBuildPlan(unittest.TestCase):
    def test_step_count(self):
        plan = build_symmetric_plan(6, 0.35, 0.5, duty=0.6)
        self.assertEqual(len(plan.steps), 6)

    def test_alternating_sides(self):
        plan = build_symmetric_plan(4, 0.35, 0.5, duty=0.6)
        self.assertEqual([s.side for s in plan.steps], ["L", "R", "L", "R"])

    def test_constant_step_length(self):
        plan = build_symmetric_plan(5, 0.3, 0.4, duty=0.6)
        diffs = [b.center - a.center for a, b in zip(plan.steps, plan.steps[1:])]
        for d in diffs:
            self.assertAlmostEqual(d, 0.3, places=12)

    def test_stance_duration(self):
        plan = build_symmetric_plan(4, 0.35, 0.5, duty=0.6)
        for s in plan.steps:
            self.assertAlmostEqual(s.t_end - s.t_start, 0.6, places=12)

    def test_double_support_exists(self):
        """duty > 0.5 时应存在双脚支撑。"""
        plan = build_symmetric_plan(4, 0.35, 0.5, duty=0.6)
        self.assertEqual(plan.phase_kind(0.55), "double")

    def test_reject_bad_arguments(self):
        with self.assertRaises(ValueError):
            build_symmetric_plan(1, 0.35, 0.5)
        with self.assertRaises(ValueError):
            build_symmetric_plan(4, 0.0, 0.5)
        with self.assertRaises(ValueError):
            build_symmetric_plan(4, 0.35, 0.0)
        with self.assertRaises(ValueError):
            build_symmetric_plan(4, 0.35, 0.5, duty=0.4)


# ===========================================================================
class TestPeriodicSolution(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p = LIPMParams(z_c=0.8)
        cls.plan = build_symmetric_plan(12, 0.35, 0.5, duty=0.6)

    def test_periodic_state_is_fixed_point(self):
        """用周期解积分一个周期后，位移恰为 stride、速度不变。"""
        x0, v0, stride, t0 = solve_periodic(self.plan, self.p, dt=0.001)
        x1, v1 = integrate(x0, v0, self.plan.zmp_reference, t0, t0 + 1.0,
                           0.001, self.p.omega)
        self.assertAlmostEqual(x1 - x0, stride, places=6)
        self.assertAlmostEqual(v1, v0, places=6)

    def test_stride_is_two_steps(self):
        _x0, _v0, stride, _t0 = solve_periodic(self.plan, self.p, dt=0.001)
        self.assertAlmostEqual(stride, 0.70, places=9)

    def test_cycle_starts_after_first_step(self):
        _x0, _v0, _s, t0 = solve_periodic(self.plan, self.p, dt=0.001)
        self.assertAlmostEqual(t0, 0.5, places=9)

    def test_reject_odd_cycle(self):
        with self.assertRaises(ValueError):
            solve_periodic(self.plan, self.p, cycle_steps=3)

    def test_reject_start_index_zero(self):
        with self.assertRaises(ValueError):
            solve_periodic(self.plan, self.p, start_index=0)

    def test_reject_plan_too_short(self):
        p2 = build_symmetric_plan(3, 0.35, 0.5, duty=0.6)
        with self.assertRaises(ValueError):
            solve_periodic(p2, self.p, cycle_steps=4)

    def test_open_loop_is_unstable(self):
        """开环周期解不稳定：误差每周期按 e^(ωT) 放大。"""
        om = self.p.omega
        x0, v0, _stride, t0 = solve_periodic(self.plan, self.p, dt=0.001)
        zf = self.plan.zmp_reference
        x, v = x0, v0
        devs = []
        for k in range(4):
            ta = t0 + k * 1.0
            x, v = integrate(x, v, zf, ta, ta + 1.0, 0.001, om)
            devs.append(abs(v - v0))
        # 相邻周期的放大倍数应明显大于 1
        ratios = [devs[i + 1] / devs[i] for i in range(len(devs) - 1)
                  if devs[i] > 1e-15]
        self.assertTrue(ratios)
        for r in ratios:
            self.assertGreater(r, 5.0)

    def test_integrate_rejects_bad_dt(self):
        with self.assertRaises(ValueError):
            integrate(0.0, 0.0, self.plan.zmp_reference, 0.0, 1.0, 0.0, 3.5)


# ===========================================================================
class TestClosedLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p = LIPMParams(z_c=0.8)
        cls.plan = build_symmetric_plan(12, 0.35, 0.5, duty=0.6)
        cls.v_avg = 0.70

    def test_travels_forward(self):
        traj = track(self.plan, self.p, 0.0, 0.0, self.v_avg, dt=0.002)
        s = summarise_track(traj)
        self.assertGreater(s["com_travel"], 3.0)
        self.assertLess(s["com_travel"], 5.0)

    def test_zmp_never_leaves_polygon(self):
        traj = track(self.plan, self.p, 0.0, 0.0, self.v_avg, dt=0.002)
        for p, lo, hi in zip(traj["zmp_actual"], traj["poly_lo"], traj["poly_hi"]):
            self.assertGreaterEqual(p, lo - 1e-9)
            self.assertLessEqual(p, hi + 1e-9)

    def test_margin_non_negative(self):
        traj = track(self.plan, self.p, 0.0, 0.0, self.v_avg, dt=0.002)
        self.assertTrue(all(m >= -1e-9 for m in traj["margin"]))

    def test_stable_flag(self):
        traj = track(self.plan, self.p, 0.0, 0.0, self.v_avg, dt=0.002)
        self.assertTrue(summarise_track(traj)["stable"])

    def test_zero_gain_diverges(self):
        """K = 0 退化成纯开环跟踪，应该发散。"""
        traj = track(self.plan, self.p, 0.0, 0.0, self.v_avg, dt=0.002, gain=0.0)
        s = summarise_track(traj)
        self.assertGreater(abs(s["com_travel"]), 100.0)

    def test_gain_one_is_stable(self):
        traj = track(self.plan, self.p, 0.0, 0.0, self.v_avg, dt=0.002, gain=1.0)
        self.assertLess(abs(summarise_track(traj)["com_travel"]), 10.0)

    def test_reject_negative_gain(self):
        with self.assertRaises(ValueError):
            zmp_feedback(0.0, 0.0, 0.0, gain=-1.0)

    def test_reject_bad_dt(self):
        with self.assertRaises(ValueError):
            track(self.plan, self.p, 0.0, 0.0, self.v_avg, dt=0.0)

    def test_history_lengths_match(self):
        traj = track(self.plan, self.p, 0.0, 0.0, self.v_avg, dt=0.002)
        n = len(traj["time"])
        for k in ("com_x", "zmp_actual", "margin", "dcm"):
            self.assertEqual(len(traj[k]), n)

    def test_dcm_definition(self):
        om = self.p.omega
        self.assertAlmostEqual(dcm(0.3, 0.7, om), 0.3 + 0.7 / om, places=12)

    def test_dcm_reference_offset(self):
        om = self.p.omega
        self.assertAlmostEqual(dcm_reference(0.5, 0.7, om), 0.5 + 0.7 / om,
                               places=12)

    def test_zmp_feedback_zero_gain_is_reference(self):
        self.assertAlmostEqual(zmp_feedback(0.3, 0.9, 0.1, gain=0.0), 0.3,
                               places=12)

    def test_zmp_feedback_moves_ahead_when_dcm_exceeds(self):
        """ξ 超过参考时应把 ZMP 往前压。"""
        p = zmp_feedback(0.3, 0.9, 0.1, gain=1.0)
        self.assertGreater(p, 0.3)


# ===========================================================================
class TestSummaries(unittest.TestCase):
    def test_summarise_keys(self):
        p = LIPMParams()
        plan = build_symmetric_plan(8, 0.35, 0.5, duty=0.6)
        x0, v0, _s, t0 = solve_periodic(plan, p, dt=0.002)
        traj = simulate(plan, p, x0, v0, dt=0.002, t_start=t0)
        s = summarise(traj)
        for k in ("duration", "samples", "com_travel", "zmp_max_error",
                  "min_margin", "stable"):
            self.assertIn(k, s)

    def test_summarise_track_keys(self):
        p = LIPMParams()
        plan = build_symmetric_plan(8, 0.35, 0.5, duty=0.6)
        traj = track(plan, p, 0.0, 0.0, 0.7, dt=0.002)
        s = summarise_track(traj)
        for k in ("duration", "com_travel", "dcm_max_error", "saturated_ratio",
                  "stable"):
            self.assertIn(k, s)


if __name__ == "__main__":
    unittest.main(verbosity=2)
