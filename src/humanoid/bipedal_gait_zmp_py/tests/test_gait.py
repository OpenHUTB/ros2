"""双足步态与 ZMP 稳定性测试。

运行：python -m pytest -q
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.dcm import clip_zmp, dcm, dcm_reference, stability_margin, zmp_command
from scripts.footstep import FootstepPlan
from scripts.gait import (
    _integrate_nominal,
    compute_periodic_solution,
    result_to_row,
    run_open_loop_instability,
    simulate_gait,
)
from scripts.lipm import (
    G,
    OMEGA,
    Z_C,
    analytic_constant_zmp,
    integrate_constant_zmp,
    lipm_acceleration,
    lipm_step,
    second_derivative_error,
    zmp_from_acceleration,
)


# ---------------------------------------------------------------------------
# LIPM
# ---------------------------------------------------------------------------


def test_omega_from_com_height():
    assert OMEGA == pytest.approx(math.sqrt(G / Z_C))
    assert OMEGA == pytest.approx(3.5018, abs=1e-3)


def test_analytic_matches_numeric_integration():
    x0, v0, p = 0.13, 0.78, 0.20
    x_num, v_num = integrate_constant_zmp(x0, v0, p, duration=1.0, dt=1e-5)
    x_ana, v_ana = analytic_constant_zmp(x0, v0, p, np.array([1.0]))
    assert float(x_ana[0]) == pytest.approx(x_num, abs=1e-9)
    assert float(v_ana[0]) == pytest.approx(v_num, abs=1e-9)


def test_analytic_obeys_lipm_equation():
    assert second_derivative_error(0.1, 0.5, 0.2, duration=1.0, dt=1e-4) < 2e-3


def test_zmp_round_trip():
    x, x_ddot = 0.4, 1.3
    p = zmp_from_acceleration(x, x_ddot)
    assert lipm_acceleration(x, p) == pytest.approx(x_ddot)


def test_upright_pendulum_is_unstable():
    # 质心在 ZMP 前方 -> 加速度朝正方向（越跑越远），不是被拉回。
    assert lipm_acceleration(0.1, 0.0) > 0.0
    x, _ = lipm_step(0.1, 0.0, 0.0, 0.1)
    assert x > 0.1


def test_zero_error_is_equilibrium():
    x, v = lipm_step(0.3, 0.0, 0.3, 0.5)
    assert x == pytest.approx(0.3)
    assert v == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# 落脚点与支撑多边形
# ---------------------------------------------------------------------------


@pytest.fixture
def plan():
    return FootstepPlan()


def test_plan_geometry(plan):
    assert plan.duration == pytest.approx(6.1)
    assert plan.stance_duration == pytest.approx(0.6)
    assert plan.support_half_width == pytest.approx(0.12)


def test_single_support_state(plan):
    state = plan.support_state(0.25)
    assert state.phase == "single"
    assert state.zmp_ref == pytest.approx(0.0)
    assert (state.left, state.right) == pytest.approx((-0.12, 0.12))


def test_double_support_state(plan):
    state = plan.support_state(0.55)
    assert state.phase == "double"
    assert (state.left, state.right) == pytest.approx((-0.12, 0.47))
    assert state.zmp_ref == pytest.approx(0.175)


def test_alternating_feet(plan):
    sides = [s.side for s in plan.steps]
    assert sides[:4] == ["L", "R", "L", "R"]
    assert all(plan.steps[i].x == pytest.approx(0.35 * i) for i in range(plan.n_steps))


def test_zmp_reference_continuous_at_phase_switch(plan):
    t_switch = 0.6 - 1e-9
    before = plan.support_state(t_switch).zmp_ref
    after = plan.support_state(t_switch + 2e-9).zmp_ref
    assert before == pytest.approx(after, abs=0.05)


def test_stance_interval_contains_land_and_lift(plan):
    for step in plan.steps:
        assert step.lift_time - step.land_time == pytest.approx(plan.stance_duration)


def test_foot_pose_by_side_alternates(plan):
    # 左右脚各有一个最近落点，t=0.25 时左脚支撑、右脚尚未落地。
    lx, lz, lstance = plan.foot_pose_by_side("L", 0.25)
    rx, rz, rstance = plan.foot_pose_by_side("R", 0.25)
    assert (lx, lz) == pytest.approx((0.0, 0.0))
    assert lstance
    assert rx < 0.35  # 右脚还在摆动/未到落点


# ---------------------------------------------------------------------------
# DCM 与稳定裕度
# ---------------------------------------------------------------------------


def test_dcm_definition():
    assert dcm(1.0, OMEGA) == pytest.approx(2.0)
    assert dcm(1.0, 0.0) == pytest.approx(1.0)


def test_dcm_reference_uses_average_speed():
    p_ref, v_avg = 0.35, 0.7
    assert dcm_reference(p_ref, v_avg) == pytest.approx(p_ref + v_avg / OMEGA)


def test_zmp_command_zero_error_gives_reference():
    assert zmp_command(0.5, 0.5, 0.35, gain=2.0) == pytest.approx(0.35)


def test_clip_zmp():
    assert clip_zmp(0.3, -0.12, 0.12) == pytest.approx(0.12)
    assert clip_zmp(-0.3, -0.12, 0.12) == pytest.approx(-0.12)
    with pytest.raises(ValueError):
        clip_zmp(0.0, 0.2, 0.1)


def test_stability_margin_sign():
    assert stability_margin(0.0, -0.1, 0.2) == pytest.approx(0.1)
    assert stability_margin(-0.1, -0.1, 0.2) == pytest.approx(0.0)
    assert stability_margin(-0.2, -0.1, 0.2) == pytest.approx(-0.1)


# ---------------------------------------------------------------------------
# 周期解与开环不稳定性
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def periodic():
    return compute_periodic_solution(FootstepPlan(), dt=5e-4)


def test_periodic_solution_is_fixed_point(periodic):
    assert periodic.residual < 1e-9
    assert periodic.end_state[0] == pytest.approx(
        periodic.x0 + periodic.displacement, abs=1e-8
    )
    assert periodic.end_state[1] == pytest.approx(periodic.v0, abs=1e-9)


def test_periodic_solution_speed_close_to_average(periodic):
    assert periodic.v0 == pytest.approx(0.7, abs=0.15)
    assert periodic.trace > 10.0  # 放大矩阵的迹远大于 1，说明周期解不稳定


def test_open_loop_growth_ratio(periodic):
    result = run_open_loop_instability(FootstepPlan(), periodic, cycles=5, dt=1e-3)
    assert result.predicted_ratio == pytest.approx(math.exp(OMEGA * periodic.period))
    tail = float(np.mean(result.ratios[-2:]))
    assert tail == pytest.approx(result.predicted_ratio, rel=0.02)


# ---------------------------------------------------------------------------
# 闭环
# ---------------------------------------------------------------------------


def test_zmp_always_inside_support_polygon():
    result = simulate_gait(FootstepPlan(), gain=1.0, dt=2e-3)
    assert np.all(result.zmp >= result.support_left - 1e-9)
    assert np.all(result.zmp <= result.support_right + 1e-9)
    assert result.min_margin >= -1e-9


def test_default_gain_walks():
    result = simulate_gait(FootstepPlan(), gain=1.0, dt=2e-3)
    assert result.stable
    assert result.travel == pytest.approx(3.93, abs=0.15)
    assert result.avg_speed == pytest.approx(0.64, abs=0.05)
    assert 0.05 < result.saturation_ratio < 0.35


def test_open_loop_gain_zero_diverges():
    result = simulate_gait(FootstepPlan(), gain=0.0, dt=2e-3)
    assert not result.stable
    assert abs(result.travel) > 1.0


def test_higher_gain_increases_saturation():
    low = simulate_gait(FootstepPlan(), gain=1.0, dt=2e-3)
    high = simulate_gait(FootstepPlan(), gain=3.0, dt=2e-3)
    assert low.saturation_ratio < high.saturation_ratio
    assert low.mean_margin > high.mean_margin


def test_result_arrays_are_finite_and_aligned():
    result = simulate_gait(FootstepPlan(), gain=1.5, dt=2e-3)
    for name in ("time", "x", "v", "zmp", "margin", "dcm"):
        values = getattr(result, name)
        assert values.ndim == 1
        assert np.all(np.isfinite(values))
    assert result.time.size == result.x.size == result.zmp.size


def test_downsample_keeps_metrics():
    result = simulate_gait(FootstepPlan(), gain=1.0, dt=2e-3)
    small = result.downsample(10)
    assert small.time.size <= result.time.size // 10 + 2
    assert small.travel == pytest.approx(result.travel)

# ---------------------------------------------------------------------------
# 以下为 unittest 用例，保证 `python -m unittest discover -s tests` 可独立运行。
# pytest 也会自动收集这些 TestCase。
# ---------------------------------------------------------------------------


class TestLIPMUnittest(unittest.TestCase):
    def test_omega_value(self) -> None:
        self.assertAlmostEqual(OMEGA, math.sqrt(G / Z_C), places=12)
        self.assertAlmostEqual(OMEGA, 3.5017852589, places=9)

    def test_zmp_inverse_of_lipm_acceleration(self) -> None:
        x, p = 0.37, -0.08
        self.assertAlmostEqual(zmp_from_acceleration(x, lipm_acceleration(x, p)), p, places=12)

    def test_analytic_initial_conditions(self) -> None:
        x0, v0, p = 0.12, 0.45, -0.03
        x, v = analytic_constant_zmp(x0, v0, p, np.array([0.0]))
        self.assertAlmostEqual(float(x[0]), x0, places=12)
        self.assertAlmostEqual(float(v[0]), v0, places=12)

    def test_analytic_position_matches_formula(self) -> None:
        x0, v0, p, t = 0.12, 0.45, -0.03, 0.37
        x, _ = analytic_constant_zmp(x0, v0, p, t)
        expected = (
            (x0 - p) * math.cosh(OMEGA * t)
            + (v0 / OMEGA) * math.sinh(OMEGA * t)
            + p
        )
        self.assertAlmostEqual(float(x), expected, places=12)

    def test_analytic_velocity_matches_x_derivative(self) -> None:
        x0, v0, p, t, h = 0.12, 0.45, -0.03, 0.4, 1e-5
        x_plus, _ = analytic_constant_zmp(x0, v0, p, t + h)
        x_minus, _ = analytic_constant_zmp(x0, v0, p, t - h)
        _, v = analytic_constant_zmp(x0, v0, p, t)
        self.assertAlmostEqual((float(x_plus) - float(x_minus)) / (2.0 * h), float(v), places=7)

    def test_constant_zmp_numerical_integration(self) -> None:
        x_num, v_num = integrate_constant_zmp(0.2, 0.1, 0.05, 0.6, dt=1e-3)
        x_exact, v_exact = analytic_constant_zmp(0.2, 0.1, 0.05, 0.6)
        self.assertAlmostEqual(x_num, float(x_exact), places=10)
        self.assertAlmostEqual(v_num, float(v_exact), places=10)

    def test_second_derivative_residual_is_small(self) -> None:
        self.assertLess(second_derivative_error(0.1, 0.3, 0.0, 0.5), 1e-5)

    def test_lipm_step_reversible(self) -> None:
        x1, v1 = lipm_step(0.1, 0.7, 0.0, 0.01)
        x2, v2 = lipm_step(x1, v1, 0.0, -0.01)
        self.assertAlmostEqual(x2, 0.1, places=11)
        self.assertAlmostEqual(v2, 0.7, places=11)


class TestFootstepPlanUnittest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.plan = FootstepPlan()

    def test_parameter_values(self) -> None:
        self.assertEqual(self.plan.n_steps, 12)
        self.assertAlmostEqual(self.plan.step_length, 0.35)
        self.assertAlmostEqual(self.plan.step_time, 0.5)
        self.assertAlmostEqual(self.plan.stance_ratio, 0.6)

    def test_stance_duration_and_duration(self) -> None:
        self.assertAlmostEqual(self.plan.stance_duration, 0.6)
        self.assertAlmostEqual(self.plan.duration, 6.1)

    def test_foot_spacing(self) -> None:
        xs = [step.x for step in self.plan.steps]
        np.testing.assert_allclose(np.diff(xs), 0.35, atol=1e-12)
        self.assertAlmostEqual(xs[0], 0.0)

    def test_feet_alternate(self) -> None:
        sides = [step.side for step in self.plan.steps]
        self.assertEqual(sides[:4], ["L", "R", "L", "R"])
        self.assertTrue(all(s == ("L" if i % 2 == 0 else "R") for i, s in enumerate(sides)))

    def test_half_length_and_effective_width(self) -> None:
        self.assertAlmostEqual(self.plan.half_length, 0.09)
        self.assertAlmostEqual(self.plan.zmp_edge_margin, 0.03)
        self.assertAlmostEqual(self.plan.support_half_width, 0.12)

    def test_single_support_interval(self) -> None:
        state = self.plan.support_state(0.2)
        self.assertEqual(state.phase, "single")
        self.assertAlmostEqual(state.left, -0.12)
        self.assertAlmostEqual(state.right, 0.12)
        self.assertAlmostEqual(state.zmp_ref, 0.0)

    def test_double_support_is_union(self) -> None:
        state = self.plan.support_state(0.55)
        self.assertEqual(state.phase, "double")
        self.assertEqual(state.stance_indices, (0, 1))
        self.assertAlmostEqual(state.left, -0.12)
        self.assertAlmostEqual(state.right, 0.47)

    def test_double_support_reference_midpoint(self) -> None:
        self.assertAlmostEqual(self.plan.support_state(0.55).zmp_ref, 0.175)

    def test_zmp_reference_continuity_at_lift(self) -> None:
        before = self.plan.support_state(0.6 - 1e-9).zmp_ref
        after = self.plan.support_state(0.6 + 1e-9).zmp_ref
        self.assertAlmostEqual(before, 0.35, places=7)
        self.assertAlmostEqual(after, 0.35, places=7)

    def test_reference_always_inside_support(self) -> None:
        for t in np.linspace(0.0, self.plan.duration, 2000):
            state = self.plan.support_state(float(t))
            self.assertGreaterEqual(state.zmp_ref, state.left - 1e-12)
            self.assertLessEqual(state.zmp_ref, state.right + 1e-12)

    def test_swing_foot_leaves_ground(self) -> None:
        x_mid, z_mid, stance = self.plan.foot_pose(0, 0.8)
        self.assertFalse(stance)
        self.assertGreater(z_mid, 0.0)
        self.assertGreater(x_mid, 0.0)
        self.assertLess(x_mid, 0.7)

    def test_support_feet_count(self) -> None:
        self.assertEqual(len(self.plan.active_steps(0.2)), 1)
        self.assertEqual(len(self.plan.active_steps(0.5)), 2)
        self.assertEqual(len(self.plan.active_steps(0.7)), 1)

    def test_invalid_parameters_raise(self) -> None:
        with self.assertRaises(ValueError):
            FootstepPlan(n_steps=1)
        with self.assertRaises(ValueError):
            FootstepPlan(stance_ratio=0.5)


class TestDCMControllerUnittest(unittest.TestCase):
    def test_dcm_definition(self) -> None:
        self.assertAlmostEqual(dcm(1.2, 0.7), 1.2 + 0.7 / OMEGA)

    def test_dcm_reference_definition(self) -> None:
        self.assertAlmostEqual(dcm_reference(0.35, 0.7), 0.35 + 0.7 / OMEGA)

    def test_zmp_command_formula(self) -> None:
        self.assertAlmostEqual(zmp_command(0.5, 0.4, 0.2, 1.5), 0.35)

    def test_clip_and_margin(self) -> None:
        self.assertEqual(clip_zmp(0.5, -0.1, 0.2), 0.2)
        self.assertAlmostEqual(stability_margin(0.0, -0.1, 0.2), 0.1)
        self.assertAlmostEqual(stability_margin(0.2, -0.1, 0.2), 0.0)


class TestPeriodicSolutionUnittest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.plan = FootstepPlan()
        cls.periodic = compute_periodic_solution(cls.plan)

    def test_affine_map_shape(self) -> None:
        self.assertEqual(self.periodic.A.shape, (2, 2))
        self.assertEqual(self.periodic.b.shape, (2,))

    def test_affine_trace_is_two_cosh(self) -> None:
        expected = 2.0 * math.cosh(OMEGA * self.periodic.period)
        self.assertAlmostEqual(self.periodic.trace, expected, places=8)

    def test_periodic_displacement(self) -> None:
        self.assertAlmostEqual(
            float(self.periodic.end_state[0] - self.periodic.x0),
            self.periodic.displacement,
            places=9,
        )

    def test_periodic_velocity(self) -> None:
        self.assertAlmostEqual(float(self.periodic.end_state[1]), self.periodic.v0, places=9)

    def test_periodic_residual(self) -> None:
        self.assertLess(self.periodic.residual, 1e-10)

    def test_map_matches_forward_integration(self) -> None:
        x, v = _integrate_nominal(
            self.plan,
            self.periodic.x0,
            self.periodic.v0,
            self.periodic.t_start,
            self.periodic.period,
            5e-4,
        )
        mapped = self.periodic.A @ np.array([self.periodic.x0, self.periodic.v0]) + self.periodic.b
        self.assertAlmostEqual(x, float(mapped[0]), places=9)
        self.assertAlmostEqual(v, float(mapped[1]), places=9)


class TestOpenLoopInstabilityUnittest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.plan = FootstepPlan()
        cls.periodic = compute_periodic_solution(cls.plan)
        cls.instability = run_open_loop_instability(cls.plan, cls.periodic)

    def test_velocity_error_grows_every_cycle(self) -> None:
        errors = np.abs(self.instability.cycle_errors)
        self.assertTrue(np.all(np.diff(errors) > 0.0))

    def test_growth_ratio_matches_exponential(self) -> None:
        tail = self.instability.ratios[-2:]
        relative_error = np.abs(tail - self.instability.predicted_ratio) / self.instability.predicted_ratio
        self.assertTrue(np.all(relative_error < 0.01))

    def test_ratio_is_about_33(self) -> None:
        mean_tail = float(np.mean(self.instability.ratios[-2:]))
        self.assertGreater(mean_tail, 32.0)
        self.assertLess(mean_tail, 34.0)

    def test_five_cycles_are_recorded(self) -> None:
        self.assertEqual(len(self.instability.cycle_errors), 5)
        self.assertEqual(len(self.instability.ratios), 4)


class TestClosedLoopGaitUnittest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.plan = FootstepPlan()
        cls.k0 = simulate_gait(cls.plan, 0.0)
        cls.k1 = simulate_gait(cls.plan, 1.0)
        cls.k15 = simulate_gait(cls.plan, 1.5)
        cls.k3 = simulate_gait(cls.plan, 3.0)

    def test_no_nan_or_inf(self) -> None:
        for result in (self.k0, self.k1, self.k15, self.k3):
            for values in (result.time, result.x, result.v, result.zmp, result.dcm, result.margin):
                self.assertTrue(np.all(np.isfinite(values)))

    def test_k0_diverges_to_huge_negative(self) -> None:
        self.assertLess(self.k0.travel, -1e6)
        self.assertFalse(self.k0.stable)
        self.assertGreater(abs(self.k0.final_velocity), 1e5)

    def test_k1_travel(self) -> None:
        self.assertAlmostEqual(self.k1.travel, 3.9311, delta=0.02)

    def test_k1_average_speed(self) -> None:
        self.assertAlmostEqual(self.k1.avg_speed, 0.6444, delta=0.005)

    def test_k1_dcm_error(self) -> None:
        self.assertAlmostEqual(self.k1.dcm_max_error, 0.2314, delta=0.005)

    def test_k1_zmp_saturation(self) -> None:
        self.assertAlmostEqual(self.k1.saturation_ratio, 0.2014, delta=0.01)

    def test_k1_is_stable(self) -> None:
        self.assertTrue(self.k1.stable)
        self.assertGreater(self.k1.travel, 3.5)

    def test_margin_nonnegative(self) -> None:
        for result in (self.k0, self.k1, self.k15, self.k3):
            self.assertGreaterEqual(float(np.min(result.margin)), -1e-12)

    def test_applied_zmp_inside_support(self) -> None:
        for result in (self.k1, self.k15, self.k3):
            self.assertTrue(np.all(result.zmp >= result.support_left - 1e-12))
            self.assertTrue(np.all(result.zmp <= result.support_right + 1e-12))

    def test_k1_min_margin_touches_boundary(self) -> None:
        self.assertAlmostEqual(self.k1.min_margin, 0.0, places=9)
        self.assertGreater(self.k1.mean_margin, 0.05)

    def test_gain_threshold(self) -> None:
        low = simulate_gait(self.plan, 0.6)
        self.assertFalse(low.stable)
        self.assertTrue(self.k1.stable)
        self.assertTrue(self.k15.stable)
        self.assertTrue(self.k3.stable)

    def test_k3_walks_slightly_faster(self) -> None:
        self.assertGreater(self.k3.travel, self.k1.travel)

    def test_result_row_fields(self) -> None:
        row = result_to_row(self.k1)
        self.assertIn("travel_m", row)
        self.assertIn("saturation_ratio", row)
        self.assertTrue(row["stable"])

    def test_time_grid(self) -> None:
        self.assertAlmostEqual(self.k1.time[0], 0.0)
        self.assertAlmostEqual(self.k1.time[-1], self.plan.duration)
        self.assertEqual(len(self.k1.time), 6101)

    def test_phase_labels(self) -> None:
        self.assertEqual(set(self.k1.phase.tolist()), {"single", "double"})

    def test_k1_zmp_reference_monotonic_overall(self) -> None:
        self.assertGreaterEqual(float(np.min(np.diff(self.k1.zmp_ref))), -1e-9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
