"""横向控制基准的单元测试（unittest，无第三方测试依赖）。

运行：
    python -m unittest discover -s tests -v
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.controllers import (  # noqa: E402
    LQRController,
    PurePursuit,
    Stanley,
    kinematic_error_model,
    make_controllers,
)
from src.lqr_solver import is_stable, solve_care_hamiltonian  # noqa: E402
from src.metrics import (  # noqa: E402
    average_ranks,
    convergence_time,
    evaluate,
    rank_records,
)
from src.path import (  # noqa: E402
    Path,
    make_scenarios,
    wrap_angle,
)
from src.simulator import simulate  # noqa: E402
from src.vehicle import (  # noqa: E402
    SteeringActuator,
    VehicleParams,
    curvature_speed_limit,
    initial_dynamic_state,
    rk4_step,
    rk4_step_dynamic,
)


class VehicleModelTests(unittest.TestCase):
    def setUp(self):
        self.params = VehicleParams()

    def test_straight_line_constant_velocity(self):
        state = np.array([0.0, 0.0, 0.0, 5.0])
        for _ in range(100):
            state = rk4_step(state, 0.0, 5.0, 0.02, self.params)
        self.assertAlmostEqual(state[0], 10.0, places=6)
        self.assertAlmostEqual(state[1], 0.0, places=10)
        self.assertAlmostEqual(state[2], 0.0, places=10)
        self.assertAlmostEqual(state[3], 5.0, places=9)

    def test_left_turn_yaw_positive(self):
        state = np.array([0.0, 0.0, 0.0, 4.0])
        for _ in range(50):
            state = rk4_step(state, 0.15, 4.0, 0.02, self.params)
        self.assertGreater(state[2], 0.0)
        self.assertGreater(state[1], 0.0)

    def test_right_turn_yaw_negative(self):
        state = np.array([0.0, 0.0, 0.0, 4.0])
        for _ in range(50):
            state = rk4_step(state, -0.15, 4.0, 0.02, self.params)
        self.assertLess(state[2], 0.0)
        self.assertLess(state[1], 0.0)

    def test_steering_angle_limit(self):
        actuator = SteeringActuator(self.params)
        for _ in range(200):
            delta = actuator.command(2.0, 0.02)
        self.assertAlmostEqual(delta, self.params.max_steer)
        self.assertLessEqual(abs(delta), self.params.max_steer + 1e-12)

    def test_steering_rate_limit(self):
        actuator = SteeringActuator(self.params)
        delta = actuator.command(self.params.max_steer, 0.02)
        self.assertLessEqual(abs(delta), self.params.max_steer_rate * 0.02 + 1e-12)

    def test_zero_speed_no_motion(self):
        state = np.array([1.0, 2.0, 0.4, 0.0])
        new = rk4_step(state, 0.3, 0.0, 0.02, self.params)
        self.assertAlmostEqual(new[0], state[0])
        self.assertAlmostEqual(new[1], state[1])
        self.assertAlmostEqual(new[2], state[2])

    def test_speed_first_order_response(self):
        state = np.array([0.0, 0.0, 0.0, 3.0])
        for _ in range(250):
            state = rk4_step(state, 0.0, 8.0, 0.02, self.params)
        self.assertGreater(state[3], 7.9)
        self.assertLessEqual(state[3], 8.0 + 1e-9)

    def test_curvature_speed_limit(self):
        self.assertAlmostEqual(curvature_speed_limit(0.0, 8.0, 4.0), 8.0)
        self.assertLess(curvature_speed_limit(0.1, 8.0, 4.0), 8.0)
        self.assertAlmostEqual(curvature_speed_limit(0.04, 8.0, 4.0), 8.0)

    def test_dynamic_model_is_finite(self):
        state = initial_dynamic_state(np.array([0.0, 0.0, 0.0, 5.0]))
        for _ in range(100):
            state = rk4_step_dynamic(state, 0.01, 8.0, 0.02, self.params)
        self.assertTrue(np.all(np.isfinite(state)))
        self.assertGreater(state[3], 0.0)


class PathTests(unittest.TestCase):
    def setUp(self):
        self.straight = Path.straight(80.0)
        self.arc = Path.arc(20.0, 80.0)

    def test_straight_tangent_and_curvature(self):
        pose = self.straight.pose_at(25.0)
        self.assertAlmostEqual(pose.x, 25.0)
        self.assertAlmostEqual(pose.y, 0.0)
        self.assertAlmostEqual(pose.psi, 0.0)
        self.assertAlmostEqual(pose.kappa, 0.0)

    def test_straight_projection(self):
        proj = self.straight.project(40.0, 1.2)
        self.assertAlmostEqual(proj.s, 40.0, places=6)
        self.assertAlmostEqual(proj.e, 1.2, places=6)
        self.assertAlmostEqual(proj.distance, 1.2, places=6)

    def test_left_lateral_error_is_positive(self):
        proj = self.straight.project(10.0, 0.7)
        self.assertGreater(proj.e, 0.0)
        proj_right = self.straight.project(10.0, -0.7)
        self.assertLess(proj_right.e, 0.0)

    def test_arc_projection_accuracy(self):
        s_true = 31.0
        pose = self.arc.pose_at(s_true)
        normal = pose.normal_left
        x = pose.x + 0.3 * normal[0]
        y = pose.y + 0.3 * normal[1]
        proj = self.arc.project(x, y, s_hint=s_true)
        self.assertAlmostEqual(proj.s, s_true, places=4)
        self.assertAlmostEqual(proj.e, 0.3, places=4)

    def test_arc_curvature_and_length(self):
        pose = self.arc.pose_at(17.0)
        self.assertAlmostEqual(pose.kappa, 1.0 / 20.0, places=10)
        self.assertAlmostEqual(self.arc.length, 80.0, places=10)
        ds = np.diff(self.arc.s)
        self.assertLess(np.max(np.abs(ds - self.arc.ds)), 1e-10)

    def test_normal_left_is_unit_and_orthogonal(self):
        pose = self.arc.pose_at(12.0)
        normal = pose.normal_left
        tangent = pose.tangent
        self.assertAlmostEqual(np.linalg.norm(normal), 1.0)
        self.assertAlmostEqual(float(np.dot(normal, tangent)), 0.0, places=12)

    def test_pose_interpolation(self):
        pose = self.straight.pose_at(12.345)
        self.assertAlmostEqual(pose.x, 12.345, places=6)
        self.assertAlmostEqual(pose.s, 12.345, places=6)

    def test_sine_is_curved_and_finite(self):
        sine = Path.sine(3.0, 40.0, 100.0)
        self.assertTrue(np.all(np.isfinite(sine.kappa)))
        self.assertGreater(np.max(np.abs(sine.kappa)), 0.02)
        self.assertLess(np.max(np.abs(sine.kappa)), 0.15)

    def test_double_lane_curvature_boundary(self):
        path = Path.double_lane(3.5, 120.0)
        self.assertTrue(np.all(np.isfinite(path.kappa)))
        self.assertLess(np.max(np.abs(path.kappa)), 0.08)
        self.assertAlmostEqual(path.pose_at(0.0).kappa, 0.0, places=6)
        self.assertAlmostEqual(path.pose_at(path.length).kappa, 0.0, places=5)

    def test_common_initial_conditions(self):
        for scenario in make_scenarios().values():
            state = scenario.initial_state()
            proj = scenario.path.project(state[0], state[1], s_hint=scenario.s0)
            self.assertAlmostEqual(proj.e, 1.2, delta=0.02, msg=scenario.key)
            self.assertAlmostEqual(wrap_angle(state[2] - proj.psi), 0.10, delta=0.02,
                                   msg=scenario.key)
            self.assertAlmostEqual(state[3], 3.0, msg=scenario.key)

    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(0.0), 0.0)
        self.assertAlmostEqual(wrap_angle(np.pi + 0.1), -np.pi + 0.1, places=10)
        self.assertAlmostEqual(wrap_angle(-np.pi - 0.1), np.pi - 0.1, places=10)


class PurePursuitTests(unittest.TestCase):
    def setUp(self):
        self.params = VehicleParams()
        self.path = Path.straight(80.0)
        self.controller = PurePursuit(self.params)
        self.proj = self.path.project(0.0, 0.0, s_hint=0.0)
        self.pose = np.array([0.0, 0.0, 0.0, 5.0])

    def test_lookahead_linear_in_speed(self):
        ld1 = self.controller.lookahead_distance(3.0)
        ld2 = self.controller.lookahead_distance(8.0)
        self.assertGreater(ld2, ld1)
        expected = self.controller.ld0 + self.controller.k_ld * 8.0
        self.assertAlmostEqual(ld2, expected)

    def test_on_path_zero_steering(self):
        delta = self.controller.compute(self.pose, self.proj, self.path, 0.02)
        self.assertAlmostEqual(delta, 0.0, places=7)

    def test_left_offset_steers_right(self):
        pose = np.array([0.0, 1.2, 0.0, 5.0])
        proj = self.path.project(0.0, 1.2, s_hint=0.0)
        delta = self.controller.compute(pose, proj, self.path, 0.02)
        self.assertLess(delta, 0.0)

    def test_heading_left_steers_right(self):
        pose = np.array([0.0, 0.0, 0.20, 5.0])
        delta = self.controller.compute(pose, self.proj, self.path, 0.02)
        self.assertLess(delta, 0.0)

    def test_geometry_formula(self):
        pose = np.array([5.0, 0.8, 0.05, 6.0])
        proj = self.path.project(5.0, 0.8, s_hint=5.0)
        ld = self.controller.lookahead_distance(6.0)
        target = self.path.pose_at(proj.s + ld)
        dxw, dyw = target.x - pose[0], target.y - pose[1]
        c, s = np.cos(pose[2]), np.sin(pose[2])
        dx = c * dxw + s * dyw
        dy = -s * dxw + c * dyw
        alpha = math.atan2(dy, dx)
        expected = math.atan2(2.0 * self.params.wheelbase * math.sin(alpha),
                              math.hypot(dx, dy))
        actual = self.controller.compute(pose, proj, self.path, 0.02)
        self.assertAlmostEqual(actual, expected, places=10)


class StanleyTests(unittest.TestCase):
    def setUp(self):
        self.params = VehicleParams()
        self.path = Path.straight(80.0)
        self.flat_proj = self.path.project(0.0, 0.0, s_hint=0.0)

    def test_left_offset_steers_right(self):
        controller = Stanley(self.params)
        pose = np.array([0.0, 1.2, 0.0, 5.0])
        proj = self.path.project(0.0, 1.2, s_hint=0.0)
        delta = controller.compute(pose, proj, self.path, 0.02)
        self.assertLess(delta, 0.0)

    def test_right_offset_steers_left(self):
        controller = Stanley(self.params)
        pose = np.array([0.0, -1.2, 0.0, 5.0])
        proj = self.path.project(0.0, -1.2, s_hint=0.0)
        delta = controller.compute(pose, proj, self.path, 0.02)
        self.assertGreater(delta, 0.0)

    def test_path_heading_left_steers_left(self):
        controller = Stanley(self.params)
        pose = np.array([0.0, 0.0, -0.20, 5.0])
        delta = controller.compute(pose, self.flat_proj, self.path, 0.02)
        self.assertGreater(delta, 0.0)

    def test_curvature_feedforward_sign(self):
        arc = Path.arc(20.0, 80.0)
        controller = Stanley(self.params)
        proj = arc.project(0.0, 0.0, s_hint=0.0)
        pose = np.array([0.0, 0.0, proj.psi, 5.0])
        delta = controller.compute(pose, proj, arc, 0.02)
        self.assertGreater(delta, 0.0)
        self.assertAlmostEqual(delta, math.atan(self.params.wheelbase / 20.0), places=5)

    def test_front_axle_switch_is_finite(self):
        arc = Path.arc(20.0, 80.0)
        controller = Stanley(self.params, use_rear_axle=False)
        proj = arc.project(0.0, 0.0, s_hint=0.0)
        pose = np.array([0.0, 0.0, proj.psi, 5.0])
        delta = controller.compute(pose, proj, arc, 0.02)
        self.assertTrue(np.isfinite(delta))
        self.assertLess(abs(delta), self.params.max_steer + 1e-9)

    def test_zero_speed_is_safe(self):
        controller = Stanley(self.params)
        pose = np.array([0.0, 1.2, 0.0, 0.0])
        proj = self.path.project(0.0, 1.2, s_hint=0.0)
        delta = controller.compute(pose, proj, self.path, 0.02)
        self.assertTrue(np.isfinite(delta))

    def test_rear_axle_arc_steady_state_is_millimetric(self):
        scenario = make_scenarios()["arc"]
        controller = Stanley(self.params, use_rear_axle=True)
        result = simulate(scenario, controller, cruise_speed=8.0)
        self.assertLess(result.metrics["steady_abs"], 0.005)

    def test_front_axle_arc_offset_matches_reference(self):
        scenario = make_scenarios()["arc"]
        controller = Stanley(self.params, use_rear_axle=False)
        result = simulate(scenario, controller, cruise_speed=8.0)
        self.assertGreater(result.metrics["steady_abs"], 0.60)
        self.assertLess(result.metrics["steady_abs"], 0.75)

    def test_front_axle_sine_offset_matches_reference(self):
        scenario = make_scenarios()["sine"]
        controller = Stanley(self.params, use_rear_axle=False)
        result = simulate(scenario, controller, cruise_speed=8.0)
        self.assertGreater(result.metrics["steady_abs"], 0.50)
        self.assertLess(result.metrics["steady_abs"], 0.65)

    def test_rear_axle_sine_steady_state_is_millimetric(self):
        scenario = make_scenarios()["sine"]
        controller = Stanley(self.params, use_rear_axle=True)
        result = simulate(scenario, controller, cruise_speed=8.0)
        self.assertLess(result.metrics["steady_abs"], 0.01)


class LQRTests(unittest.TestCase):
    def setUp(self):
        self.params = VehicleParams()

    def _design(self, v=8.0):
        a, b = kinematic_error_model(v, self.params)
        q = np.diag([1.0, 1.0, 0.05])
        return a, b, solve_care_hamiltonian(a, b, q, np.array([[1.0]]))

    def test_error_model_shape(self):
        a, b = kinematic_error_model(8.0, self.params)
        self.assertEqual(a.shape, (3, 3))
        self.assertEqual(b.shape, (3, 1))
        # ė = v·psi_e
        self.assertAlmostEqual(a[0, 1], 8.0)
        # psi_e_dot 由 v/L 驱动
        self.assertAlmostEqual(b[1, 0], 8.0 / self.params.wheelbase)

    def test_riccati_residual_and_stability(self):
        for v in (3.0, 5.0, 8.0, 12.0):
            a, b, solution = self._design(v)
            self.assertEqual(solution["K"].shape, (1, 3))
            self.assertLess(solution["residual_norm"], 1e-6)
            self.assertTrue(is_stable(a - b @ solution["K"], tolerance=1e-6))

    def test_p_matrix_symmetric_psd(self):
        _, _, solution = self._design()
        p = solution["P"]
        self.assertTrue(np.allclose(p, p.T, atol=1e-9))
        self.assertGreaterEqual(float(np.min(np.linalg.eigvalsh(p))), -1e-8)

    def test_gain_is_finite_and_positive(self):
        controller = LQRController(self.params)
        controller.update_gain(8.0, force=True)
        self.assertEqual(controller.K.shape, (1, 3))
        self.assertTrue(np.all(np.isfinite(controller.K)))
        self.assertTrue(np.all(controller.K > 0.0))
        self.assertLess(controller.riccati_residual, 1e-6)

    def test_gain_changes_with_speed(self):
        controller = LQRController(self.params)
        controller.update_gain(5.0, force=True)
        k_low = controller.K.copy()
        controller.update_gain(12.0, force=True)
        self.assertGreater(float(np.linalg.norm(controller.K - k_low)), 0.05)

    def test_large_error_does_not_wind_up(self):
        controller = LQRController(self.params)
        path = Path.straight(80.0)
        pose = np.array([20.0, 1.2, 0.0, 5.0])
        proj = path.project(20.0, 1.2, s_hint=20.0)
        for _ in range(200):
            controller.compute(pose, proj, path, 0.02)
        self.assertAlmostEqual(controller.integral, 0.0, places=9)

    def test_moderate_error_accumulates_with_limit(self):
        controller = LQRController(self.params)
        path = Path.straight(80.0)
        pose = np.array([20.0, 0.3, 0.0, 8.0])
        proj = path.project(20.0, 0.3, s_hint=20.0)
        for _ in range(200):
            controller.compute(pose, proj, path, 0.02)
        self.assertGreater(controller.integral, 0.0)
        self.assertLessEqual(controller.integral, controller.integral_limit + 1e-12)

    def test_feedback_sign_steers_toward_path(self):
        # 车在路径左侧（e > 0）且航向与路径一致：反馈应给出右转（负）。
        controller = LQRController(self.params)
        controller.update_gain(8.0, force=True)
        path = Path.straight(80.0)
        pose = np.array([0.0, 1.2, 0.0, 8.0])
        proj = path.project(0.0, 1.2, s_hint=0.0)
        delta = controller.compute(pose, proj, path, 0.02)
        self.assertLess(delta, 0.0)

    def test_lqr_closed_loop_is_finite(self):
        scenario = make_scenarios()["double_lane"]
        controller = LQRController(self.params)
        result = simulate(scenario, controller, cruise_speed=8.0)
        self.assertTrue(np.all(np.isfinite(result.e)))
        self.assertLess(result.metrics["max_abs"], 2.0)
        self.assertLess(result.metrics["steady_abs"], 0.05)

    def test_lqr_tracks_sine_without_offroad(self):
        scenario = make_scenarios()["sine"]
        controller = LQRController(self.params)
        result = simulate(scenario, controller, cruise_speed=8.0)
        self.assertLess(result.metrics["max_abs"], 2.0)
        self.assertLess(result.metrics["offroad_ratio"], 0.15)


class MetricsTests(unittest.TestCase):
    def test_zero_error_metrics(self):
        time = np.arange(0.0, 5.0 + 1e-9, 0.02)
        error = np.zeros_like(time)
        metrics = evaluate(time, error, np.zeros_like(time), 0.02)
        self.assertAlmostEqual(metrics["mean_abs"], 0.0)
        self.assertAlmostEqual(metrics["rms"], 0.0)
        self.assertAlmostEqual(metrics["max_abs"], 0.0)
        self.assertAlmostEqual(metrics["steady_abs"], 0.0)
        self.assertAlmostEqual(metrics["convergence_s"], 0.0)

    def test_constant_error_metrics(self):
        time = np.arange(0.0, 1.0 + 1e-9, 0.02)
        error = np.full_like(time, 0.5)
        metrics = evaluate(time, error, np.zeros_like(time), 0.02)
        self.assertAlmostEqual(metrics["mean_abs"], 0.5)
        self.assertAlmostEqual(metrics["rms"], 0.5)
        self.assertAlmostEqual(metrics["max_abs"], 0.5)
        self.assertAlmostEqual(metrics["steady_abs"], 0.5)
        self.assertAlmostEqual(metrics["convergence_s"], 1.0)

    def test_convergence_first_hold(self):
        time = np.arange(0.0, 5.0 + 1e-9, 0.02)
        error = np.full_like(time, 1.0)
        error[time >= 2.0] = 0.1
        self.assertAlmostEqual(convergence_time(time, error), 2.0, places=9)

    def test_convergence_reentry(self):
        time = np.arange(0.0, 5.0 + 1e-9, 0.02)
        error = np.full_like(time, 1.0)
        error[(time >= 1.0) & (time < 2.0)] = 0.1
        error[(time >= 2.0) & (time < 3.0)] = 1.0
        error[time >= 3.0] = 0.1
        self.assertAlmostEqual(convergence_time(time, error), 3.0, places=9)

    def test_never_converged_returns_duration(self):
        time = np.arange(0.0, 4.0 + 1e-9, 0.02)
        error = np.full_like(time, 0.5)
        self.assertAlmostEqual(convergence_time(time, error), 4.0, places=9)

    def test_steady_uses_last_quarter(self):
        time = np.arange(0.0, 4.0 + 1e-9, 0.02)
        error = np.ones_like(time)
        error[time >= 3.0] = 0.2
        metrics = evaluate(time, error, np.zeros_like(time), 0.02)
        self.assertAlmostEqual(metrics["steady_abs"], 0.2, places=9)

    def test_offroad_ratio(self):
        time = np.arange(0.0, 1.0 + 1e-9, 0.02)
        error = np.full_like(time, 1.0)
        metrics = evaluate(time, error, np.zeros_like(time), 0.02,
                           offroad_threshold=0.8)
        self.assertAlmostEqual(metrics["offroad_ratio"], 1.0)

    def test_invalid_shapes_raise(self):
        with self.assertRaises(ValueError):
            evaluate(np.zeros(4), np.zeros(3), np.zeros(4), 0.02)
        with self.assertRaises(ValueError):
            evaluate(np.zeros(1), np.zeros(1), np.zeros(1), 0.02)

    def test_nonfinite_raises(self):
        error = np.zeros(10)
        error[3] = np.nan
        with self.assertRaises(ValueError):
            evaluate(np.arange(10.0), error, np.zeros(10), 0.02)

    def test_average_ranks_with_ties(self):
        ranks = average_ranks([1.0, 1.0, 3.0])
        self.assertAlmostEqual(ranks[0], 1.5)
        self.assertAlmostEqual(ranks[1], 1.5)
        self.assertAlmostEqual(ranks[2], 3.0)


class IntegrationTests(unittest.TestCase):
    def test_all_scenarios_all_controllers_finite_and_limited(self):
        params = VehicleParams()
        scenarios = make_scenarios()
        controllers = make_controllers(params)
        for scenario in scenarios.values():
            for controller in controllers:
                result = simulate(scenario, controller, cruise_speed=8.0)
                self.assertTrue(np.all(np.isfinite(result.x)))
                self.assertTrue(np.all(np.isfinite(result.y)))
                self.assertTrue(np.all(np.isfinite(result.psi)))
                self.assertTrue(np.all(np.isfinite(result.delta)))
                self.assertLessEqual(float(np.max(np.abs(result.delta))),
                                     params.max_steer + 1e-12)
                rate = np.diff(result.delta) / 0.02
                self.assertLessEqual(float(np.max(np.abs(rate))),
                                     params.max_steer_rate + 1e-9)

    def test_dynamic_model_run_is_finite(self):
        params = VehicleParams()
        scenario = make_scenarios()["straight"]
        controller = PurePursuit(params)
        result = simulate(scenario, controller, model="dynamic", cruise_speed=8.0)
        self.assertTrue(np.all(np.isfinite(result.e)))
        self.assertLess(result.metrics["max_abs"], 2.0)

    def test_ranking_is_a_permutation(self):
        params = VehicleParams()
        scenarios = make_scenarios()
        results = []
        for scenario in scenarios.values():
            for controller in make_controllers(params):
                result = simulate(scenario, controller, cruise_speed=8.0)
                row = {"scenario": result.scenario, "controller": result.controller,
                       "label": result.label}
                row.update(result.metrics)
                results.append(row)
        ranked, summary = rank_records(results)
        self.assertEqual(len(ranked), 12)
        self.assertEqual(sorted(item["overall_rank"] for item in summary), [1, 2, 3])
        self.assertEqual(sorted(item["scenario_rank"] for item in ranked if item["scenario"] == "sine"),
                         [1, 2, 3])


if __name__ == "__main__":
    unittest.main(verbosity=2)
