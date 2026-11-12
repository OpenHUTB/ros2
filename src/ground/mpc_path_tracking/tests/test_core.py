"""Unit tests for the MPC tracking module (NumPy only)."""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from mpc_path_tracking.core import (
    MpcConfig,
    Unicycle,
    linearize,
    mpc_command,
    solve_box_qp,
)
from mpc_path_tracking.headless import run_experiment
from mpc_path_tracking.scene import (
    ReferencePath,
    TrackConfig,
    run_mpc,
    run_pure_pursuit,
    tracking_metrics,
)


class SolverTests(unittest.TestCase):
    def test_unconstrained_solution_matches_the_analytic_one(self):
        hessian = np.array([[4.0, 1.0], [1.0, 3.0]])
        gradient = np.array([1.0, -2.0])
        solution = solve_box_qp(hessian, gradient, [-10.0] * 2, [10.0] * 2)
        expected = -np.linalg.solve(hessian, gradient)
        np.testing.assert_allclose(solution, expected, atol=1e-6)

    def test_solution_respects_the_box(self):
        hessian = np.eye(2)
        gradient = np.array([-5.0, 5.0])
        solution = solve_box_qp(hessian, gradient, [-1.0, -1.0], [1.0, 1.0])
        np.testing.assert_allclose(solution, [1.0, -1.0], atol=1e-9)

    def test_inconsistent_dimensions_are_rejected(self):
        with self.assertRaises(ValueError):
            solve_box_qp(np.eye(2), np.zeros(3), np.zeros(3), np.ones(3))


class ModelTests(unittest.TestCase):
    def test_straight_line_step(self):
        state = Unicycle().step((0.0, 0.0, 0.0), (1.0, 0.0), 0.1)
        np.testing.assert_allclose(state, [0.1, 0.0, 0.0], atol=1e-12)

    def test_linearization_matches_finite_differences(self):
        state = np.array([0.0, 0.0, 0.3])
        control = np.array([0.8, 0.2])
        jacobian_state, jacobian_control = linearize(state, control, 0.1)
        step = 1e-6
        numeric_state = np.column_stack(
            [
                (Unicycle().step(state + step * np.eye(3)[i], control, 0.1)
                 - Unicycle().step(state, control, 0.1)) / step
                for i in range(3)
            ]
        )
        numeric_control = np.column_stack(
            [
                (Unicycle().step(state, control + step * np.eye(2)[i], 0.1)
                 - Unicycle().step(state, control, 0.1)) / step
                for i in range(2)
            ]
        )
        np.testing.assert_allclose(jacobian_state, numeric_state, atol=1e-5)
        np.testing.assert_allclose(jacobian_control, numeric_control, atol=1e-5)


class PathTests(unittest.TestCase):
    def test_sample_is_periodic(self):
        path = ReferencePath()
        first, _ = path.sample(0.0)
        again, _ = path.sample(path.length)
        np.testing.assert_allclose(first, again, atol=1e-6)

    def test_projection_of_a_point_on_the_path_is_exact(self):
        path = ReferencePath()
        sample, _ = path.sample(10.0)
        self.assertAlmostEqual(path.project(sample[0][:2], 10.0, window=2.0), 10.0, delta=0.1)


class ControllerTests(unittest.TestCase):
    def test_mpc_holds_the_path_from_an_exact_start(self):
        path = ReferencePath()
        track = TrackConfig(duration_s=12.0, lateral_offset=0.0, heading_offset_deg=0.0)
        metrics = tracking_metrics(path, run_mpc(path, track, MpcConfig()))
        self.assertLess(metrics["lateral_steady_rmse_m"], 0.05)

    def test_pure_pursuit_stays_within_the_limits(self):
        path = ReferencePath()
        run = run_pure_pursuit(path, TrackConfig(duration_s=10.0))
        self.assertLessEqual(float(np.max(np.abs(run["controls"][:, 1]))), 1.2 + 1e-9)
        self.assertLessEqual(float(np.max(np.abs(run["controls"][:, 0]))), 1.0 + 1e-9)

    def test_mpc_command_respects_the_absolute_limits(self):
        config = MpcConfig()
        reference_states = np.array([[index * 0.09, 0.0, 0.0] for index in range(1, 26)])
        reference_controls = np.array([[0.9, 0.0] for _ in range(25)])
        command, _ = mpc_command(
            reference_states, reference_controls, np.array([0.0, 1.0, 0.0]),
            np.zeros(2), config,
        )
        self.assertLessEqual(abs(command[0]), config.max_speed + 1e-9)
        self.assertLessEqual(abs(command[1]), config.max_yaw_rate + 1e-9)


class ExperimentTests(unittest.TestCase):
    def test_headless_writes_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            summary = run_experiment(
                out_dir=Path(directory), make_plots=False, duration_s=8.0
            )
            stored = json.loads(
                (Path(directory) / "summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(stored["module"], "mpc_path_tracking")
            self.assertIn("lateral_rmse_m", summary["mpc"])
            self.assertLessEqual(
                summary["mpc"]["yaw_rate_abs_max"], MpcConfig().max_yaw_rate + 1e-9
            )


if __name__ == "__main__":
    unittest.main()
