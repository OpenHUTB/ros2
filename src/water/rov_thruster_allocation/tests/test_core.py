"""Unit tests for the thruster allocation module (NumPy only)."""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from rov_thruster_allocation.core import (
    DEFAULT_THRUSTERS,
    allocate_pseudo_inverse,
    allocate_qp,
    configuration_matrix,
    effort,
    error_norm,
    limits,
    at_limit_ratio,
    solve_box_qp,
    wrench_of,
)
from rov_thruster_allocation.headless import run_experiment
from rov_thruster_allocation.scene import DemandConfig, make_demands


class MatrixTests(unittest.TestCase):
    def test_configuration_matrix_is_full_rank(self):
        matrix = configuration_matrix(DEFAULT_THRUSTERS)
        self.assertEqual(matrix.shape, (6, 8))
        self.assertEqual(np.linalg.matrix_rank(matrix), 6)
        self.assertLess(float(np.linalg.cond(matrix)), 100.0)

    def test_column_contains_force_and_moment(self):
        column = DEFAULT_THRUSTERS[0].column()
        self.assertEqual(column.shape, (6,))
        self.assertAlmostEqual(float(np.linalg.norm(column[:3])), 1.0, places=9)

    def test_zero_direction_is_rejected(self):
        from rov_thruster_allocation.core import Thruster

        with self.assertRaises(ValueError):
            Thruster("bad", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)).column()


class AllocationTests(unittest.TestCase):
    def setUp(self):
        self.matrix = configuration_matrix(DEFAULT_THRUSTERS)
        self.limit = limits(DEFAULT_THRUSTERS)

    def test_pseudo_inverse_reproduces_a_feasible_wrench(self):
        forces = np.array([10.0, -5.0, 20.0, 0.0, 5.0, -5.0, 15.0, -10.0])
        demand = wrench_of(self.matrix, forces)
        recovered = allocate_pseudo_inverse(self.matrix, demand)
        self.assertLess(error_norm(self.matrix, recovered, demand), 1e-9)

    def test_qp_respects_the_thruster_limits(self):
        demand = np.array([400.0, 200.0, 300.0, 40.0, 30.0, 20.0])
        forces = allocate_qp(self.matrix, demand, self.limit)
        self.assertTrue(np.all(np.abs(forces) <= self.limit + 1e-9))

    def test_qp_beats_clipping_outside_the_envelope(self):
        rng = np.random.default_rng(0)
        worse = []
        better = []
        for _ in range(20):
            demand = rng.normal(0.0, 300.0, 6)
            clipped = allocate_pseudo_inverse(self.matrix, demand, self.limit)
            optimised = allocate_qp(self.matrix, demand, self.limit)
            worse.append(error_norm(self.matrix, clipped, demand))
            better.append(error_norm(self.matrix, optimised, demand))
        self.assertLess(float(np.mean(better)), float(np.mean(worse)))

    def test_at_limit_and_effort(self):
        forces = np.array([40.0, 0.0, -40.0])
        self.assertAlmostEqual(at_limit_ratio(forces, np.full(3, 40.0)), 2.0 / 3.0)
        self.assertAlmostEqual(effort(np.array([3.0, 4.0])), 25.0)


class SolverTests(unittest.TestCase):
    def test_box_qp_matches_the_analytic_solution(self):
        hessian = np.array([[2.0, 0.0], [0.0, 4.0]])
        gradient = np.array([2.0, -8.0])
        solution = solve_box_qp(hessian, gradient, [-10.0, -10.0], [10.0, 10.0])
        np.testing.assert_allclose(solution, [-1.0, 2.0], atol=1e-6)

    def test_box_qp_clips(self):
        solution = solve_box_qp(np.eye(2), np.array([-5.0, 5.0]), [-1.0, -1.0], [1.0, 1.0])
        np.testing.assert_allclose(solution, [1.0, -1.0], atol=1e-9)


class DemandTests(unittest.TestCase):
    def test_demands_are_reproducible(self):
        first = make_demands(DemandConfig(seed=4))
        second = make_demands(DemandConfig(seed=4))
        np.testing.assert_array_equal(first["inside_wrenches"], second["inside_wrenches"])

    def test_outside_demands_are_scaled(self):
        data = make_demands(DemandConfig(outside_scale=2.0, seed=1))
        ratio = np.linalg.norm(data["outside_wrenches"][0]) / np.linalg.norm(
            data["inside_wrenches"][0]
        )
        self.assertAlmostEqual(float(ratio), 2.0, places=9)


class ExperimentTests(unittest.TestCase):
    def test_headless_writes_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            summary = run_experiment(out_dir=Path(directory), make_plots=False)
            stored = json.loads(
                (Path(directory) / "summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(stored["module"], "rov_thruster_allocation")
            self.assertLess(stored["inside"]["pseudo_inverse"]["wrench_error_rms"], 1e-9)
            self.assertLess(
                summary["outside"]["qp_allocation"]["wrench_error_rms"],
                summary["outside"]["clipped_pseudo_inverse"]["wrench_error_rms"],
            )


if __name__ == "__main__":
    unittest.main()
