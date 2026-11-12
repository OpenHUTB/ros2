import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_dwa_avoidance"))

from dwa_core import DWA, DWAParams, RobotState, scan_points_from_ranges, wrap_angle  # noqa: E402
from global_planner import (OccupancyGrid, astar, point_at_distance,  # noqa: E402
                            simplify_path)
from hybrid_planner import AStarDWAController  # noqa: E402


class TestGeometry(unittest.TestCase):
    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(0.0), 0.0)
        self.assertAlmostEqual(abs(wrap_angle(3.0 * math.pi)), math.pi, places=6)
        self.assertAlmostEqual(abs(wrap_angle(-3.0 * math.pi)), math.pi, places=6)
        self.assertAlmostEqual(wrap_angle(math.pi + 1e-9), -math.pi, places=6)

    def test_scan_points_single_beam(self):
        pose = RobotState(1.0, 2.0, math.pi / 2.0)
        points = scan_points_from_ranges([1.0], 0.0, 0.0, pose)
        self.assertEqual(points.shape, (1, 2))
        self.assertAlmostEqual(points[0, 0], 1.0, places=6)
        self.assertAlmostEqual(points[0, 1], 3.0, places=6)

    def test_scan_points_drop_invalid(self):
        pose = RobotState(0.0, 0.0, 0.0)
        points = scan_points_from_ranges([0.0, float("nan"), 5.0, float("inf")],
                                         0.0, 0.1, pose)
        self.assertEqual(points.shape[0], 1)

    def test_rollout_straight_line(self):
        params = DWAParams(dt=0.1, predict_time=1.0)
        planner = DWA(params)
        traj = planner._rollout(RobotState(0, 0, 0, 0.5, 0),
                                np.array([0.5]), np.array([0.0]))
        self.assertAlmostEqual(traj[0, -1, 0], 0.5, places=6)
        self.assertAlmostEqual(traj[0, -1, 1], 0.0, places=6)

    def test_rollout_arc(self):
        params = DWAParams(dt=0.1, predict_time=1.0)
        planner = DWA(params)
        traj = planner._rollout(RobotState(0, 0, 0, 0.5, 1.0),
                                np.array([0.5]), np.array([1.0]))
        radius = 0.5 / 1.0
        self.assertAlmostEqual(traj[0, -1, 0], radius * math.sin(1.0), places=5)
        self.assertAlmostEqual(traj[0, -1, 1], radius * (1 - math.cos(1.0)), places=5)


class TestDWA(unittest.TestCase):
    def test_invalid_parameters_raise(self):
        with self.assertRaises(ValueError):
            DWA(DWAParams(max_v=-1.0))
        with self.assertRaises(ValueError):
            DWA(DWAParams(predict_time=0.01, dt=0.1))

    def test_dynamic_window_respects_acceleration(self):
        planner = DWA(DWAParams(max_acc_v=1.0, max_acc_w=2.0, dt=0.1))
        v_low, v_high, w_low, w_high = planner.dynamic_window(RobotState(v=0.5, w=0.0), 5.0)
        self.assertAlmostEqual(v_low, 0.4, places=6)
        self.assertAlmostEqual(v_high, 0.6, places=6)
        self.assertAlmostEqual(w_low, -0.2, places=6)
        self.assertAlmostEqual(w_high, 0.2, places=6)

    def test_dynamic_window_clips_to_limits(self):
        planner = DWA(DWAParams(max_v=0.8, min_v=-0.15))
        v_low, v_high, _, _ = planner.dynamic_window(RobotState(v=0.0, w=0.0), 5.0)
        self.assertAlmostEqual(v_low, -0.12, places=6)
        self.assertAlmostEqual(v_high, 0.12, places=6)

    def test_goal_reached_returns_zero(self):
        planner = DWA(DWAParams(goal_tolerance=0.3))
        result = planner.plan(RobotState(0, 0, 0, 0.5, 0.0), (0.1, 0.0),
                              np.empty((0, 2)))
        self.assertEqual(result.source, "goal")
        self.assertEqual(result.v, 0.0)
        self.assertEqual(result.w, 0.0)

    def test_open_space_prefers_speed(self):
        planner = DWA(DWAParams())
        result = planner.plan(RobotState(0, 0, 0, 0.8, 0.0), (10.0, 0.0),
                              np.empty((0, 2)))
        self.assertEqual(result.source, "dwa")
        self.assertGreater(result.v, 0.6)

    def test_plan_is_collision_free(self):
        planner = DWA(DWAParams())
        points = np.array([[1.5, 0.0]])
        result = planner.plan(RobotState(0, 0, 0, 0.5, 0.0), (5.0, 0.0), points)
        self.assertEqual(result.source, "dwa")
        d = np.hypot(result.trajectory[:, 0] - 1.5, result.trajectory[:, 1])
        self.assertGreaterEqual(d.min(), planner.params.robot_radius +
                                planner.params.safety_margin - 1e-6)

    def test_rotate_fallback_when_enclosed(self):
        params = DWAParams()
        planner = DWA(params)
        angles = np.linspace(0, 2 * math.pi, 36, endpoint=False)
        radius = params.robot_radius + params.safety_margin - 0.01
        points = np.column_stack([0.2 + radius * np.cos(angles),
                                  0.2 + radius * np.sin(angles)])
        result = planner.plan(RobotState(0.2, 0.2, 0.0, 0.0, 0.0), (10.0, 10.0), points)
        self.assertEqual(result.source, "rotate")
        self.assertEqual(result.v, 0.0)


class TestGlobalPlanner(unittest.TestCase):
    def _grid_with_wall(self) -> OccupancyGrid:
        grid = OccupancyGrid(0.0, 0.0, 10.0, 10.0, resolution=0.1)
        for gy in range(0, 80):
            grid.occupied[gy, 50] = True
        return grid

    def test_astar_goes_around_wall(self):
        grid = self._grid_with_wall()
        path = astar(grid, (2.0, 2.0), (8.0, 2.0))
        self.assertTrue(path)
        self.assertLess(path[-1][0], 8.1)
        self.assertGreater(max(p[1] for p in path), 8.0)

    def test_astar_no_path_when_enclosed(self):
        grid = OccupancyGrid(0.0, 0.0, 5.0, 5.0, resolution=0.1)
        for i in range(10, 40):
            for j in range(10, 40):
                if i in (10, 39) or j in (10, 39):
                    grid.occupied[j, i] = True
        self.assertEqual(astar(grid, (2.5, 2.5), (4.0, 4.0)), [])

    def test_simplify_path(self):
        grid = OccupancyGrid(0.0, 0.0, 10.0, 10.0, resolution=0.1)
        path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)]
        simplified = simplify_path(grid, path)
        self.assertEqual(simplified[0], path[0])
        self.assertEqual(simplified[-1], path[-1])
        self.assertLessEqual(len(simplified), len(path))

    def test_point_at_distance_projects_onto_path(self):
        path = [(0.0, 0.0), (10.0, 0.0)]
        target = point_at_distance(path, (5.0, 0.0), 2.0)
        self.assertAlmostEqual(target[0], 7.0, places=6)
        self.assertAlmostEqual(target[1], 0.0, places=6)

    def test_point_at_distance_short_path(self):
        path = [(0.0, 0.0), (1.0, 0.0)]
        target = point_at_distance(path, (0.0, 0.0), 5.0)
        self.assertAlmostEqual(target[0], 1.0, places=6)


class TestHybridPlanner(unittest.TestCase):
    def test_reaches_open_goal(self):
        import sim  # local import: sim imports the same modules

        params = sim.build_params()
        scenario = sim.Scenario("open", (0.0, 0.0, 0.0), (4.0, 0.0), [], [], 30.0)
        controller = AStarDWAController(params, (-1.0, -5.0, 8.0, 5.0))
        result = sim.simulate(scenario, controller, params)
        self.assertTrue(result.success)
        self.assertFalse(result.collision)


if __name__ == "__main__":
    unittest.main()
