import unittest

from carla_nn_roundabout_entry.slam_nav import OccupancyGrid, PlanFailure
from carla_nn_roundabout_entry.navigation import GridNavigator, raycast_circles


class OccupancyNavigationTests(unittest.TestCase):
    def make_free_grid(self, width=5, height=3):
        grid = OccupancyGrid(width, height)
        for y in range(height):
            for x in range(width):
                grid.set_cell(x, y, grid.FREE)
        return grid

    def test_unknown_cells_are_not_traversable(self):
        grid = OccupancyGrid(5, 3)
        for x in range(5):
            grid.set_cell(x, 1, grid.FREE)
        grid.set_cell(2, 1, grid.UNKNOWN)
        result = grid.plan((0, 1), (4, 1))
        self.assertEqual(result.failure, PlanFailure.NO_PATH)

    def test_astar_finds_observed_free_route(self):
        grid = self.make_free_grid()
        grid.set_cell(2, 1, grid.OCCUPIED)
        result = grid.plan((0, 1), (4, 1))
        self.assertTrue(result.found)
        self.assertEqual(result.path[0], (0, 1))
        self.assertEqual(result.path[-1], (4, 1))
        self.assertNotIn((2, 1), result.path)

    def test_hit_ray_marks_free_cells_and_occupied_endpoint(self):
        grid = OccupancyGrid(5, 1)
        grid.integrate_ray((0, 0), (4, 0))
        self.assertEqual(grid.cells[0], [grid.FREE] * 4 + [grid.OCCUPIED])

    def test_miss_ray_marks_max_range_endpoint_free(self):
        grid = OccupancyGrid(5, 1)
        grid.integrate_ray((0, 0), (4, 0), hit=False)
        self.assertEqual(grid.cells[0], [grid.FREE] * 5)

    def test_zero_length_ray_does_not_create_obstacle(self):
        grid = OccupancyGrid(2, 2)
        grid.integrate_ray((1, 1), (1, 1))
        self.assertEqual(grid.cells[1][1], grid.UNKNOWN)

    def test_ray_stops_at_existing_obstacle(self):
        grid = OccupancyGrid(5, 1)
        grid.set_cell(2, 0, grid.OCCUPIED)
        grid.integrate_ray((0, 0), (4, 0))
        self.assertEqual(grid.cells[0], [grid.FREE, grid.FREE, grid.OCCUPIED,
                                         grid.UNKNOWN, grid.UNKNOWN])

    def test_plan_reports_start_goal_and_connectivity_failures(self):
        grid = self.make_free_grid(3, 1)
        grid.set_cell(0, 0, grid.OCCUPIED)
        self.assertEqual(grid.plan((0, 0), (2, 0)).failure,
                         PlanFailure.START_NOT_FREE)
        grid.set_cell(0, 0, grid.FREE)
        grid.set_cell(2, 0, grid.UNKNOWN)
        self.assertEqual(grid.plan((0, 0), (2, 0)).failure,
                         PlanFailure.GOAL_NOT_FREE)
        grid.set_cell(2, 0, grid.FREE)
        grid.set_cell(1, 0, grid.OCCUPIED)
        self.assertEqual(grid.plan((0, 0), (2, 0)).failure,
                         PlanFailure.NO_PATH)

    def test_diagonal_path_cannot_cut_blocked_corner(self):
        grid = self.make_free_grid(2, 2)
        grid.set_cell(1, 0, grid.OCCUPIED)
        result = grid.plan((0, 0), (1, 1))
        self.assertTrue(result.found)
        self.assertEqual(len(result.path), 3)

    def test_start_equal_goal_returns_single_cell_path(self):
        grid = self.make_free_grid(1, 1)
        result = grid.plan((0, 0), (0, 0))
        self.assertTrue(result.found)
        self.assertEqual(result.path, ((0, 0),))

    def test_scan_update_incrementally_maps_and_plans(self):
        navigator = GridNavigator(goal=(3.0, 0.0))
        result = navigator.update_scan(
            pose=(0.0, 0.0, 0.0),
            ranges=[5.0] * 25,
            angle_min=-1.57079632679,
            angle_increment=3.14159265359 / 24,
            range_max=5.0,
            stamp=1.0,
        )
        self.assertTrue(result.found)
        self.assertEqual(navigator.rays_integrated, 25)
        self.assertEqual(navigator.plan_count, 1)

    def test_infinite_range_marks_visible_cells_free(self):
        navigator = GridNavigator()
        navigator.update_scan(
            pose=(0.0, 0.0, 0.0),
            ranges=[float("inf")],
            angle_min=0.0,
            angle_increment=0.0,
            range_max=5.0,
            stamp=1.0,
        )
        self.assertEqual(navigator.grid._state(navigator.world_to_cell((5.0, 0.0))),
                         OccupancyGrid.FREE)
        self.assertEqual(navigator.rays_integrated, 1)

    def test_nan_range_is_ignored(self):
        navigator = GridNavigator()
        navigator.update_scan(
            pose=(0.0, 0.0, 0.0),
            ranges=[float("nan")],
            angle_min=0.0,
            angle_increment=0.0,
            range_max=5.0,
            stamp=1.0,
        )
        self.assertEqual(navigator.rays_integrated, 0)

    def test_metrics_record_frequency_latency_completion_and_collision(self):
        navigator = GridNavigator(goal=(3.0, 0.0))
        for stamp in (1.0, 1.1, 1.2):
            navigator.update_scan((0.0, 0.0, 0.0), [5.0] * 25,
                                  -1.57079632679, 3.14159265359 / 24, 5.0, stamp)
        navigator.record_collision(True)
        navigator.finish_episode(True)
        metrics = navigator.metrics()
        self.assertEqual(metrics["map_updates"], 3)
        self.assertGreater(metrics["map_update_frequency_hz"], 0)
        self.assertGreaterEqual(metrics["planning_average_ms"], 0)
        self.assertEqual(metrics["route_completion_rate"], 1.0)
        self.assertEqual(metrics["route_completion_basis"], "rule_policy_simulator")
        self.assertFalse(metrics["planned_path_drives_vehicle"])
        self.assertEqual(metrics["collision_frames"], 1)

    def test_simulated_scan_hits_nearest_circle(self):
        ranges = raycast_circles((0.0, 0.0), 0.0, [(5.0, 0.0, 1.0)], rays=3, max_range=10.0)
        self.assertEqual(len(ranges), 3)
        self.assertAlmostEqual(ranges[1], 4.0)
        self.assertEqual(ranges[0], 10.0)


if __name__ == "__main__":
    unittest.main()
