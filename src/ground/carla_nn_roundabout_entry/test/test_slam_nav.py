import unittest

from carla_nn_roundabout_entry.slam_nav import OccupancyGrid, PlanFailure


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


if __name__ == "__main__":
    unittest.main()
