import unittest

from carla_nn_roundabout_entry.slam_nav import OccupancyGrid


class OccupancyNavigationTests(unittest.TestCase):
    def test_unknown_cells_are_not_traversable(self):
        grid = OccupancyGrid(5, 3)
        for x in range(5):
            grid.set_cell(x, 1, grid.FREE)
        grid.set_cell(2, 1, grid.UNKNOWN)
        self.assertEqual(grid.plan((0, 1), (4, 1)), [])

    def test_astar_finds_observed_free_route(self):
        grid = OccupancyGrid(5, 3)
        for y in range(3):
            for x in range(5):
                grid.set_cell(x, y, grid.FREE)
        grid.set_cell(2, 1, grid.OCCUPIED)
        path = grid.plan((0, 1), (4, 1))
        self.assertEqual(path[0], (0, 1))
        self.assertEqual(path[-1], (4, 1))
        self.assertNotIn((2, 1), path)


if __name__ == "__main__":
    unittest.main()
