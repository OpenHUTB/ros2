import unittest

from carla_nn_roundabout_entry.sensors import closest_gap, raycast_ranges
from carla_nn_roundabout_entry.slam_nav import OccupancyGrid


class SensorTests(unittest.TestCase):
    def test_front_ray_detects_vehicle_circle(self):
        values = raycast_ranges((0.0, 0.0), 0.0, [(8.0, 0.0, 1.0)], rays=5)
        self.assertAlmostEqual(values[2], 7.0)
        self.assertEqual(values[0], 30.0)

    def test_gap_reports_closing_speed_and_ttc(self):
        gap = closest_gap((0.0, 0.0, 0.0, 8.0), [(20.0, 0.0, 3.0, 0.0)])
        self.assertEqual(gap["gap_m"], 20.0)
        self.assertEqual(gap["closing_mps"], 5.0)
        self.assertEqual(gap["ttc_s"], 4.0)

    def test_receding_vehicle_has_no_imminent_ttc(self):
        gap = closest_gap((0.0, 0.0, 0.0, 4.0), [(10.0, 0.0, 8.0, 0.0)])
        self.assertEqual(gap["ttc_s"], 30.0)

    def test_max_range_ray_is_a_miss(self):
        ranges = raycast_ranges((0.0, 0.0), 0.0, [], rays=3, max_range=12.0)
        self.assertEqual(ranges, [12.0, 12.0, 12.0])

        grid = OccupancyGrid(5, 3)
        grid.integrate_ray((0, 1), (4, 1), hit=False)
        self.assertEqual(grid.cells[1][4], OccupancyGrid.FREE)


if __name__ == "__main__":
    unittest.main()
