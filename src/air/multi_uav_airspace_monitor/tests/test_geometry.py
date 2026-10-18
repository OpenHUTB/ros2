import unittest

from multi_uav_airspace_monitor.geometry import (
    distance_point_to_segment,
    distance_to_polygon,
    latlon_to_local,
    local_to_latlon,
    point_in_polygon,
    polygon_area,
)


class GeometryTest(unittest.TestCase):
    def setUp(self):
        self.square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]

    def test_inside_outside_and_boundary(self):
        self.assertTrue(point_in_polygon((5.0, 5.0), self.square))
        self.assertFalse(point_in_polygon((15.0, 5.0), self.square))
        self.assertTrue(point_in_polygon((10.0, 5.0), self.square))

    def test_distance_and_area(self):
        self.assertAlmostEqual(distance_to_polygon((5.0, 5.0), self.square), 0.0)
        self.assertAlmostEqual(distance_to_polygon((13.0, 5.0), self.square), 3.0)
        self.assertAlmostEqual(
            distance_point_to_segment((5.0, 3.0), (0.0, 0.0), (10.0, 0.0)), 3.0
        )
        self.assertAlmostEqual(polygon_area(self.square), 100.0)

    def test_local_latlon_round_trip(self):
        lat, lon = local_to_latlon(120.0, -80.0, 28.22, 112.94)
        east, north = latlon_to_local(lat, lon, 28.22, 112.94)
        self.assertAlmostEqual(east, 120.0, places=3)
        self.assertAlmostEqual(north, -80.0, places=3)


if __name__ == "__main__":
    unittest.main()
