"""core.py 的离线单元测试。"""

import math
import unittest

from uav_geofence_bypass import core

SQUARE = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]


class TestPolygons(unittest.TestCase):
    def test_point_inside(self):
        self.assertTrue(core.point_in_polygon((1.0, 1.0), SQUARE))

    def test_point_outside(self):
        self.assertFalse(core.point_in_polygon((3.0, 1.0), SQUARE))

    def test_inflation_grows_and_contains_original(self):
        grown = core.inflate_polygon(SQUARE, 0.5)
        self.assertEqual(len(grown), 4)
        self.assertGreater(abs(core.signed_area(grown)), abs(core.signed_area(SQUARE)))
        for p in SQUARE:
            self.assertTrue(core.point_in_polygon(p, grown))
        self.assertTrue(core.point_in_polygon((-0.3, 1.0), grown))
        self.assertFalse(core.point_in_polygon((-0.3, 1.0), SQUARE))

    def test_clearance_sign(self):
        self.assertGreater(core.clearance((3.0, 1.0), SQUARE), 0.0)
        self.assertLess(core.clearance((1.0, 1.0), SQUARE), 0.0)


class TestSegments(unittest.TestCase):
    def test_crossing(self):
        self.assertTrue(core.segments_intersect((0, 0), (2, 2), (0, 2), (2, 0)))

    def test_parallel(self):
        self.assertFalse(core.segments_intersect((0, 0), (2, 0), (0, 1), (2, 1)))

    def test_segment_clear_of_polygon(self):
        self.assertTrue(core.segment_clear((0.0, 0.0), (0.0, 8.0), core.ZONES[0]))

    def test_segment_blocked_by_polygon(self):
        self.assertFalse(core.segment_clear((0.0, 0.0), (15.0, 0.0), core.ZONES[0]))


class TestPlan(unittest.TestCase):
    def test_direct_when_clear(self):
        path, blocked = core.plan_path((0.0, 0.0), (0.0, 8.0), core.ZONES, 0.6)
        self.assertFalse(blocked)
        self.assertEqual(len(path), 2)
        self.assertEqual(path[-1], (0.0, 8.0))

    def test_bypass_route_exists(self):
        path, blocked = core.plan_path(core.START, core.GOAL, core.ZONES, 0.6)
        self.assertFalse(blocked)
        self.assertGreaterEqual(len(path), 3)
        self.assertEqual(path[0], core.START)
        self.assertEqual(path[-1], core.GOAL)

    def test_route_avoids_inflated_zones(self):
        path, _ = core.plan_path(core.START, core.GOAL, core.ZONES, 0.6)
        inflated = [core.inflate_polygon(z, 0.6) for z in core.ZONES]
        for a, b in zip(path[:-1], path[1:]):
            for poly in inflated:
                self.assertTrue(core.segment_clear(a, b, poly))

    def test_route_longer_than_direct(self):
        path, _ = core.plan_path(core.START, core.GOAL, core.ZONES, 0.6)
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path[:-1], path[1:]))
        self.assertGreater(length, 15.0)


class TestNextWaypoint(unittest.TestCase):
    def test_skips_reached_points(self):
        path = [(0.0, 0.0), (1.0, 0.0), (5.0, 0.0)]
        wp, idx = core.next_waypoint(path, (0.95, 0.0), 0.5)
        self.assertEqual(idx, 2)
        self.assertEqual(wp, (5.0, 0.0))


if __name__ == "__main__":
    unittest.main()
