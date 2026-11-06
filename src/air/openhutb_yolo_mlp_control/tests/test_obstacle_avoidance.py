"""Local routing against synthetic obstacles; no simulator required."""

import importlib.util
from pathlib import Path
import math
import unittest

SPEC = importlib.util.spec_from_file_location(
    'traffic_obstacles', Path(__file__).parents[1] / 'tools/traffic/obstacle_avoidance.py')
obstacles = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(obstacles)


class Queries:
    def __init__(self, clear):
        self.test = clear

    def clear(self, rays):
        return all(self.test(a, b) for a, b in rays)


class AvoidanceTests(unittest.TestCase):
    def test_clear_corridor_keeps_original_goal(self):
        router = obstacles.LocalAvoidance(Queries(lambda a, b: True))
        self.assertEqual(router.select([0, 0, 0], [30, 0, 0]), ([30, 0, 0], 'follow'))

    def test_obstructed_corridor_stops_instead_of_advancing(self):
        router = obstacles.LocalAvoidance(Queries(lambda a, b: False))
        self.assertEqual(router.select([0, 0, 0], [30, 0, 0]), ([0, 0, 0], 'blocked'))

    def test_local_wall_produces_checked_detour(self):
        def clear(a, b):
            # Thin wall at x=4, spanning y=-2..2, z=-4..4.
            if abs(b[0]-a[0]) < 1e-9:
                return True
            t = (4-a[0])/(b[0]-a[0])
            if not 0 <= t <= 1:
                return True
            y, z = a[1]+t*(b[1]-a[1]), a[2]+t*(b[2]-a[2])
            return abs(y) > 2 or abs(z) > 4

        router = obstacles.LocalAvoidance(Queries(clear))
        waypoint, phase = router.select([0, 0, 0], [20, 0, 0])
        self.assertEqual(phase, 'avoid')
        self.assertTrue(router.clear([0, 0, 0], waypoint))
        self.assertTrue(router.clear(waypoint, [20, 0, 0]))
        # Moving the target must not keep switching the intermediate waypoint.
        self.assertEqual(router.select([0, 0, 0], [20, 1, 0]), (waypoint, 'avoid'))

    def test_new_obstacle_invalidates_detour(self):
        queries = Queries(lambda a, b: True)
        router = obstacles.LocalAvoidance(queries)
        router.waypoint = [6, 8, 0]
        queries.test = lambda a, b: False
        self.assertEqual(router.select([0, 0, 0], [20, 0, 0])[1], 'blocked')

    def test_margin_checks_more_than_centre_ray(self):
        rays = obstacles.corridor_rays([0, 0, 0], [10, 0, 0], 1.5)
        self.assertEqual(len(rays), 9)
        self.assertTrue(any(abs(a[1]) == 1.5 and abs(a[2]) == 1.5 for a, b in rays))
        for a, b in rays:
            self.assertAlmostEqual(math.dist(a, b), 10)

    def test_vertical_and_zero_length_corridors(self):
        rays = obstacles.corridor_rays([0, 0, 0], [0, 0, -10], 1.5)
        self.assertEqual(len(rays), 9)
        self.assertTrue(all(math.isfinite(v) for ray in rays for p in ray for v in p))
        self.assertEqual(obstacles.corridor_rays([0, 0, 0], [0, 0, 0], 1.5), [])

    def test_query_failure_does_not_return_a_movement(self):
        class FailedQueries:
            def clear(self, rays):
                raise RuntimeError('timeout')
        router = obstacles.LocalAvoidance(FailedQueries())
        with self.assertRaisesRegex(RuntimeError, 'timeout'):
            router.select([0, 0, 0], [20, 0, 0])
