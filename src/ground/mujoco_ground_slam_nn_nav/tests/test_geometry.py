import math
import unittest

import numpy as np

from gvnn.geometry import (bresenham, inflate, line_of_sight, path_length,
                           relative_pose, transform_points, wrap_angle)


class GeometryTest(unittest.TestCase):
    def test_wrap(self):
        self.assertAlmostEqual(wrap_angle(3 * math.pi), math.pi)
        self.assertAlmostEqual(wrap_angle(-3 * math.pi), math.pi)

    def test_transform_roundtrip(self):
        pts = np.array([[1.0, 0.0], [0.0, 1.0]])
        pose = (2.0, 3.0, math.pi / 2)
        world = transform_points(pts, pose)
        np.testing.assert_allclose(world[0], [2.0, 4.0], atol=1e-9)
        rel = relative_pose((0, 0, 0), pose)
        np.testing.assert_allclose(rel[:2], [2.0, 3.0], atol=1e-9)

    def test_bresenham(self):
        cells = list(bresenham(0, 0, 4, 2))
        self.assertEqual(cells[0], (0, 0))
        self.assertEqual(cells[-1], (4, 2))
        self.assertGreaterEqual(len(cells), 5)

    def test_line_of_sight(self):
        occ = np.zeros((20, 20), dtype=bool)
        occ[10, :] = True
        self.assertFalse(line_of_sight(occ, (1, 1), (18, 18), 1.0, 0))
        self.assertTrue(line_of_sight(occ, (1, 1), (18, 5), 1.0, 0))

    def test_inflate(self):
        g = np.zeros((5, 5), dtype=bool)
        g[2, 2] = True
        out = inflate(g, 1)
        self.assertTrue(out[1:4, 1:4].all())
        self.assertFalse(out[0, 0])

    def test_path_length(self):
        self.assertAlmostEqual(path_length([(0, 0), (3, 4), (3, 8)]), 9.0)


if __name__ == "__main__":
    unittest.main()
