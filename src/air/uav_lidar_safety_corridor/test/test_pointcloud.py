#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import struct
import sys
import unittest

import numpy as np

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
)

from pointcloud import pointcloud2_to_xyz  # noqa: E402


class Field:
    def __init__(self, name, offset):
        self.name = name
        self.offset = offset
        self.datatype = 7


class Cloud:
    def __init__(self, values):
        self.width = len(values)
        self.height = 1
        self.point_step = 12
        self.row_step = 12 * self.width
        self.data = b"".join(struct.pack("<fff", *value) for value in values)
        self.fields = [
            Field("x", 0),
            Field("y", 4),
            Field("z", 8),
        ]


class PointCloudTest(unittest.TestCase):
    def test_xyz_float32(self):
        cloud = Cloud([(1.0, 2.0, 3.0), (-1.5, 0.25, 4.0)])
        xyz = pointcloud2_to_xyz(cloud)
        np.testing.assert_allclose(
            xyz, [[1.0, 2.0, 3.0], [-1.5, 0.25, 4.0]]
        )

    def test_filters_non_finite_points(self):
        cloud = Cloud([(1.0, 2.0, 3.0), (float("nan"), 0.0, 0.0)])
        xyz = pointcloud2_to_xyz(cloud)
        self.assertEqual(xyz.shape, (1, 3))

    def test_missing_field_is_rejected(self):
        cloud = Cloud([(1.0, 2.0, 3.0)])
        cloud.fields = cloud.fields[:2]
        with self.assertRaises(ValueError):
            pointcloud2_to_xyz(cloud)


if __name__ == "__main__":
    unittest.main()
