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

from depth_guard import (  # noqa: E402
    central_min_depth,
    depth_image_to_array,
    depth_speed_scale,
)


class Image:
    encoding = "32FC1"
    width = 2
    height = 2
    step = 8
    data = struct.pack("<ffff", 5.0, 4.0, 3.0, 2.0)


class DepthGuardTest(unittest.TestCase):
    def test_decode_float_depth(self):
        depth = depth_image_to_array(Image())
        np.testing.assert_allclose(depth, [[5.0, 4.0], [3.0, 2.0]])

    def test_central_min_depth(self):
        depth = np.array(
            [[2.0, 2.0, 40.0, 40.0],
             [2.0, 1.2, 40.0, 40.0],
             [2.0, 2.0, 40.0, 40.0]],
            dtype=np.float32,
        )
        self.assertAlmostEqual(
            central_min_depth(depth, 0.25, 0.75, 0.0, 1.0), 1.2
        )

    def test_depth_speed_scale(self):
        self.assertEqual(depth_speed_scale(0.8, 1.2, 3.5), 0.0)
        self.assertEqual(depth_speed_scale(4.0, 1.2, 3.5), 1.0)
        self.assertAlmostEqual(depth_speed_scale(2.35, 1.2, 3.5), 0.5)

    def test_invalid_encoding_is_rejected(self):
        image = Image()
        image.encoding = "rgb8"
        with self.assertRaises(ValueError):
            depth_image_to_array(image)


if __name__ == "__main__":
    unittest.main()
