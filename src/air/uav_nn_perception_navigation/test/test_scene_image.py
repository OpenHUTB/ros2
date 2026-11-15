"""AirSim Scene decoding must accept both supported byte layouts."""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from scene_image import decode_scene_bgr  # noqa: E402


class TestSceneImage(unittest.TestCase):
    def test_bgr_is_preserved(self):
        image = decode_scene_bgr(bytes([10, 20, 30, 40, 50, 60]), 1, 2)
        np.testing.assert_array_equal(image, [[[10, 20, 30], [40, 50, 60]]])

    def test_bgra_drops_alpha(self):
        image = decode_scene_bgr(bytes([10, 20, 30, 255, 40, 50, 60, 0]), 1, 2)
        np.testing.assert_array_equal(image, [[[10, 20, 30], [40, 50, 60]]])
        self.assertTrue(image.flags.c_contiguous)

    def test_invalid_shape_is_rejected(self):
        with self.assertRaises(ValueError):
            decode_scene_bgr(bytes([1, 2]), 1, 1)

    def test_empty_dimensions_are_rejected(self):
        with self.assertRaises(ValueError):
            decode_scene_bgr(b'', 0, 2)


if __name__ == '__main__':
    unittest.main()
