import math
import os
import sys
import unittest


MODULE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if MODULE_DIR not in sys.path:
    sys.path.insert(0, MODULE_DIR)

from survey_core import build_lawnmower_path, path_length, translate_path


class LawnmowerPathTests(unittest.TestCase):
    def test_path_covers_both_rectangle_boundaries(self):
        path = build_lawnmower_path(60, 40, 8, 20)

        self.assertEqual(len(path), 12)
        self.assertEqual(min(point[0] for point in path), -30.0)
        self.assertEqual(max(point[0] for point in path), 30.0)
        self.assertEqual(min(point[1] for point in path), -20.0)
        self.assertEqual(max(point[1] for point in path), 20.0)
        self.assertTrue(all(point[2] == -20.0 for point in path))

    def test_lanes_alternate_direction(self):
        path = build_lawnmower_path(10, 4, 2, 5)

        self.assertEqual(path[0][0:2], (-5.0, -2.0))
        self.assertEqual(path[1][0:2], (5.0, -2.0))
        self.assertEqual(path[2][0:2], (5.0, 0.0))
        self.assertEqual(path[3][0:2], (-5.0, 0.0))

    def test_actual_spacing_never_exceeds_requested_spacing(self):
        path = build_lawnmower_path(10, 10, 3, 5)
        lanes = sorted(set(point[1] for point in path))
        gaps = [right - left for left, right in zip(lanes, lanes[1:])]

        self.assertTrue(all(gap <= 3.0 for gap in gaps))
        self.assertEqual(lanes[0], -5.0)
        self.assertEqual(lanes[-1], 5.0)

    def test_spacing_wider_than_area_still_covers_both_edges(self):
        path = build_lawnmower_path(10, 4, 20, 5)
        self.assertEqual(len(path), 4)
        self.assertEqual(sorted(set(point[1] for point in path)), [-2.0, 2.0])

    def test_translation_uses_world_frame_offset(self):
        path = build_lawnmower_path(2, 2, 2, 3)
        translated = translate_path(path, 10, -4, 1)

        self.assertEqual(translated[0], (9.0, -5.0, -2.0))
        self.assertEqual(translated[1], (11.0, -5.0, -2.0))
        self.assertEqual(translated[-1], (9.0, -3.0, -2.0))

    def test_path_length(self):
        self.assertAlmostEqual(path_length([(0, 0, 0), (3, 4, 0)]), 5.0)
        self.assertAlmostEqual(
            path_length([(3, 4, 0)], start=(0, 0, 0)), 5.0
        )

    def test_non_positive_and_non_finite_parameters_are_rejected(self):
        invalid_values = (0, -1, math.nan, math.inf, -math.inf)
        for value in invalid_values:
            for index in range(4):
                values = [10, 8, 2, 5]
                values[index] = value
                with self.subTest(value=value, parameter=index):
                    with self.assertRaises(ValueError):
                        build_lawnmower_path(*values)

    def test_non_finite_offsets_are_rejected(self):
        with self.assertRaises(ValueError):
            translate_path([(0, 0, 0)], offset_x=math.nan)


if __name__ == "__main__":
    unittest.main(verbosity=2)
