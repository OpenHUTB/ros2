import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from panoramic_camera_projection.stitcher import (
    FACE_NAMES,
    create_demo_cubemap,
    face_for_direction,
    stitch_cubemap,
)


class PanoramaProjectionTests(unittest.TestCase):
    def test_face_centers_map_to_expected_faces(self):
        directions = {
            "front": (1.0, 0.0, 0.0),
            "right": (0.0, 1.0, 0.0),
            "back": (-1.0, 0.0, 0.0),
            "left": (0.0, -1.0, 0.0),
            "up": (0.0, 0.0, 1.0),
            "down": (0.0, 0.0, -1.0),
        }
        for expected, vector in directions.items():
            name, u, v = face_for_direction(*vector)
            self.assertEqual(name, expected)
            self.assertAlmostEqual(u, 0.0)
            self.assertAlmostEqual(v, 0.0)

    def test_demo_stitch_has_expected_shape(self):
        faces = create_demo_cubemap(64)
        panorama = stitch_cubemap(faces, (128, 256))
        self.assertEqual(panorama.shape, (128, 256, 3))
        unique_colors = np.unique(panorama.reshape(-1, 3), axis=0)
        self.assertGreaterEqual(len(unique_colors), len(FACE_NAMES))

    def test_missing_face_is_rejected(self):
        faces = create_demo_cubemap(32)
        faces.pop("down")
        with self.assertRaisesRegex(ValueError, "missing cube faces"):
            stitch_cubemap(faces)


if __name__ == "__main__":
    unittest.main(verbosity=2)
