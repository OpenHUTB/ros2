import unittest

from survey_mission_planner.camera import CameraSpec


class CameraTest(unittest.TestCase):
    def setUp(self):
        self.camera = CameraSpec(13.2, 8.8, 24.0, 5472, 3648)

    def test_altitude_for_gsd(self):
        altitude = self.camera.altitude_for_gsd_m(0.012)
        self.assertAlmostEqual(altitude, 0.012 * 24.0 * 5472 / 13.2, places=6)
        self.assertAlmostEqual(self.camera.gsd_m(altitude), 0.012, places=6)

    def test_footprint_scales_with_altitude(self):
        width, height = self.camera.footprint_m(100.0)
        self.assertAlmostEqual(width, 55.0, places=3)
        self.assertAlmostEqual(height, 36.667, places=3)

    def test_invalid_spec_is_rejected(self):
        with self.assertRaises(ValueError):
            CameraSpec(0.0, 8.8, 24.0, 5472, 3648)


if __name__ == "__main__":
    unittest.main()
