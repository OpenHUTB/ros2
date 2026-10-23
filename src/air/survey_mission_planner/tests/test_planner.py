import json
import unittest
from pathlib import Path

import numpy as np

from survey_mission_planner.camera import CameraSpec
from survey_mission_planner.planner import SurveyPlanner


class PlannerTest(unittest.TestCase):
    def setUp(self):
        config = json.loads(
            (Path(__file__).resolve().parents[1] / "config" / "demo_mission.json").read_text()
        )
        self.polygon = np.asarray(config["polygon"], dtype=float)
        self.plan = SurveyPlanner(
            CameraSpec(**config["camera"]),
            target_gsd_m=config["target_gsd_m"],
            forward_overlap=config["forward_overlap"],
            side_overlap=config["side_overlap"],
            speed_mps=config["speed_mps"],
            origin_lat=config["origin_lat"],
            origin_lon=config["origin_lon"],
        ).plan(self.polygon)

    def test_altitude_and_spacing(self):
        self.assertAlmostEqual(self.plan.altitude_m, 119.389, places=2)
        self.assertAlmostEqual(
            self.plan.line_spacing_m,
            self.plan.footprint_width_m * 0.3,
            places=6,
        )
        self.assertAlmostEqual(
            self.plan.shot_spacing_m,
            self.plan.footprint_height_m * 0.2,
            places=6,
        )

    def test_coverage_and_counts(self):
        self.assertGreaterEqual(self.plan.coverage_ratio, 0.90)
        self.assertGreater(self.plan.line_count, 0)
        self.assertGreater(self.plan.image_count, self.plan.line_count)
        self.assertGreater(self.plan.flight_distance_m, 0)

    def test_waypoints_stay_inside_or_on_site(self):
        from survey_mission_planner.geometry import point_in_polygon

        for point in self.plan.waypoints_enu:
            self.assertTrue(point_in_polygon(point, self.polygon))

    def test_deterministic(self):
        second = SurveyPlanner(
            CameraSpec(13.2, 8.8, 24.0, 5472, 3648),
            target_gsd_m=0.012,
            forward_overlap=0.8,
            side_overlap=0.7,
        ).plan(self.polygon)
        np.testing.assert_allclose(self.plan.waypoints_enu, second.waypoints_enu)


if __name__ == "__main__":
    unittest.main()
