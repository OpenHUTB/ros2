import json
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree

import numpy as np

from survey_mission_planner.camera import CameraSpec
from survey_mission_planner.export import export_csv, export_kml, export_wpml
from survey_mission_planner.planner import SurveyPlanner


class ExportTest(unittest.TestCase):
    def setUp(self):
        config = json.loads(
            (Path(__file__).resolve().parents[1] / "config" / "demo_mission.json").read_text()
        )
        self.plan = SurveyPlanner(
            CameraSpec(**config["camera"]),
            target_gsd_m=config["target_gsd_m"],
        ).plan(np.asarray(config["polygon"], dtype=float))

    def test_exports_are_valid(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            export_csv(self.plan, directory / "waypoints.csv")
            export_kml(self.plan, directory / "mission.kml")
            export_wpml(self.plan, directory / "mission.wpml")
            self.assertEqual(
                len((directory / "waypoints.csv").read_text().strip().splitlines()),
                len(self.plan.waypoints_enu) + 1,
            )
            ElementTree.parse(directory / "mission.kml")
            wpml = ElementTree.parse(directory / "mission.wpml")
            self.assertIn("kml", wpml.getroot().tag)
            self.assertIn("wpmz", (directory / "mission.wpml").read_text())


if __name__ == "__main__":
    unittest.main()
