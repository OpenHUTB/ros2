#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import tempfile
import unittest
from unittest import mock
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
import sys
sys.path.insert(0, str(SCRIPTS))

from preflight_validator import (  # noqa: E402
    DEFAULT_LIMITS,
    validate_mission_data,
    validate_mission_file,
    write_report,
)


def mission(points, limits=None):
    return {
        "frame": "enu_relative",
        "limits": limits or {},
        "waypoints": points,
    }


class ValidatorTest(unittest.TestCase):
    def test_valid_demo_file(self):
        report = validate_mission_file(ROOT / "config/missions/demo_valid.yaml")
        self.assertTrue(report["valid"])
        self.assertEqual(report["status"], "VALID")
        self.assertEqual(report["summary"]["waypoint_count"], 4)
        self.assertEqual(report["summary"]["segment_count"], 4)

    def test_missing_file_is_structured_error(self):
        report = validate_mission_file(ROOT / "config/missions/not_found.yaml")
        self.assertFalse(report["valid"])
        self.assertEqual(report["errors"][0]["code"], "E001")

    def test_frame_must_be_explicit(self):
        data = mission([[0, 0, 2], [3, 0, 2]])
        data["frame"] = "world"
        report = validate_mission_data(data)
        self.assertIn("E002", [item["code"] for item in report["errors"]])

    def test_too_few_waypoints(self):
        report = validate_mission_data(mission([[0, 0, 2]]))
        self.assertIn("E004", [item["code"] for item in report["errors"]])

    def test_nonfinite_and_boolean_coordinates_rejected(self):
        report = validate_mission_data(mission([[0, 0, 2], [math.inf, 0, 2], [True, 0, 2]]))
        self.assertGreaterEqual(
            sum(item["code"] == "E006" for item in report["errors"]), 2
        )

    def test_numeric_string_coordinates_are_rejected(self):
        report = validate_mission_data(mission([[0, 0, 2], ["3", "0", "2"]]))
        self.assertIn("E006", [item["code"] for item in report["errors"]])

    def test_altitude_bounds(self):
        report = validate_mission_data(mission(
            [[0, 0, 2], [2, 0, 25]],
            {"min_altitude_m": 0.5, "max_altitude_m": 10.0},
        ))
        self.assertIn("E007", [item["code"] for item in report["errors"]])

    def test_duplicate_or_short_segment(self):
        report = validate_mission_data(mission(
            [[0, 0, 2], [3, 0, 2], [3.05, 0, 2]],
            {"min_segment_length_m": 0.25},
        ))
        self.assertIn("E008", [item["code"] for item in report["errors"]])

    def test_steep_climb_is_rejected(self):
        report = validate_mission_data(mission(
            [[0, 0, 2], [1, 0, 8]],
            {"max_climb_angle_deg": 60.0},
        ))
        self.assertIn("E009", [item["code"] for item in report["errors"]])

    def test_sharp_turn_is_rejected(self):
        report = validate_mission_data(mission(
            [[0, 0, 2], [4, 0, 2], [-0.5, 1, 2]],
            {"max_turn_angle_deg": 150.0},
        ))
        self.assertIn("E012", [item["code"] for item in report["errors"]])

    def test_moderate_turn_is_warning_not_error(self):
        report = validate_mission_data(mission(
            [[0, 0, 2], [4, 0, 2], [3, 2, 2]],
            {"warning_turn_angle_deg": 90.0, "max_turn_angle_deg": 150.0},
        ))
        self.assertTrue(report["valid"])
        self.assertIn("W001", [item["code"] for item in report["warnings"]])
        self.assertTrue(any(
            item["status"] == "WARNING" for item in report["waypoints"]
        ))

    def test_total_distance_limit(self):
        report = validate_mission_data(mission(
            [[0, 0, 2], [4, 0, 2], [8, 0, 2]],
            {"max_total_distance_m": 5.0},
        ))
        self.assertIn("E013", [item["code"] for item in report["errors"]])

    def test_invalid_limit_values_are_structured(self):
        report = validate_mission_data(mission(
            [[0, 0, 2], [3, 0, 2]],
            {
                "max_horizontal_speed_mps": 0,
                "warning_turn_angle_deg": 160,
                "max_turn_angle_deg": 150,
            },
        ))
        self.assertGreaterEqual(
            sum(item["code"] == "E011" for item in report["errors"]), 2
        )

    def test_json_round_trip(self):
        report = validate_mission_file(ROOT / "config/missions/demo_valid.yaml")
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "report.json"
            write_report(path, report)
            loaded = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(loaded["status"], report["status"])
        self.assertEqual(loaded["summary"], report["summary"])

    def test_home_path_is_expanded(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "mission.yaml"
            path.write_text(
                yaml.safe_dump(mission([[0, 0, 2], [3, 0, 2]])),
                encoding="utf-8",
            )
            with mock.patch.object(Path, "expanduser", return_value=path):
                report = validate_mission_file("~/mission.yaml")
        self.assertTrue(report["valid"])

    def test_default_limits_are_not_mutated(self):
        before = dict(DEFAULT_LIMITS)
        validate_mission_data(mission(
            [[0, 0, 2], [3, 0, 2]],
            {"max_waypoints": 20},
        ))
        self.assertEqual(DEFAULT_LIMITS, before)


class YamlFileTest(unittest.TestCase):
    def test_broken_yaml_is_reported(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "broken.yaml"
            path.write_text("frame: [\n", encoding="utf-8")
            report = validate_mission_file(path)
        self.assertFalse(report["valid"])
        self.assertEqual(report["errors"][0]["code"], "E001")

    def test_yaml_file_round_trip(self):
        data = mission([[0, 0, 2], [3, 0, 2]])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "mission.yaml"
            path.write_text(yaml.safe_dump(data), encoding="utf-8")
            report = validate_mission_file(path)
        self.assertTrue(report["valid"])


if __name__ == "__main__":
    unittest.main()
