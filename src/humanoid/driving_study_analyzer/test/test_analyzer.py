#!/usr/bin/env python3
"""Tests for the driving-study CSV analyzer."""

from __future__ import annotations

import csv
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from driving_study_analyzer.analyzer import SchemaError, analyze_files, summarize


def _read(path: Path):
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        return [dict(row) for row in reader], list(reader.fieldnames or [])


class DrivingStudyAnalyzerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.root = Path(__file__).resolve().parents[1]
        self.driving = self.root / "data" / "sample_session.csv"
        self.gaze = self.root / "data" / "sample_session_gaze.csv"

    def test_summary_has_expected_metrics(self):
        report = analyze_files(self.driving, self.gaze)
        self.assertEqual(report["schema_version"], "2")
        self.assertEqual(report["rows"]["driving_with_time"], 5)
        self.assertEqual(report["gaze"]["saccade_event_count"], 1)
        self.assertAlmostEqual(report["speed_kmh"]["max"], 28.0)
        self.assertAlmostEqual(report["gaze"]["mean_saccade_amplitude_deg"], 10.0)

    def test_missing_column_is_rejected(self):
        rows, columns = _read(self.driving)
        broken = [name for name in columns if name != "speed_kmh"]
        with self.assertRaises(SchemaError):
            summarize(rows, [], broken, None)

    def test_empty_sim_time_is_rejected(self):
        with self.assertRaises(SchemaError):
            summarize([{"sim_time_s": ""}], [], None, None)

    def test_without_gaze_file_still_works(self):
        report = analyze_files(self.driving, None)
        self.assertEqual(report["rows"]["gaze"], 0)
        json.dumps(report)

    def test_unknown_control_values_are_ignored(self):
        rows = [
            {"sim_time_s": "0.0", "speed_kmh": "bad", "steer_input": "", "throttle_input": "", "brake_input": ""},
            {"sim_time_s": "0.1", "speed_kmh": "10", "steer_input": "0.2", "throttle_input": "0.3", "brake_input": "0.0"},
        ]
        report = summarize(rows, [], None, None)
        self.assertAlmostEqual(report["speed_kmh"]["mean"], 10.0)
        self.assertAlmostEqual(report["controls"]["steer_active_ratio"], 0.5)


if __name__ == "__main__":
    unittest.main()
