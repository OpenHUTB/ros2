"""Tests for footprint-aware TTC, closest approach, and clearance."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vru_conflict_risk_map.risk import (  # noqa: E402
    EgoPoint,
    VruPoint,
    load_ego_track,
    load_vru_tracks,
)
from vru_conflict_risk_map.swept import (  # noqa: E402
    evaluate_swept_conflicts,
    swept_geometry,
)
from vru_conflict_risk_map.cli import main as cli_main  # noqa: E402


class SweptGeometryTest(unittest.TestCase):
    def test_head_on_collision_uses_combined_radius(self):
        geometry = swept_geometry(5.0, 0.0, -2.0, 0.0, 1.3)
        self.assertAlmostEqual(geometry.time_to_closest_approach_s, 2.5)
        self.assertAlmostEqual(geometry.closest_approach_distance_m, 0.0)
        self.assertAlmostEqual(geometry.min_clearance_m, -1.3)
        self.assertAlmostEqual(geometry.collision_ttc_s, 1.85)

    def test_offset_crossing_is_not_a_collision(self):
        geometry = swept_geometry(5.0, 2.0, -2.0, 0.0, 1.3)
        self.assertAlmostEqual(geometry.closest_approach_distance_m, 2.0)
        self.assertAlmostEqual(geometry.min_clearance_m, 0.7)
        self.assertIsNone(geometry.collision_ttc_s)

    def test_wide_clearance_is_safe(self):
        geometry = swept_geometry(10.0, 3.0, -2.0, 0.0, 1.3)
        self.assertAlmostEqual(geometry.closest_approach_distance_m, 3.0)
        self.assertAlmostEqual(geometry.min_clearance_m, 1.7)
        self.assertIsNone(geometry.collision_ttc_s)

    def test_already_overlapping_footprints_collide_at_zero(self):
        geometry = swept_geometry(1.0, 0.0, 0.0, 0.0, 1.3)
        self.assertAlmostEqual(geometry.min_clearance_m, -0.3)
        self.assertEqual(geometry.collision_ttc_s, 0.0)

    def test_moving_apart_uses_current_position_as_cpa(self):
        geometry = swept_geometry(5.0, 0.0, 2.0, 0.0, 1.3)
        self.assertEqual(geometry.time_to_closest_approach_s, 0.0)
        self.assertAlmostEqual(geometry.min_clearance_m, 3.7)
        self.assertIsNone(geometry.collision_ttc_s)


class SweptEvaluationTest(unittest.TestCase):
    def test_counts_are_per_track_not_per_sample(self):
        ego = [EgoPoint(0.0, 0.0, 0.0, 0.0), EgoPoint(1.0, 0.0, 0.0, 0.0)]
        vru = [
            VruPoint("p1", "pedestrian", 0.0, 1.0, 0.0),
            VruPoint("p1", "pedestrian", 1.0, 1.0, 0.0),
        ]
        report = evaluate_swept_conflicts(ego, vru)
        self.assertEqual(report["counts"]["critical"], 1)
        self.assertEqual(report["point_counts"]["critical"], 2)
        track = report["tracks"][0]
        self.assertEqual(track["collision_ttc_s"], 0.0)
        self.assertTrue(track["collision_within_horizon"])

    def test_near_miss_is_caution_without_collision(self):
        ego = [EgoPoint(0.0, 0.0, 0.0, 0.0)]
        vru = [VruPoint("p1", "pedestrian", 0.0, 0.0, 1.1)]
        report = evaluate_swept_conflicts(
            ego, vru, ego_radius_m=0.0,
            vru_radii_m={"pedestrian": 0.0},
            caution_clearance_m=1.5,
        )
        track = report["tracks"][0]
        self.assertEqual(track["risk"], "caution")
        self.assertFalse(track["collision_within_horizon"])

    def test_unknown_class_is_reported_as_no_data(self):
        report = evaluate_swept_conflicts(
            [EgoPoint(0.0, 0.0, 0.0, 1.0)],
            [VruPoint("x", "robot", 0.0, 0.0, 0.0)],
        )
        self.assertEqual(report["overall_risk"], "no_data")
        self.assertEqual(report["invalid_class_points"], 1)

    def test_invalid_geometry_configuration_is_rejected(self):
        ego = [EgoPoint(0.0, 0.0, 0.0, 0.0)]
        vru = [VruPoint("p1", "pedestrian", 0.0, 1.0, 0.0)]
        with self.assertRaises(ValueError):
            evaluate_swept_conflicts(ego, vru, ego_radius_m=-0.1)
        with self.assertRaises(ValueError):
            evaluate_swept_conflicts(ego, vru, horizon_s=0.0)

    def test_sample_data_runs_in_swept_mode(self):
        root = Path(__file__).resolve().parents[1]
        report = evaluate_swept_conflicts(
            load_ego_track(root / "data" / "ego.csv"),
            load_vru_tracks(root / "data" / "vru.csv"),
        )
        self.assertEqual(report["analysis"], "swept")
        self.assertIn(report["overall_risk"], {"safe", "caution", "critical"})
        self.assertEqual(len(report["tracks"]), 3)

    def test_cli_creates_nested_output_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "nested" / "report.json"
            self.assertEqual(
                cli_main(["--mode", "swept", "--out", str(output)]), 0)
            self.assertTrue(output.is_file())


if __name__ == "__main__":
    unittest.main()
