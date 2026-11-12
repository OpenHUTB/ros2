#!/usr/bin/env python3
"""Tests for VRU conflict-risk evaluation."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vru_conflict_risk_map.risk import (
    EgoPoint,
    VruPoint,
    evaluate_conflicts,
    load_ego_track,
    load_vru_tracks,
)


class RiskMapTest(unittest.TestCase):
    def test_critical_distance(self):
        report = evaluate_conflicts(
            [EgoPoint(0.0, 0.0, 0.0, 10.0)],
            [VruPoint("p1", "pedestrian", 0.0, 2.0, 0.0)],
        )
        self.assertEqual(report["overall_risk"], "critical")

    def test_caution_distance(self):
        report = evaluate_conflicts(
            [EgoPoint(0.0, 0.0, 0.0, 2.0)],
            [VruPoint("p1", "pedestrian", 0.0, 5.0, 0.0)],
        )
        self.assertEqual(report["overall_risk"], "caution")

    def test_safe_distance(self):
        report = evaluate_conflicts(
            [EgoPoint(0.0, 0.0, 0.0, 2.0)],
            [VruPoint("c1", "child", 0.0, 20.0, 0.0)],
        )
        self.assertEqual(report["overall_risk"], "safe")

    def test_nearest_ego_sample_is_used(self):
        ego = [EgoPoint(0.0, 0.0, 0.0, 2.0), EgoPoint(0.1, 1.0, 0.0, 2.0)]
        report = evaluate_conflicts(ego, [VruPoint("b1", "cyclist", 0.09, 5.0, 0.0)])
        self.assertEqual(report["tracks"][0]["min_distance_m"], 4.0)

    def test_multiple_tracks_are_kept(self):
        report = evaluate_conflicts(
            [EgoPoint(0.0, 0.0, 0.0, 10.0)],
            [
                VruPoint("p1", "pedestrian", 0.0, 2.0, 0.0),
                VruPoint("p2", "child", 0.0, 5.0, 0.0),
            ],
        )
        self.assertEqual(len(report["tracks"]), 2)

    def test_stopped_ego_uses_vru_relative_motion_for_ttc(self):
        ego = [EgoPoint(0.0, 0.0, 0.0, 0.0), EgoPoint(1.0, 0.0, 0.0, 0.0)]
        vru = [
            VruPoint("p1", "pedestrian", 0.0, 12.0, 0.0),
            VruPoint("p1", "pedestrian", 1.0, 11.0, 0.0),
        ]
        report = evaluate_conflicts(ego, vru)
        self.assertAlmostEqual(report["tracks"][0]["min_ttc_s"], 11.0)
        self.assertEqual(report["overall_risk"], "safe")

    def test_stationary_side_vru_has_no_ttc(self):
        ego = [EgoPoint(0.0, 0.0, 0.0, 10.0), EgoPoint(1.0, 10.0, 0.0, 10.0)]
        vru = [
            VruPoint("p1", "pedestrian", 0.0, 0.0, 10.0),
            VruPoint("p1", "pedestrian", 1.0, 0.0, 10.0),
        ]
        report = evaluate_conflicts(ego, vru)
        self.assertIsNone(report["tracks"][0]["min_ttc_s"])
        self.assertEqual(report["overall_risk"], "safe")

    def test_counts_are_per_track_not_per_sample(self):
        ego = [EgoPoint(0.0, 0.0, 0.0, 10.0), EgoPoint(0.1, 1.0, 0.0, 10.0)]
        vru = [
            VruPoint("p1", "pedestrian", 0.0, 2.0, 0.0),
            VruPoint("p1", "pedestrian", 0.1, 2.0, 0.0),
        ]
        report = evaluate_conflicts(ego, vru)
        self.assertEqual(report["counts"]["critical"], 1)
        self.assertEqual(report["point_counts"]["critical"], 2)

    def test_min_distance_and_min_ttc_keep_their_own_timestamps(self):
        ego = [
            EgoPoint(0.0, 0.0, 0.0, 10.0),
            EgoPoint(1.0, 10.0, 0.0, 10.0),
            EgoPoint(2.0, 20.0, 0.0, 10.0),
        ]
        vru = [
            VruPoint("p1", "pedestrian", 0.0, 5.0, 0.0),
            VruPoint("p1", "pedestrian", 1.0, 5.0, 0.0),
            VruPoint("p1", "pedestrian", 2.0, 18.0, 0.0),
        ]
        track = evaluate_conflicts(ego, vru)["tracks"][0]
        self.assertEqual(track["min_distance_m"], 2.0)
        self.assertEqual(track["min_distance_timestamp_s"], 2.0)
        self.assertAlmostEqual(track["min_ttc_s"], 0.5)
        self.assertEqual(track["min_ttc_timestamp_s"], 0.0)

    def test_unmatched_points_are_reported_as_no_data(self):
        report = evaluate_conflicts(
            [EgoPoint(0.0, 0.0, 0.0, 1.0)],
            [VruPoint("p1", "pedestrian", 0.1, 2.0, 0.0)],
            timestamp_tolerance=0.05,
        )
        self.assertEqual(report["overall_risk"], "no_data")
        self.assertEqual(report["skipped_points"], 1)
        self.assertEqual(report["skipped_tracks"], 1)
        self.assertTrue(report["warnings"])

    def test_unknown_class_is_skipped_with_warning(self):
        report = evaluate_conflicts(
            [EgoPoint(0.0, 0.0, 0.0, 1.0)],
            [VruPoint("x", "robot", 0.0, 0.0, 0.0)],
        )
        self.assertEqual(report["overall_risk"], "no_data")
        self.assertEqual(report["invalid_class_points"], 1)
        self.assertEqual(report["skipped_tracks"], 1)
        self.assertTrue(report["warnings"])

    def test_sample_data_runs_through_loaders(self):
        root = Path(__file__).resolve().parents[1]
        report = evaluate_conflicts(
            load_ego_track(root / "data" / "ego.csv"),
            load_vru_tracks(root / "data" / "vru.csv"),
        )
        self.assertEqual(report["overall_risk"], "critical")
        self.assertEqual(len(report["tracks"]), 3)
        self.assertEqual(report["counts"], {"critical": 1, "caution": 1, "safe": 1})


if __name__ == "__main__":
    unittest.main()
