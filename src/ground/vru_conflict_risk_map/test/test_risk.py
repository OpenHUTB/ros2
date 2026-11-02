#!/usr/bin/env python3
"""Tests for VRU conflict-risk evaluation."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vru_conflict_risk_map.risk import EgoPoint, VruPoint, evaluate_conflicts


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

    def test_unknown_class_is_rejected(self):
        with self.assertRaises(ValueError):
            evaluate_conflicts(
                [EgoPoint(0.0, 0.0, 0.0, 1.0)],
                [VruPoint("x", "robot", 0.0, 0.0, 0.0)],
            )


if __name__ == "__main__":
    unittest.main()
