#!/usr/bin/env python3
"""Tests for VRU track loading and metrics."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vru_twin_metrics.metrics import VruTrack, evaluate_vru_tracks, load_vru_tracks


def track(track_id: str, timestamp: float, x: float, y: float = 0.0, cls: str = "pedestrian") -> VruTrack:
    return VruTrack(track_id=track_id, vru_class=cls, timestamp=timestamp, x=x, y=y)


class VruMetricsTest(unittest.TestCase):
    def test_identical_tracks_score_perfectly(self):
        tracks = [track("p1", 0.0, 0.0), track("p1", 0.5, 0.6)]
        metrics = evaluate_vru_tracks(tracks, tracks)
        self.assertAlmostEqual(metrics.mota, 1.0)
        self.assertAlmostEqual(metrics.ade_m, 0.0)
        self.assertAlmostEqual(metrics.fde_m, 0.0)

    def test_missing_track_counts_as_false_negative(self):
        gt = [track("p1", 0.0, 0.0), track("p2", 0.0, 2.0)]
        pred = [track("p1", 0.0, 0.0)]
        metrics = evaluate_vru_tracks(gt, pred)
        self.assertEqual(metrics.false_negatives, 1)
        self.assertAlmostEqual(metrics.mota, 0.5)

    def test_shifted_track_has_ade_and_fde(self):
        gt = [track("p1", 0.0, 0.0), track("p1", 0.5, 0.6)]
        pred = [track("p1", 0.0, 1.0), track("p1", 0.5, 1.6)]
        metrics = evaluate_vru_tracks(gt, pred, max_distance=2.0)
        self.assertAlmostEqual(metrics.ade_m, 1.0)
        self.assertAlmostEqual(metrics.fde_m, 1.0)

    def test_id_switch_is_counted(self):
        gt = [track("p1", 0.0, 0.0), track("p1", 0.5, 0.6)]
        pred = [track("t1", 0.0, 0.0), track("t2", 0.5, 0.6)]
        metrics = evaluate_vru_tracks(gt, pred)
        self.assertEqual(metrics.id_switches, 1)
        self.assertAlmostEqual(metrics.mota, 0.5)

    def test_class_validation(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.csv"
            path.write_text(
                "track_id,class,timestamp,x,y\np1,car,0.0,0.0,0.0\n",
                encoding="utf-8",
            )
            with self.assertRaises(ValueError):
                load_vru_tracks(path)

    def test_loader_reads_child_and_cyclist(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "tracks.csv"
            path.write_text(
                "track_id,class,timestamp,x,y\n"
                "c1,child,0.0,1.0,2.0\n"
                "b1,cyclist,0.0,3.0,4.0\n",
                encoding="utf-8",
            )
            tracks = load_vru_tracks(path)
            self.assertEqual({item.vru_class for item in tracks}, {"child", "cyclist"})


if __name__ == "__main__":
    unittest.main()
