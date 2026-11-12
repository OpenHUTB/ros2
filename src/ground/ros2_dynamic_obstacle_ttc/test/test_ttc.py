import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_dynamic_obstacle_ttc"))

from ttc_core import (Cluster, Tracker, WarningGate, cluster_scan,  # noqa: E402
                      time_to_collision)
from sim import head_on, parallel, receding, run_scenario  # noqa: E402


class TestClustering(unittest.TestCase):
    def test_single_blob_is_one_cluster(self):
        ranges = [float("inf")] * 180
        for i in range(87, 94):
            ranges[i] = 2.0
        clusters = cluster_scan(ranges, -math.pi, 2.0 * math.pi / 180)
        self.assertEqual(len(clusters), 1)
        self.assertAlmostEqual(clusters[0].x, 2.0, delta=0.05)
        self.assertAlmostEqual(clusters[0].y, 0.0, delta=0.1)

    def test_two_blobs_are_separated(self):
        ranges = [float("inf")] * 180
        for i in range(20, 24):
            ranges[i] = 1.5
        for i in range(150, 154):
            ranges[i] = 1.5
        clusters = cluster_scan(ranges, -math.pi, 2.0 * math.pi / 180)
        self.assertEqual(len(clusters), 2)


class TestTracking(unittest.TestCase):
    def test_tracker_estimates_velocity(self):
        tracker = Tracker()
        tracker.update([Cluster(3.0, 0.0, 0.1, 5)], 0.0)
        tracks = tracker.update([Cluster(2.8, 0.0, 0.1, 5)], 0.1)
        self.assertEqual(len(tracks), 1)
        self.assertAlmostEqual(tracks[0].vx, -2.0, delta=0.2)

    def test_missed_tracks_are_removed(self):
        tracker = Tracker(max_missed=1)
        tracker.update([Cluster(3.0, 0.0, 0.1, 5)], 0.0)
        tracker.update([], 0.1)
        tracker.update([], 0.2)
        self.assertEqual(len(tracker.tracks), 0)


class TestTTC(unittest.TestCase):
    def test_head_on_time_to_collision(self):
        track = Tracker().update([Cluster(3.0, 0.0, 0.2, 5)], 0.0)[0]
        track.vx, track.vy = -1.0, 0.0
        ttc = time_to_collision(track, robot_radius=0.25, safety_margin=0.1)
        self.assertAlmostEqual(ttc, 2.45, delta=0.05)  # 3.0 - (0.25+0.1+0.2)

    def test_receding_obstacle_has_no_ttc(self):
        track = Tracker().update([Cluster(3.0, 0.0, 0.2, 5)], 0.0)[0]
        track.vx, track.vy = 1.0, 0.0
        self.assertEqual(time_to_collision(track), float("inf"))

    def test_warning_hysteresis(self):
        gate = WarningGate(on_ttc=2.5, off_ttc=3.5)
        self.assertTrue(gate.update(2.0))
        self.assertTrue(gate.update(3.0))   # inside hysteresis
        self.assertFalse(gate.update(3.6))


class TestScenarios(unittest.TestCase):
    def test_head_on_warns(self):
        row = run_scenario(head_on())
        self.assertTrue(row["warned"])
        self.assertFalse(row["false_positive"])

    def test_parallel_does_not_warn(self):
        row = run_scenario(parallel())
        self.assertFalse(row["warned"])
        self.assertIsNone(row["min_ttc_s"])

    def test_wall_blob_is_kept_but_excluded_from_ttc(self):
        ranges = [3.0] * 180
        clusters = cluster_scan(ranges, -math.pi, 2.0 * math.pi / 180)
        self.assertEqual(len(clusters), 1)
        self.assertGreater(clusters[0].radius, 0.8)
        track = Tracker().update(clusters, 0.0)[0]
        self.assertEqual(time_to_collision(track), float("inf"))

    def test_receding_does_not_warn(self):
        row = run_scenario(receding())
        self.assertFalse(row["warned"])


if __name__ == "__main__":
    unittest.main()


