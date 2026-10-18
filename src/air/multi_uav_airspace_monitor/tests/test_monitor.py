import unittest

from multi_uav_airspace_monitor.monitor import (
    UAVTrack,
    Zone,
    evaluate_scenario,
    geofence_alerts,
    pairwise_conflicts,
)
from multi_uav_airspace_monitor.scenario import build_demo_scenario


class MonitorTest(unittest.TestCase):
    def test_geofence_detects_intrusion(self):
        zone = Zone("test", [[0, 0], [10, 0], [10, 10], [0, 10]], ceiling_z=50)
        track = UAVTrack(
            "uav",
            times=[0.0, 1.0, 2.0],
            positions=[[-5, 5, 20], [5, 5, 20], [15, 5, 20]],
        )
        alerts = geofence_alerts(track, [zone])
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0].severity, "critical")
        self.assertAlmostEqual(alerts[0].time_s, 1.0)

    def test_altitude_band_prevents_false_positive(self):
        zone = Zone("test", [[0, 0], [10, 0], [10, 10], [0, 10]], ceiling_z=50)
        track = UAVTrack(
            "uav",
            times=[0.0, 1.0],
            positions=[[-5, 5, 80], [5, 5, 80]],
        )
        self.assertEqual(geofence_alerts(track, [zone]), [])

    def test_head_on_conflict(self):
        track_a = UAVTrack("a", [0, 10], [[-50, 0, 20], [50, 0, 20]])
        track_b = UAVTrack("b", [0, 10], [[50, 0, 20], [-50, 0, 20]])
        conflicts = pairwise_conflicts([track_a, track_b], warning_m=30, critical_m=10)
        self.assertEqual(len(conflicts), 1)
        self.assertEqual(conflicts[0].severity, "critical")
        self.assertLess(conflicts[0].separation_m, 0.01)
        self.assertIn("climb", conflicts[0].advisory)

    def test_parallel_tracks_are_safe(self):
        track_a = UAVTrack("a", [0, 10], [[-50, 0, 20], [50, 0, 20]])
        track_b = UAVTrack("b", [0, 10], [[-50, 50, 20], [50, 50, 20]])
        self.assertEqual(pairwise_conflicts([track_a, track_b], warning_m=30), [])

    def test_demo_scenario_has_conflict_and_geofence_alert(self):
        summary = evaluate_scenario(build_demo_scenario())
        self.assertEqual(summary["status"], "critical")
        self.assertGreaterEqual(len(summary["conflicts"]), 1)
        self.assertTrue(any(alert["kind"] == "geofence" for alert in summary["alerts"]))


if __name__ == "__main__":
    unittest.main()
