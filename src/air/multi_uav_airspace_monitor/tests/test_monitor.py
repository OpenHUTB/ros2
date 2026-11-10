import contextlib
import io
import json
import os
import tempfile
import unittest
from pathlib import Path

from multi_uav_airspace_monitor.monitor import (
    UAVTrack,
    Zone,
    evaluate_scenario,
    geofence_alerts,
    pairwise_conflicts,
)
from multi_uav_airspace_monitor.scenario import build_demo_scenario

import main as cli


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

    def test_conflict_inside_horizon_survives_later_global_minimum(self):
        # 20 s 内两机间距从 20 m 线性降到 0 m：全局最近点在 20 s，但 horizon 内
        # 10 s 处已经贴到 10 m，必须报出来，而不是因为最小时刻在窗口外被整对跳过。
        times = [0.0, 20.0]
        track_a = UAVTrack("a", times, [[0, 0, 20], [0, 0, 20]])
        track_b = UAVTrack("b", times, [[20, 0, 20], [0, 0, 20]])
        conflicts = pairwise_conflicts(
            [track_a, track_b], horizon_s=10.0, warning_m=30.0, critical_m=10.0
        )
        self.assertEqual(len(conflicts), 1)
        self.assertLessEqual(conflicts[0].time_s, 10.0 + 1e-9)
        self.assertAlmostEqual(conflicts[0].separation_m, 10.0, places=3)
        self.assertEqual(conflicts[0].severity, "critical")

    def test_pair_outside_horizon_stays_safe(self):
        # 前 10 s 最近仍有 52.5 m（> warning 30 m），更近的时刻出现在窗口外，
        # 这种情形不能因为“整段最小距离很小”而误报。
        times = [0.0, 20.0]
        track_a = UAVTrack("a", times, [[0, 0, 20], [0, 0, 20]])
        track_b = UAVTrack("b", times, [[100, 0, 20], [5, 0, 20]])
        self.assertEqual(
            pairwise_conflicts([track_a, track_b], horizon_s=10.0, warning_m=30.0), []
        )

    def test_invalid_horizon_is_rejected(self):
        track_a = UAVTrack("a", [0, 10], [[-50, 0, 20], [50, 0, 20]])
        track_b = UAVTrack("b", [0, 10], [[50, 0, 20], [-50, 0, 20]])
        with self.assertRaises(ValueError):
            pairwise_conflicts([track_a, track_b], horizon_s=0.0)

    def test_warning_only_conflict_reports_warning(self):
        # 两机保持 20 m 平行飞行：介于 critical(10) 与 warning(30) 之间，
        # 状态应为 warning 而不是 critical。
        summary = evaluate_scenario(
            {
                "horizon_s": 20.0,
                "warning_distance_m": 30.0,
                "critical_distance_m": 10.0,
                "uavs": [
                    {"id": "a", "times": [0, 20], "positions": [[0, 0, 20], [40, 0, 20]]},
                    {"id": "b", "times": [0, 20], "positions": [[0, 20, 20], [40, 20, 20]]},
                ],
            }
        )
        self.assertEqual(summary["status"], "warning")
        self.assertEqual([item["severity"] for item in summary["conflicts"]], ["warning"])

    def test_minimum_separation_is_measured_inside_horizon(self):
        # 全段最小间隔 0 m 出现在 20 s，但 horizon 只到 10 s：窗口内最近 50 m。
        # minimum_separation_m 必须与 conflicts 用同一个窗口，否则会出现
        # “status=safe 但 minimum_separation_m=0” 的自相矛盾输出。
        times = [0.0, 20.0]
        summary = evaluate_scenario(
            {
                "horizon_s": 10.0,
                "warning_distance_m": 30.0,
                "critical_distance_m": 10.0,
                "uavs": [
                    {"id": "a", "times": times, "positions": [[0, 0, 20], [0, 0, 20]]},
                    {"id": "b", "times": times, "positions": [[100, 0, 20], [0, 0, 20]]},
                ],
            }
        )
        self.assertEqual(summary["status"], "safe")
        self.assertEqual(summary["conflicts"], [])
        self.assertAlmostEqual(summary["minimum_separation_m"], 50.0, places=3)
        self.assertAlmostEqual(summary["minimum_separation_full_m"], 0.0, places=3)

    def test_empty_scenario_is_rejected(self):
        with self.assertRaises(ValueError):
            evaluate_scenario({"uavs": []})
        with self.assertRaises(ValueError):
            evaluate_scenario({})

    def test_check_command_exit_codes(self):
        crossing = {
            "horizon_s": 20.0,
            "warning_distance_m": 30.0,
            "critical_distance_m": 10.0,
            "uavs": [
                {"id": "a", "times": [0, 20], "positions": [[-50, 0, 20], [50, 0, 20]]},
                {"id": "b", "times": [0, 20], "positions": [[50, 0, 20], [-50, 0, 20]]},
            ],
        }
        far_apart = {
            "horizon_s": 20.0,
            "warning_distance_m": 30.0,
            "critical_distance_m": 10.0,
            "uavs": [
                {"id": "a", "times": [0, 20], "positions": [[0, 0, 20], [20, 0, 20]]},
                {"id": "b", "times": [0, 20], "positions": [[0, 400, 20], [20, 400, 20]]},
            ],
        }
        for scenario, expected, status in (
            (crossing, 1, "critical"),
            (far_apart, 0, "safe"),
        ):
            handle, path = tempfile.mkstemp(suffix=".json", dir=os.path.dirname(__file__))
            os.close(handle)
            try:
                Path(path).write_text(json.dumps(scenario), encoding="utf-8")
                buffer = io.StringIO()
                with contextlib.redirect_stdout(buffer):
                    code = cli.run_check(Path(path))
                self.assertEqual(code, expected)
                self.assertEqual(json.loads(buffer.getvalue())["status"], status)
            finally:
                os.remove(path)

        handle, path = tempfile.mkstemp(suffix=".json", dir=os.path.dirname(__file__))
        os.close(handle)
        try:
            Path(path).write_text("{not json", encoding="utf-8")
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(cli.run_check(Path(path)), 2)
        finally:
            os.remove(path)


if __name__ == "__main__":
    unittest.main()
