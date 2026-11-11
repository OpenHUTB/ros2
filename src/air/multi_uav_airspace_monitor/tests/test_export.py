"""Tests for the per-sample separation series and its CSV export."""

import csv
import json
import tempfile
import unittest
from pathlib import Path

import main as cli
from multi_uav_airspace_monitor.csv_export import FIELDS, write_separation_csv
from multi_uav_airspace_monitor.monitor import (
    UAVTrack,
    separation_rows_for_scenario,
    separation_series,
)

# Two UAVs flying parallel lines 40 m apart, 0..10 s.
PARALLEL = {
    "uavs": [
        {"id": "uav_a", "times": [0.0, 5.0, 10.0],
         "positions": [[0, 0, 20], [5, 0, 20], [10, 0, 20]]},
        {"id": "uav_b", "times": [0.0, 5.0, 10.0],
         "positions": [[0, 40, 20], [5, 40, 20], [10, 40, 20]]},
    ],
    "zones": [],
    "horizon_s": 4.0,
    "step_s": 1.0,
}


def make_track(uav_id, positions, times=None):
    if times is None:
        times = [float(index) for index in range(len(positions))]
    return UAVTrack(uav_id=uav_id, times=times, positions=positions)


class SeparationSeriesTest(unittest.TestCase):
    def test_series_matches_constant_separation(self):
        rows = separation_rows_for_scenario(PARALLEL)
        self.assertTrue(rows)
        self.assertEqual({row["separation_m"] for row in rows}, {40.0})
        self.assertEqual({(row["uav_a"], row["uav_b"]) for row in rows}, {("uav_a", "uav_b")})
        # horizon 4 s, step 1 s -> 0, 1, 2, 3, 4 s
        self.assertEqual(sorted({row["time_s"] for row in rows}), [0.0, 1.0, 2.0, 3.0, 4.0])

    def test_series_is_sorted_and_follows_step(self):
        rows = separation_rows_for_scenario(PARALLEL)
        self.assertEqual(
            rows, sorted(rows, key=lambda row: (row["time_s"], row["uav_a"], row["uav_b"]))
        )
        steps = [row["time_s"] for row in rows]
        for first, second in zip(steps, steps[1:]):
            self.assertAlmostEqual(second - first, 1.0)

    def test_series_on_three_uavs_covers_every_pair(self):
        scenario = {
            "uavs": [
                {"id": "a", "times": [0, 5], "positions": [[0, 0, 10], [0, 0, 10]]},
                {"id": "b", "times": [0, 5], "positions": [[0, 20, 10], [0, 20, 10]]},
                {"id": "c", "times": [0, 5], "positions": [[0, 0, 40], [0, 0, 40]]},
            ],
            "horizon_s": 2.0,
            "step_s": 1.0,
        }
        rows = separation_rows_for_scenario(scenario)
        pairs = {(row["uav_a"], row["uav_b"]) for row in rows}
        self.assertEqual(pairs, {("a", "b"), ("a", "c"), ("b", "c")})
        distances = {(row["uav_a"], row["uav_b"]): row["separation_m"] for row in rows}
        self.assertAlmostEqual(distances[("a", "b")], 20.0)
        self.assertAlmostEqual(distances[("a", "c")], 30.0)
        self.assertAlmostEqual(distances[("b", "c")], (20.0 ** 2 + 30.0 ** 2) ** 0.5, places=2)

    def test_series_window_respects_horizon(self):
        tracks = [
            make_track("a", [[0, 0, 0], [10, 0, 0]], times=[0.0, 10.0]),
            make_track("b", [[0, 0, 5], [10, 0, 5]], times=[0.0, 10.0]),
        ]
        full = separation_series(tracks, step_s=1.0, horizon_s=None)
        window = separation_series(tracks, step_s=1.0, horizon_s=3.0)
        self.assertAlmostEqual(max(row["time_s"] for row in full), 10.0)
        self.assertAlmostEqual(max(row["time_s"] for row in window), 3.0)
        self.assertLess(len(window), len(full))

    def test_series_is_empty_for_single_uav(self):
        scenario = {"uavs": [{"id": "solo", "times": [0, 1],
                             "positions": [[0, 0, 0], [1, 0, 0]]}]}
        self.assertEqual(separation_rows_for_scenario(scenario), [])


class CsvExportTest(unittest.TestCase):
    def test_csv_round_trip(self):
        rows = separation_rows_for_scenario(PARALLEL)
        with tempfile.TemporaryDirectory() as tmp:
            target = write_separation_csv(rows, Path(tmp) / "sep.csv")
            with target.open(encoding="utf-8", newline="") as handle:
                reader = csv.DictReader(handle)
                self.assertEqual(tuple(reader.fieldnames or ()), FIELDS)
                parsed = list(reader)
            self.assertEqual(len(parsed), len(rows))
            self.assertEqual(parsed[0], {key: str(rows[0][key]) for key in FIELDS})
            self.assertNotIn("\r\n", target.read_text(encoding="utf-8"))

    def test_csv_header_only_when_no_pair(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = write_separation_csv([], Path(tmp) / "empty.csv")
            self.assertEqual(
                target.read_text(encoding="utf-8").splitlines(), [",".join(FIELDS)]
            )

    def test_csv_creates_missing_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = write_separation_csv([], Path(tmp) / "nested" / "deep" / "sep.csv")
            self.assertTrue(target.exists())


class CliCsvTest(unittest.TestCase):
    def test_check_writes_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "scenario.json"
            config.write_text(json.dumps(PARALLEL), encoding="utf-8")
            csv_path = Path(tmp) / "out" / "sep.csv"
            code = cli.main(["check", "--config", str(config), "--csv", str(csv_path)])
            self.assertEqual(code, 0)  # 40 m parallel flight is safe
            with csv_path.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 5)
            self.assertEqual(rows[0]["separation_m"], "40.0")

    def test_demo_writes_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "demo_sep.csv"
            code = cli.main(["demo", "--output", str(Path(tmp) / "out"),
                             "--csv", str(csv_path)])
            self.assertEqual(code, 0)
            self.assertTrue(csv_path.exists())
            self.assertEqual(
                csv_path.read_text(encoding="utf-8").splitlines()[0], ",".join(FIELDS)
            )


if __name__ == "__main__":
    unittest.main()
