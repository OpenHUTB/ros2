import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from uav_gps_integrity_guard.core import (
    GpsIntegrityConfig,
    GpsIntegrityGuard,
    GpsSample,
)
from uav_gps_integrity_guard.runtime_support import (
    CSV_COLUMNS,
    IntegrityCsvJournal,
    decode_gps_sample,
    validate_simulator_parameters,
)


class GpsIntegrityTests(unittest.TestCase):
    def sample(self, t, x, reported=3.0, accuracy=0.9, imu=3.0):
        return GpsSample(t, x, 0.0, 25.0, reported, accuracy, imu)

    def test_clean_track_is_valid(self):
        guard = GpsIntegrityGuard()
        result = guard.evaluate(self.sample(0.0, 0.0))
        result = guard.evaluate(self.sample(0.2, 0.6))
        self.assertEqual(result.state, "VALID")

    def test_position_jump_is_rejected(self):
        guard = GpsIntegrityGuard()
        guard.evaluate(self.sample(0.0, 0.0))
        result = guard.evaluate(self.sample(0.2, 8.0, reported=3.0))
        self.assertEqual(result.state, "REJECTED")

    def test_imu_residual_raises_risk(self):
        guard = GpsIntegrityGuard()
        guard.evaluate(self.sample(0.0, 0.0))
        result = guard.evaluate(self.sample(0.2, 0.6, reported=1.0, imu=6.0))
        self.assertGreater(result.risk, 0.0)

    def test_poor_accuracy_is_rejected(self):
        result = GpsIntegrityGuard().evaluate(self.sample(0.0, 0.0, accuracy=7.0))
        self.assertEqual(result.state, "REJECTED")

    def test_invalid_speed_rejected(self):
        with self.assertRaises(ValueError):
            GpsIntegrityGuard().evaluate(self.sample(0.0, 0.0, reported=-1.0))

    def test_non_increasing_time_rejected(self):
        guard = GpsIntegrityGuard()
        guard.evaluate(self.sample(1.0, 3.0))
        with self.assertRaises(ValueError):
            guard.evaluate(self.sample(1.0, 3.0))

    def test_recommendation_for_spoof(self):
        guard = GpsIntegrityGuard()
        result = guard.evaluate(self.sample(0.0, 0.0, accuracy=7.0))
        self.assertEqual(result.recommendation, "REJECT_GPS_USE_DEADRECKONING")

    def test_risk_is_bounded(self):
        result = GpsIntegrityGuard().evaluate(
            self.sample(0.0, 0.0, reported=100.0, accuracy=100.0, imu=0.0)
        )
        self.assertGreaterEqual(result.risk, 0.0)
        self.assertLessEqual(result.risk, 1.0)

    def test_state_requires_consecutive_frame_confirmation(self):
        guard = GpsIntegrityGuard(GpsIntegrityConfig(switch_frames=2))
        guard.evaluate(self.sample(0.0, 0.0))
        first = guard.evaluate(
            self.sample(0.2, 0.6, reported=1.0, accuracy=3.0, imu=6.0)
        )
        second = guard.evaluate(
            self.sample(0.4, 1.2, reported=1.0, accuracy=3.0, imu=6.0)
        )
        self.assertEqual(first.state, "VALID")
        self.assertEqual(second.state, "SUSPECT")

    def test_complete_suspect_rejected_valid_sequence(self):
        guard = GpsIntegrityGuard(GpsIntegrityConfig(switch_frames=2))
        states = [
            guard.evaluate(self.sample(0.0, 0.0)).state,
            guard.evaluate(
                self.sample(0.2, 0.6, reported=1.0, accuracy=3.0, imu=6.0)
            ).state,
            guard.evaluate(
                self.sample(0.4, 1.2, reported=1.0, accuracy=3.0, imu=6.0)
            ).state,
            guard.evaluate(
                self.sample(0.6, 1.8, reported=3.0, accuracy=7.0, imu=3.0)
            ).state,
            guard.evaluate(self.sample(0.8, 2.4)).state,
            guard.evaluate(self.sample(1.0, 3.0)).state,
        ]
        self.assertEqual(
            states,
            ["VALID", "VALID", "SUSPECT", "REJECTED", "REJECTED", "VALID"],
        )

    def test_invalid_json_rejected(self):
        with self.assertRaisesRegex(ValueError, "invalid GPS sample JSON"):
            decode_gps_sample("{not-json")

    def test_missing_sample_field_rejected(self):
        payload = asdict(self.sample(0.0, 0.0))
        del payload["imu_speed_mps"]
        with self.assertRaisesRegex(ValueError, "missing fields"):
            decode_gps_sample(json.dumps(payload))

    def test_unknown_sample_field_rejected(self):
        payload = asdict(self.sample(0.0, 0.0))
        payload["gps_week"] = 2300
        with self.assertRaisesRegex(ValueError, "unknown fields"):
            decode_gps_sample(json.dumps(payload))

    def test_csv_journal_records_header_samples_and_transitions(self):
        guard = GpsIntegrityGuard(GpsIntegrityConfig(switch_frames=2))
        samples = [
            self.sample(0.0, 0.0),
            self.sample(0.2, 0.6, reported=1.0, accuracy=3.0, imu=6.0),
            self.sample(0.4, 1.2, reported=1.0, accuracy=3.0, imu=6.0),
            self.sample(0.6, 1.8, reported=3.0, accuracy=7.0, imu=3.0),
        ]
        with TemporaryDirectory() as directory:
            path = Path(directory) / "integrity.csv"
            with IntegrityCsvJournal(path) as journal:
                for sample in samples:
                    journal.write(sample, guard.evaluate(sample))
            with path.open(newline="", encoding="utf-8") as csv_file:
                rows = list(csv.reader(csv_file))

        self.assertEqual(tuple(rows[0]), CSV_COLUMNS)
        self.assertEqual(len(rows), len(samples) + 1)
        self.assertEqual(
            [row[1] for row in rows[1:]],
            ["VALID", "VALID", "SUSPECT", "REJECTED"],
        )

    def test_simulator_parameters_reject_non_finite_or_out_of_range_values(self):
        for rate_hz, duration_s in (
            (0.0, 0.0),
            (-1.0, 0.0),
            (math.nan, 0.0),
            (math.inf, 0.0),
            (5.0, -1.0),
            (5.0, math.nan),
            (5.0, math.inf),
        ):
            with self.subTest(rate_hz=rate_hz, duration_s=duration_s):
                with self.assertRaises(ValueError):
                    validate_simulator_parameters(rate_hz, duration_s)

    def test_valid_simulator_parameters_are_preserved(self):
        self.assertEqual(validate_simulator_parameters(5.0, 15.0), (5.0, 15.0))


if __name__ == "__main__":
    unittest.main()
