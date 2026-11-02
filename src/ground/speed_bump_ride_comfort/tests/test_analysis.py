"""扫速、建议车速与演示数据集的逻辑校验。"""

import unittest

import numpy as np

from speed_bump_ride_comfort.analysis import (
    build_demo,
    recommend_max_speed,
    speed_curve,
    sweep,
)
from speed_bump_ride_comfort.bump import SpeedBump
from speed_bump_ride_comfort.vehicle import QuarterCar


def fake_curve(speeds, peaks, min_load=None):
    speeds = np.asarray(speeds, dtype=float)
    peaks = np.asarray(peaks, dtype=float)
    return {
        "speeds_mps": speeds,
        "peak_accel_mps2": peaks,
        "tire_load_min_ratio": np.ones_like(peaks) if min_load is None else np.asarray(min_load, dtype=float),
        "suspension_travel_mm": np.full_like(peaks, 40.0),
    }


class RecommendSpeedTest(unittest.TestCase):
    def test_returns_highest_speed_within_limit(self):
        curve = fake_curve([1, 2, 3, 4], [1.0, 2.0, 5.0, 9.0])
        advice = recommend_max_speed(curve, 4.0)
        self.assertEqual(advice["speed_mps"], 2.0)
        self.assertEqual(advice["next_speed_mps"], 3.0)
        self.assertEqual(advice["next_peak_accel_mps2"], 5.0)

    def test_all_speeds_within_limit_returns_the_last_one(self):
        advice = recommend_max_speed(fake_curve([1, 2, 3], [1.0, 1.5, 2.0]), 9.0)
        self.assertEqual(advice["speed_mps"], 3.0)
        self.assertIsNone(advice["next_speed_mps"])

    def test_no_speed_within_limit_returns_none(self):
        advice = recommend_max_speed(fake_curve([1, 2, 3], [7.0, 8.0, 9.0]), 3.0)
        self.assertIsNone(advice["speed_mps"])
        self.assertEqual(advice["first_speed_mps"], 1.0)

    def test_invalid_limit_is_rejected(self):
        with self.assertRaises(ValueError):
            recommend_max_speed(fake_curve([1, 2], [1.0, 2.0]), 0.0)


class SweepTest(unittest.TestCase):
    def setUp(self):
        self.vehicle = QuarterCar()
        self.bump = SpeedBump()

    def test_sweep_returns_one_passage_per_speed(self):
        passages = sweep(self.vehicle, self.bump, (1.5, 3.0), dt=2.0e-3)
        self.assertEqual(len(passages), 2)
        self.assertAlmostEqual(passages[0].speed_mps, 1.5)

    def test_speed_curve_matches_sweep_lengths_and_grows(self):
        curve = speed_curve(self.vehicle, self.bump, dt=2.0e-3,
                            v_min_mps=1.0, v_max_mps=5.0, step_mps=1.0)
        self.assertEqual(len(curve["speeds_mps"]), 5)
        peaks = curve["peak_accel_mps2"]
        self.assertTrue(np.all(np.diff(peaks) > 0.0))
        self.assertTrue(np.all(curve["tire_load_min_ratio"] <= 1.0 + 1e-9))

    def test_invalid_sweep_range_is_rejected(self):
        with self.assertRaises(ValueError):
            speed_curve(self.vehicle, self.bump, step_mps=0.0)
        with self.assertRaises(ValueError):
            speed_curve(self.vehicle, self.bump, v_min_mps=5.0, v_max_mps=5.0)


class DemoDatasetTest(unittest.TestCase):
    def test_demo_bundle_is_consistent(self):
        demo = build_demo(
            speeds_mps=(1.5, 3.0),
            dt=2.0e-3,
            sweep_kwargs={"v_min_mps": 1.0, "v_max_mps": 5.0, "step_mps": 1.0},
        )
        self.assertEqual(len(demo["passages"]), 2)
        self.assertEqual(len(demo["rows"]), 2)
        self.assertEqual([entry["label"] for entry in demo["advice"]], ["舒适", "可接受"])
        # 每一级建议车速都应满足自己的阈值，且不超过下一档阈值对应的车速
        speeds = [entry["speed_mps"] for entry in demo["advice"] if entry["speed_mps"]]
        for entry in demo["advice"]:
            if entry["speed_mps"] is None:
                continue
            self.assertLessEqual(entry["peak_accel_mps2"], entry["limit_mps2"] + 1e-9)
        self.assertEqual(speeds, sorted(speeds))
        self.assertAlmostEqual(demo["vehicle_summary"]["damping_ratio"], 0.278, places=3)
        self.assertAlmostEqual(demo["bump_summary"]["reference_area_m2"], 0.0458, places=4)

    def test_metrics_table_flags_follow_the_primary_limit(self):
        demo = build_demo(
            speeds_mps=(1.5, 8.0),
            dt=2.0e-3,
            sweep_kwargs={"v_min_mps": 1.0, "v_max_mps": 5.0, "step_mps": 2.0},
        )
        self.assertFalse(demo["rows"][0]["exceeds_comfort_limit"])
        self.assertTrue(demo["rows"][1]["exceeds_comfort_limit"])


if __name__ == "__main__":
    unittest.main()
