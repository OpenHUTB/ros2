"""指标定义的解析校验（RMS、峰值、VDV）与指标字典的一致性。"""

import unittest

import numpy as np

from speed_bump_ride_comfort.comfort import (
    analysis_window,
    passage_metrics,
    peak_abs,
    rms,
    vibration_dose_value,
)
from speed_bump_ride_comfort.vehicle import Passage


def synthetic_passage(accel_value: float, duration_s: float = 1.0, dt: float = 1.0e-3,
                      speed_mps: float = 2.0) -> Passage:
    """构造一段常值加速度的时间历程，便于手算期望值。"""
    steps = int(round(duration_s / dt)) + 1
    time = dt * np.arange(steps)
    accel = np.full(steps, float(accel_value))
    return Passage(
        time_s=time,
        road_height_m=np.zeros(steps),
        sprung_position_m=np.zeros(steps),
        sprung_accel_mps2=accel,
        travel_m=np.full(steps, 0.01),
        tire_load_ratio=np.ones(steps),
        speed_mps=speed_mps,
    )


class MetricDefinitionTest(unittest.TestCase):
    def test_peak_abs(self):
        self.assertAlmostEqual(peak_abs(np.array([-3.0, 1.0, 2.5])), 3.0)

    def test_rms_of_constant_signal_equals_the_constant(self):
        self.assertAlmostEqual(rms(np.full(500, 2.5)), 2.5, places=12)

    def test_rms_of_sine_matches_analytic_value(self):
        t = np.linspace(0.0, 1.0, 10001)
        self.assertAlmostEqual(rms(2.0 * np.sin(2.0 * np.pi * t)), 2.0 / np.sqrt(2.0),
                               places=3)

    def test_vdv_of_constant_acceleration(self):
        # VDV = (∫ a⁴ dt)^(1/4) = a * T^(1/4)
        self.assertAlmostEqual(vibration_dose_value(np.full(1001, 2.0), 1.0e-3), 2.0,
                               places=9)

    def test_vdv_doubles_with_amplitude(self):
        dt = 1.0e-3
        single = vibration_dose_value(np.full(501, 1.5), dt)
        double = vibration_dose_value(np.full(501, 3.0), dt)
        self.assertAlmostEqual(double, 2.0 * single, places=9)

    def test_empty_and_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            peak_abs(np.array([]))
        with self.assertRaises(ValueError):
            rms(np.array([]))
        with self.assertRaises(ValueError):
            vibration_dose_value(np.array([]), 1.0e-3)
        with self.assertRaises(ValueError):
            vibration_dose_value(np.array([1.0]), 0.0)


class PassageMetricsTest(unittest.TestCase):
    def test_metrics_of_constant_two_mps2(self):
        metrics = passage_metrics(synthetic_passage(2.0), limit_mps2=1.5,
                                  travel_limit_mm=5.0)
        self.assertAlmostEqual(metrics["peak_accel_mps2"], 2.0, places=6)
        self.assertAlmostEqual(metrics["rms_accel_mps2"], 2.0, places=6)
        self.assertAlmostEqual(metrics["vdv_mps1_75"], 2.0, places=6)
        self.assertAlmostEqual(metrics["suspension_travel_mm"], 10.0, places=6)
        self.assertAlmostEqual(metrics["speed_kmh"], 7.2, places=6)

    def test_threshold_and_travel_flags(self):
        over = passage_metrics(synthetic_passage(2.0), limit_mps2=1.5,
                               travel_limit_mm=5.0)
        self.assertTrue(over["exceeds_comfort_limit"])
        self.assertTrue(over["exceeds_travel_limit"])
        under = passage_metrics(synthetic_passage(2.0), limit_mps2=9.0,
                                travel_limit_mm=50.0)
        self.assertFalse(under["exceeds_comfort_limit"])
        self.assertFalse(under["exceeds_travel_limit"])
        self.assertIsNone(passage_metrics(synthetic_passage(2.0))["exceeds_travel_limit"])

    def test_analysis_window_excludes_the_approach_segment(self):
        time = np.array([-0.2, -0.1, 0.0, 0.1, 0.2])
        passage = Passage(
            time_s=time,
            road_height_m=np.zeros(5),
            sprung_position_m=np.zeros(5),
            sprung_accel_mps2=np.zeros(5),
            travel_m=np.zeros(5),
            tire_load_ratio=np.ones(5),
            speed_mps=2.0,
        )
        mask = analysis_window(passage)
        self.assertEqual(int(mask.sum()), 3)
        self.assertTrue(bool(mask[2]))

    def test_tire_contact_loss_is_reported(self):
        passage = synthetic_passage(2.0)
        passage.tire_load_ratio[10] = -0.2
        metrics = passage_metrics(passage)
        self.assertAlmostEqual(metrics["tire_load_min_ratio"], -0.2, places=6)
        self.assertAlmostEqual(metrics["tire_load_peak_ratio"], 1.0, places=6)


if __name__ == "__main__":
    unittest.main()
