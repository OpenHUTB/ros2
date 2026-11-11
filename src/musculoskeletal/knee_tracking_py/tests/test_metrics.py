"""指标函数在已知构造数据上的正确性测试。"""

from __future__ import annotations

import math
import unittest

import numpy as np

from src.metrics import (
    first_entry_time,
    max_abs_error,
    overshoot_percent,
    rmse,
    settling_time,
    sine_metrics,
    steady_state_error,
    step_metrics,
)


class TestBasicErrors(unittest.TestCase):
    def test_rmse_known(self) -> None:
        self.assertAlmostEqual(rmse([1.0, -1.0], [0.0, 0.0]), 1.0, places=12)

    def test_rmse_constant_offset(self) -> None:
        self.assertAlmostEqual(rmse([3.0, 3.0, 3.0], [1.0, 1.0, 1.0]), 2.0)

    def test_max_abs_error_known(self) -> None:
        self.assertAlmostEqual(max_abs_error([1.0, -3.0, 2.0], [0.0, 0.0, 0.0]), 3.0)

    def test_shape_mismatch_rejected(self) -> None:
        with self.assertRaises(ValueError):
            rmse([1.0], [1.0, 2.0])

    def test_nan_rejected(self) -> None:
        with self.assertRaises(ValueError):
            rmse([math.nan], [0.0])


class TestOvershoot(unittest.TestCase):
    def test_monotonic_no_overshoot(self) -> None:
        self.assertEqual(overshoot_percent([0.0, 5.0, 9.9, 10.0], 10.0, 10.0), 0.0)

    def test_ten_percent_overshoot(self) -> None:
        self.assertAlmostEqual(
            overshoot_percent([0.0, 5.0, 11.0], 10.0, 10.0), 10.0
        )

    def test_downward_step(self) -> None:
        # 10 -> 5，峰值到 4，相对幅值 5 的超调 = 20%
        self.assertAlmostEqual(
            overshoot_percent([10.0, 7.0, 4.0], 5.0, 5.0), 20.0
        )

    def test_zero_amplitude_rejected(self) -> None:
        with self.assertRaises(ValueError):
            overshoot_percent([0.0], 10.0, 0.0)


class TestSettlingAndEntry(unittest.TestCase):
    def test_settling_time_known(self) -> None:
        t = np.arange(6, dtype=float)
        y = np.array([0.0, 2.0, 4.85, 4.9, 5.0, 5.05])
        self.assertEqual(settling_time(t, y, 5.0, 0.2), 2.0)

    def test_settling_requires_hold(self) -> None:
        t = np.arange(6, dtype=float)
        y = np.array([5.0, 5.0, 7.0, 5.0, 5.0, 5.0])
        self.assertEqual(settling_time(t, y, 5.0, 0.2), 3.0)

    def test_settling_none_when_final_outside(self) -> None:
        t = np.arange(6, dtype=float)
        y = np.array([0.0, 2.0, 4.0, 4.2, 4.0, 4.0])
        self.assertIsNone(settling_time(t, y, 5.0, 0.2))

    def test_first_entry_time(self) -> None:
        t = np.arange(6, dtype=float)
        y = np.array([0.0, 2.0, 4.6, 5.0, 6.0, 5.0])
        self.assertEqual(first_entry_time(t, y, 5.0, 0.5), 2.0)

    def test_first_entry_none(self) -> None:
        t = np.arange(4, dtype=float)
        y = np.array([0.0, 1.0, 2.0, 3.0])
        self.assertIsNone(first_entry_time(t, y, 5.0, 0.5))

    def test_already_inside_settles_at_start(self) -> None:
        t = np.arange(4, dtype=float)
        y = np.array([5.0, 5.0, 5.0, 5.0])
        self.assertEqual(settling_time(t, y, 5.0, 0.2), 0.0)


class TestSteadyStateAndStepMetrics(unittest.TestCase):
    def test_steady_state_error_last_window(self) -> None:
        t = np.arange(6, dtype=float)
        y = np.array([0.0, 0.0, 0.0, 3.0, 4.0, 4.0])
        self.assertAlmostEqual(steady_state_error(t, y, 5.0, 1.0), -1.0)

    def test_step_metrics_on_exponential(self) -> None:
        t = np.linspace(0.0, 5.0, 1001)
        y = 50.0 - 30.0 * np.exp(-t / 0.3)
        metrics = step_metrics(t, y, target=50.0, step_amplitude_deg=30.0)
        self.assertEqual(metrics["overshoot_percent"], 0.0)
        self.assertIsNotNone(metrics["settling_time_s"])
        assert metrics["settling_time_s"] is not None
        # 2% 带为 0.6 deg：30*exp(-t/0.3)=0.6 -> t=1.174 s
        self.assertAlmostEqual(float(metrics["settling_time_s"]), 1.174, places=2)
        # ±5 deg：30*exp(-t/0.3)=5 -> t=0.538 s
        self.assertAlmostEqual(float(metrics["wide_band_entry_s"]), 0.538, places=2)
        self.assertAlmostEqual(float(metrics["steady_state_error_deg"]), 0.0, places=3)

    def test_step_metrics_never_enters_band(self) -> None:
        t = np.linspace(0.0, 5.0, 501)
        y = 50.0 - 6.0 * np.ones_like(t)
        metrics = step_metrics(t, y, target=50.0, step_amplitude_deg=30.0)
        self.assertIsNone(metrics["settling_time_s"])
        self.assertIsNone(metrics["wide_band_entry_s"])
        self.assertAlmostEqual(float(metrics["steady_state_error_deg"]), -6.0)

    def test_step_metrics_requires_relative_time(self) -> None:
        t = np.arange(1, 11, dtype=float)
        y = np.full(10, 50.0)
        with self.assertRaises(ValueError):
            step_metrics(t, y, 50.0, 30.0)

    def test_sine_metrics(self) -> None:
        t = np.linspace(0.0, 1.0, 101)
        ref = np.zeros_like(t)
        actual = np.full_like(t, 2.0)
        metrics = sine_metrics(t, actual, ref)
        self.assertAlmostEqual(metrics["rmse_deg"], 2.0)
        self.assertAlmostEqual(metrics["max_abs_error_deg"], 2.0)


if __name__ == "__main__":
    unittest.main()
