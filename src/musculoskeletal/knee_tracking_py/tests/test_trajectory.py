"""参考轨迹生成测试。"""

from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from src.config import load_config
from src.trajectory import (
    make_time_vector,
    sine_reference,
    sine_trajectory,
    step_reference,
    step_trajectory,
)

ROOT = Path(__file__).resolve().parents[1]


class TestSineReference(unittest.TestCase):
    def test_starts_at_bias(self) -> None:
        self.assertAlmostEqual(sine_reference(0.0, 30.0, 40.0, 0.4), 30.0, places=12)

    def test_quarter_period_peak(self) -> None:
        # f=0.4 Hz -> T=2.5 s，四分之一周期为 0.625 s
        self.assertAlmostEqual(
            sine_reference(0.625, 30.0, 40.0, 0.4), 70.0, places=9
        )

    def test_period_repeats(self) -> None:
        self.assertAlmostEqual(
            sine_reference(2.5, 30.0, 40.0, 0.4),
            sine_reference(0.0, 30.0, 40.0, 0.4),
            places=9,
        )

    def test_trough(self) -> None:
        self.assertAlmostEqual(
            sine_reference(1.875, 30.0, 40.0, 0.4), -10.0, places=9
        )

    def test_vectorized_shape(self) -> None:
        t = np.linspace(0.0, 5.0, 11)
        values = sine_reference(t, 30.0, 40.0, 0.4)
        self.assertEqual(values.shape, (11,))
        self.assertTrue(np.all(np.isfinite(values)))

    def test_invalid_frequency(self) -> None:
        with self.assertRaises(ValueError):
            sine_reference(0.0, 30.0, 40.0, 0.0)


class TestStepReference(unittest.TestCase):
    def test_before_step(self) -> None:
        self.assertAlmostEqual(step_reference(0.49, 20.0, 50.0, 0.5), 20.0)

    def test_at_step(self) -> None:
        self.assertAlmostEqual(step_reference(0.5, 20.0, 50.0, 0.5), 50.0)

    def test_after_step(self) -> None:
        self.assertAlmostEqual(step_reference(2.0, 20.0, 50.0, 0.5), 50.0)

    def test_vectorized(self) -> None:
        t = np.array([0.0, 0.25, 0.5, 1.0])
        values = step_reference(t, 20.0, 50.0, 0.5)
        np.testing.assert_array_equal(values, [20.0, 20.0, 50.0, 50.0])

    def test_negative_step_time_rejected(self) -> None:
        with self.assertRaises(ValueError):
            step_reference(0.0, 20.0, 50.0, -0.1)


class TestTimeAndConfigTrajectory(unittest.TestCase):
    def test_time_vector_endpoint(self) -> None:
        t = make_time_vector(3.0, 0.005)
        self.assertEqual(t.size, 601)
        self.assertAlmostEqual(t[0], 0.0)
        self.assertAlmostEqual(t[-1], 3.0, places=12)

    def test_invalid_time_vector_args(self) -> None:
        with self.assertRaises(ValueError):
            make_time_vector(0.0, 0.005)
        with self.assertRaises(ValueError):
            make_time_vector(1.0, 0.0)

    def test_config_driven_sine(self) -> None:
        cfg = load_config(ROOT / "config" / "params.yaml")
        t = np.array([0.0, 0.625])
        values = sine_trajectory(t, cfg.trajectory)
        np.testing.assert_allclose(values, [30.0, 70.0], atol=1e-9)

    def test_config_driven_step(self) -> None:
        cfg = load_config(ROOT / "config" / "params.yaml")
        t = np.array([0.0, 0.5, 1.0])
        values = step_trajectory(t, cfg.trajectory)
        np.testing.assert_array_equal(values, [20.0, 50.0, 50.0])


if __name__ == "__main__":
    unittest.main()
