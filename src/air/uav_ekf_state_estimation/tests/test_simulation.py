import unittest

import numpy as np

from uav_ekf_state_estimation.simulation import run_simulation


class SimulationTest(unittest.TestCase):
    def test_demo_metrics_are_finite_and_ekf_beats_dead_reckoning(self):
        result = run_simulation(duration_s=25.0, seed=7)
        for value in result.metrics.values():
            self.assertTrue(value == value)
        self.assertLess(result.metrics["ekf_rmse_m"], result.metrics["dead_reckoning_rmse_m"])
        self.assertLess(result.metrics["ekf_rmse_m"], result.metrics["gps_rmse_m"] * 2.5)

    def test_repeatable(self):
        first = run_simulation(duration_s=10.0, seed=7)
        second = run_simulation(duration_s=10.0, seed=7)
        self.assertEqual(first.metrics, second.metrics)

    def test_gps_baseline_keeps_the_measured_altitude(self):
        """纯 GPS 基线只能用 GPS 自己量到的三个分量，不能借真值高度。

        基线一旦用 ``true_position[:, 2]``，垂直误差就从 GPS 自身的 sigma=2.5 m
        塌到厘米级（25 s 运行时 std≈0.06 m），``gps_rmse_m`` 只剩水平项，EKF 相对
        基线的优势会被系统性低估。所以基线高度与真值之差必须保留传感器的离散度。
        """
        result = run_simulation(duration_s=25.0, seed=7)
        vertical = result.gps_position[:, 2] - result.true_position[:, 2]
        self.assertGreater(float(np.std(vertical)), 1.0)
        self.assertAlmostEqual(float(np.mean(np.abs(vertical))), 2.5, delta=1.5)


if __name__ == "__main__":
    unittest.main()
