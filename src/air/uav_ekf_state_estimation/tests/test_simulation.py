import unittest

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


if __name__ == "__main__":
    unittest.main()
