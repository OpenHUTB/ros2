#!/usr/bin/env python3
import io
import os
import sys
import unittest
from contextlib import redirect_stderr

test_dir = os.path.dirname(os.path.abspath(__file__))
for module_dir in (os.path.abspath(os.path.join(test_dir, "..", "scripts")), test_dir):
    if os.path.isdir(module_dir):
        sys.path.insert(0, module_dir)

from altitude_filter import AltitudeKalman, rmse  # noqa: E402
from altitude_sim import AltitudeSimulation, truth_at  # noqa: E402
from main import main as cli_main  # noqa: E402


class AltitudeFilterTest(unittest.TestCase):
    def test_stationary_barometer_bias_converges(self):
        filt = AltitudeKalman(initial_altitude=0.0)
        for _ in range(100):
            filt.predict(0.0, 0.02)
            filt.update(2.0)
        self.assertAlmostEqual(filt.altitude, 2.0, delta=0.08)

    def test_acceleration_predicts_upward_motion(self):
        filt = AltitudeKalman(initial_altitude=0.0, initial_velocity=0.0)
        altitude, velocity = filt.predict(2.0, 0.1)
        self.assertGreater(altitude, 0.0)
        self.assertGreater(velocity, 0.0)

    def test_closed_loop_fusion_is_better_than_barometer(self):
        simulation = AltitudeSimulation(duration=12.0, rate_hz=50.0)
        metrics = simulation.run()
        self.assertEqual(metrics["samples"], 600)
        self.assertLess(metrics["fused_rmse"], metrics["baro_rmse"] * 0.95)

    def test_truth_takes_off_from_ground_and_reaches_hover(self):
        self.assertEqual(truth_at(0.0), (0.0, 0.0, 0.0))
        self.assertEqual(truth_at(8.0), (8.0, 0.0, 0.0))
        self.assertEqual(truth_at(2.0), truth_at(2.0))
        self.assertGreater(truth_at(2.0)[0], 0.0)
        self.assertGreater(truth_at(2.0)[1], 0.0)

    def test_rmse(self):
        self.assertAlmostEqual(rmse([1.0, 2.0], [0.0, 0.0]), (2.5) ** 0.5)

    def test_offline_cli_rejects_unknown_arguments(self):
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                cli_main(["--duraton", "12"])


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
