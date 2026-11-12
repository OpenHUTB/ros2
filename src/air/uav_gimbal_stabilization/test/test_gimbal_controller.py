#!/usr/bin/env python3
import io
import math
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stderr

test_dir = os.path.dirname(os.path.abspath(__file__))
for module_dir in (os.path.abspath(os.path.join(test_dir, "..", "scripts")), test_dir):
    if os.path.isdir(module_dir):
        sys.path.insert(0, module_dir)

from gimbal_controller import AxisPID, GimbalController, angular_error  # noqa: E402
from gimbal_sim import GimbalSimulation, target_angles  # noqa: E402
from main import main as cli_main  # noqa: E402


def _rmse(errors):
    return math.sqrt(sum(error * error for error in errors) / len(errors))


class GimbalControllerTest(unittest.TestCase):
    def test_pid_output_is_rate_limited(self):
        pid = AxisPID(kp=100.0, max_rate=20.0, deadband=0.0)
        self.assertEqual(pid.update(90.0, 0.1), 20.0)

    def test_deadband_produces_zero_rate(self):
        pid = AxisPID(deadband=0.25)
        self.assertEqual(pid.update(0.1, 0.1), 0.0)

    def test_two_axis_command_contains_errors(self):
        controller = GimbalController()
        command = controller.command(5.0, -3.0, 1.0, -1.0, 0.05)
        self.assertAlmostEqual(command.pan_error, 4.0)
        self.assertAlmostEqual(command.tilt_error, -2.0)

    def test_closed_loop_error_decreases(self):
        simulation = GimbalSimulation(duration=8.0, rate_hz=50.0)
        simulation.run()
        metrics = simulation.metrics()
        self.assertEqual(metrics["samples"], 400)
        self.assertLess(metrics["rmse_deg"], 6.0)
        self.assertGreaterEqual(metrics["saturation_ratio"], 0.0)
        window = len(simulation.rows) // 5
        early_rmse = _rmse(
            [row["error_deg"] for row in simulation.rows[:window]]
        )
        late_rmse = _rmse(
            [row["error_deg"] for row in simulation.rows[-window:]]
        )
        self.assertLess(late_rmse, early_rmse * 0.85)

    def test_target_motion_is_deterministic(self):
        self.assertEqual(target_angles(0.0), target_angles(0.0))
        self.assertTrue(all(abs(value) < 30.0 for value in target_angles(3.0)))

    def test_combined_error(self):
        self.assertAlmostEqual(angular_error(3.0, 4.0, 0.0, 0.0), 5.0)

    def test_csv_time_starts_at_zero(self):
        simulation = GimbalSimulation(duration=0.04, rate_hz=50.0)
        simulation.run()
        self.assertEqual(simulation.rows[0]["t"], 0.0)

    def test_invalid_simulation_parameters_are_rejected(self):
        cases = (
            {"duration": 0.0},
            {"duration": float("nan")},
            {"rate_hz": 0.0},
            {"rate_hz": float("inf")},
        )
        for kwargs in cases:
            with self.subTest(**kwargs):
                with self.assertRaises(ValueError):
                    GimbalSimulation(**kwargs)

    def test_empty_results_do_not_write_outputs(self):
        simulation = GimbalSimulation()
        with tempfile.TemporaryDirectory() as folder:
            csv_path = os.path.join(folder, "empty.csv")
            plot_path = os.path.join(folder, "empty.png")
            simulation.write_csv(csv_path)
            simulation.plot(plot_path)
            self.assertFalse(os.path.exists(csv_path))
            self.assertFalse(os.path.exists(plot_path))

    def test_offline_cli_rejects_unknown_arguments(self):
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                cli_main(["--duraton", "12"])

    def test_offline_cli_rejects_invalid_limits(self):
        cases = (
            ("--duration", "0"),
            ("--duration", "nan"),
            ("--rate", "0"),
            ("--rate", "inf"),
        )
        for option, value in cases:
            with self.subTest(option=option, value=value):
                with redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit):
                        cli_main([option, value])


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
