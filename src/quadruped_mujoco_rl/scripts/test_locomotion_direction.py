#!/usr/bin/env python3
"""Regression tests for signed quadruped gait direction."""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from locomotion_core import (  # noqa: E402
    QuadrupedModel,
    TrotGait,
    clamp_speed_command,
    default_model_path,
    simulate,
    speed_direction,
)


class DirectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        model_path = os.environ.get("QUADRUPED_MODEL_PATH", default_model_path())
        cls.robot = QuadrupedModel(model_path)

    def test_speed_direction_uses_deadband_and_sign(self):
        self.assertEqual(speed_direction(0.0), 0)
        self.assertEqual(speed_direction(0.02), 0)
        self.assertEqual(speed_direction(0.3), 1)
        self.assertEqual(speed_direction(-0.3), -1)

    def test_speed_direction_rejects_non_finite_values(self):
        with self.assertRaises(ValueError):
            speed_direction(float("nan"))

    def test_invalid_speed_commands_stop_and_finite_commands_are_clamped(self):
        self.assertEqual(clamp_speed_command(float("nan")), 0.0)
        self.assertEqual(clamp_speed_command(float("inf")), 0.0)
        self.assertEqual(clamp_speed_command(3.0, max_speed=1.2), 1.2)

    def test_gait_rejects_unsupported_direction(self):
        with self.assertRaises(ValueError):
            TrotGait(self.robot, direction=0)

    def test_speed_magnitude_updates_gait_amplitudes_without_phase_reset(self):
        gait = TrotGait(self.robot, amplitude_scale=0.3 / 0.8)
        phase = 0.42
        gait.update(phase)
        first = gait.params()

        gait.set_amplitude_scale(0.9 / 0.8)
        gait.update(phase)
        second = gait.params()

        self.assertGreater(second["a_thigh"], first["a_thigh"])
        self.assertGreater(second["a_calf"], first["a_calf"])
        self.assertAlmostEqual(second["a_thigh"] / first["a_thigh"], 3.0)
        self.assertAlmostEqual(second["a_calf"] / first["a_calf"], 3.0)

    def test_signed_gait_moves_in_requested_direction(self):
        forward = TrotGait(self.robot, direction=1)
        backward = TrotGait(self.robot, direction=-1)

        forward_metrics = simulate(self.robot, forward, seconds=2.0)
        backward_metrics = simulate(self.robot, backward, seconds=2.0)

        self.assertGreater(forward_metrics["dx"], 0.1)
        self.assertLess(backward_metrics["dx"], -0.1)


if __name__ == "__main__":
    unittest.main()

