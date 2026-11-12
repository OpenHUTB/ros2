"""键盘控制的按键映射、输入限幅与退出行为。"""

import contextlib
import io
import unittest
from unittest.mock import MagicMock, patch

from mujoco_muscle_gripper import manual


class TestManual(unittest.TestCase):
    def test_command_mapping(self):
        cases = (
            ("w", 0.45, 0.06),
            ("s", 0.35, 0.06),
            ("r", 0.40, 0.07),
            ("f", 0.40, 0.05),
            (" ", 0.00, 0.06),
            ("x", 0.40, 0.06),
        )
        for key, expected_activation, expected_height in cases:
            with self.subTest(key=key):
                activation, height = manual.command(key, 0.40, 0.06)
                self.assertAlmostEqual(activation, expected_activation)
                self.assertAlmostEqual(height, expected_height)

    def test_command_limits(self):
        cases = (
            ("w", 0.98, 0.06, 1.00, 0.06),
            ("s", 0.02, 0.06, 0.00, 0.06),
            ("r", 0.40, 0.149, 0.40, 0.15),
            ("f", 0.40, 0.001, 0.40, 0.00),
        )
        for key, activation, height, expected_activation, expected_height in cases:
            with self.subTest(key=key):
                result_activation, result_height = manual.command(
                    key, activation, height
                )
                self.assertAlmostEqual(result_activation, expected_activation)
                self.assertAlmostEqual(result_height, expected_height)

    def test_quit_stops_before_processing_or_advancing(self):
        with patch.object(manual, "_make", return_value=(MagicMock(), MagicMock())), \
             patch.object(manual, "command") as command, \
             patch.object(manual.mujoco, "mj_step") as step, \
             contextlib.redirect_stdout(io.StringIO()):
            records = manual.run(keys="qwr")
        self.assertEqual(records, [])
        command.assert_not_called()
        step.assert_not_called()
