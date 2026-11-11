"""PD 控制器与激活映射测试。"""

from __future__ import annotations

import math
import unittest

from src.config import ControllerConfig
from src.controller import PDController


def make_controller() -> PDController:
    return PDController(
        ControllerConfig(kp=30.0, kd=3.0, limit_epsilon_deg=0.01),
        max_torque=17.5,
        angle_min_deg=-15.0,
        angle_max_deg=90.0,
    )


class TestPDMapping(unittest.TestCase):
    def setUp(self) -> None:
        self.ctrl = make_controller()

    def test_positive_error_uses_flexor_only(self) -> None:
        cmd = self.ctrl.compute(theta=0.0, omega=0.0, theta_des=math.radians(10.0))
        self.assertTrue(cmd.valid)
        self.assertGreater(cmd.flex, 0.0)
        self.assertEqual(cmd.ext, 0.0)
        self.assertGreater(cmd.tau_pd, 0.0)

    def test_negative_error_uses_extensor_only(self) -> None:
        cmd = self.ctrl.compute(theta=math.radians(10.0), omega=0.0, theta_des=0.0)
        self.assertTrue(cmd.valid)
        self.assertEqual(cmd.flex, 0.0)
        self.assertGreater(cmd.ext, 0.0)
        self.assertLess(cmd.tau_pd, 0.0)

    def test_activation_normalization_mid_range(self) -> None:
        cmd = self.ctrl.compute(theta=0.0, omega=0.0, theta_des=0.1)
        self.assertAlmostEqual(cmd.tau_pd, 3.0, places=12)
        self.assertAlmostEqual(cmd.flex, 3.0 / 17.5, places=12)
        self.assertEqual(cmd.ext, 0.0)

    def test_flexor_saturation(self) -> None:
        cmd = self.ctrl.compute(theta=0.0, omega=0.0, theta_des=1.0)
        self.assertEqual(cmd.flex, 1.0)
        self.assertEqual(cmd.reason, "flexor_saturated")

    def test_extensor_saturation(self) -> None:
        # 构造大的正角速度，让 PD 力矩为负并超过 -17.5 N·m
        cmd = self.ctrl.compute(
            theta=math.radians(30.0), omega=40.0, theta_des=math.radians(30.0)
        )
        self.assertEqual(cmd.ext, 1.0)
        self.assertEqual(cmd.reason, "extensor_saturated")

    def test_derivative_term_can_flip_sign(self) -> None:
        cmd = self.ctrl.compute(
            theta=0.0, omega=2.0, theta_des=math.radians(1.0)
        )
        self.assertLess(cmd.tau_pd, 0.0)
        self.assertEqual(cmd.flex, 0.0)
        self.assertGreater(cmd.ext, 0.0)

    def test_zero_error_zero_velocity_is_quiet(self) -> None:
        cmd = self.ctrl.compute(theta=math.radians(30.0), omega=0.0,
                                theta_des=math.radians(30.0))
        self.assertEqual(cmd.flex, 0.0)
        self.assertEqual(cmd.ext, 0.0)
        self.assertTrue(cmd.valid)


class TestLimitProtection(unittest.TestCase):
    def setUp(self) -> None:
        self.ctrl = make_controller()

    def test_upper_limit_stops_flexor(self) -> None:
        theta = math.radians(89.995)
        cmd = self.ctrl.compute(theta=theta, omega=0.0, theta_des=math.radians(90.0))
        self.assertEqual(cmd.flex, 0.0)
        self.assertEqual(cmd.reason, "upper_limit_flexor_off")

    def test_upper_limit_keeps_extensor(self) -> None:
        theta = math.radians(89.995)
        cmd = self.ctrl.compute(theta=theta, omega=0.0, theta_des=math.radians(80.0))
        self.assertEqual(cmd.flex, 0.0)
        self.assertGreater(cmd.ext, 0.0)
        self.assertEqual(cmd.reason, "ok")

    def test_lower_limit_stops_extensor(self) -> None:
        theta = math.radians(-14.995)
        cmd = self.ctrl.compute(theta=theta, omega=0.0, theta_des=math.radians(-15.0))
        self.assertEqual(cmd.ext, 0.0)
        self.assertEqual(cmd.reason, "lower_limit_extensor_off")

    def test_lower_limit_keeps_flexor(self) -> None:
        theta = math.radians(-14.995)
        cmd = self.ctrl.compute(theta=theta, omega=0.0, theta_des=math.radians(-10.0))
        self.assertGreater(cmd.flex, 0.0)
        self.assertEqual(cmd.ext, 0.0)
        self.assertEqual(cmd.reason, "ok")


class TestInvalidInput(unittest.TestCase):
    def setUp(self) -> None:
        self.ctrl = make_controller()

    def test_nan_theta_invalidates(self) -> None:
        cmd = self.ctrl.compute(theta=math.nan, omega=0.0, theta_des=0.0)
        self.assertFalse(cmd.valid)
        self.assertEqual((cmd.flex, cmd.ext), (0.0, 0.0))
        self.assertEqual(cmd.reason, "invalid_input")

    def test_inf_omega_invalidates(self) -> None:
        cmd = self.ctrl.compute(theta=0.0, omega=math.inf, theta_des=0.0)
        self.assertFalse(cmd.valid)
        self.assertEqual((cmd.flex, cmd.ext), (0.0, 0.0))

    def test_target_above_upper_limit_invalidates(self) -> None:
        cmd = self.ctrl.compute(theta=0.0, omega=0.0, theta_des=math.radians(100.0))
        self.assertFalse(cmd.valid)
        self.assertEqual((cmd.flex, cmd.ext), (0.0, 0.0))

    def test_target_below_lower_limit_invalidates(self) -> None:
        cmd = self.ctrl.compute(theta=0.0, omega=0.0, theta_des=math.radians(-20.0))
        self.assertFalse(cmd.valid)
        self.assertEqual(cmd.reason, "invalid_target")

    def test_non_numeric_input_invalidates(self) -> None:
        cmd = self.ctrl.compute(theta="0", omega=0.0, theta_des=0.0)  # type: ignore[arg-type]
        self.assertFalse(cmd.valid)

    def test_invalid_command_outputs_finite_zero(self) -> None:
        cmd = self.ctrl.compute(theta=math.nan, omega=math.nan, theta_des=math.nan)
        for value in (cmd.flex, cmd.ext, cmd.tau_pd, cmd.error_deg):
            self.assertTrue(math.isfinite(value))


if __name__ == "__main__":
    unittest.main()
