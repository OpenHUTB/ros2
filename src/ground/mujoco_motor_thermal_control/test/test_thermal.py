from __future__ import annotations

import pytest

from mujoco_motor_thermal_control.thermal import MotorThermalModel


def test_temperature_rises_under_load_and_cools_at_idle() -> None:
    motor = MotorThermalModel()
    loaded = [motor.update(1.5, 0.1) for _ in range(300)]
    assert loaded[-1] > loaded[0] > 25.0
    idle = [motor.update(0.0, 0.1) for _ in range(600)]
    assert idle[-1] < loaded[-1]


def test_thermal_model_rejects_non_positive_time_step() -> None:
    with pytest.raises(ValueError, match="positive"):
        MotorThermalModel().update(1.0, 0.0)
