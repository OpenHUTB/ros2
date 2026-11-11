import pytest

from mujoco_motor_thermal_guard.guard import MotorTemperatureGuard


def test_guard_predicts_rising_temperature_and_applies_hysteresis():
    guard = MotorTemperatureGuard(limit_c=40, release_c=35, horizon_s=5)
    assert guard.update(1.0, 30).active is False
    decision = guard.update(2.0, 34)
    assert decision.predicted_temperature_c == pytest.approx(54)
    assert decision.active is True
    assert decision.speed_scale < 1
    assert guard.update(3.0, 34.5).active is True
    assert guard.update(4.0, 32).active is False


def test_guard_rejects_invalid_time_order_and_thresholds():
    with pytest.raises(ValueError):
        MotorTemperatureGuard(limit_c=30, release_c=30)
    guard = MotorTemperatureGuard()
    guard.update(2, 25)
    with pytest.raises(ValueError, match="increase"):
        guard.update(2, 26)
