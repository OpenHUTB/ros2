import pytest

from mujoco_motor_thermal_guard.guard import MotorTemperatureGuard, update_guard_sample


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


def test_invalid_temperature_sample_is_discarded_without_mutating_guard():
    guard = MotorTemperatureGuard()
    first, error = update_guard_sample(guard, 2.0, 25.0)
    assert first is not None
    assert error is None

    discarded, error = update_guard_sample(guard, 1.0, 80.0)
    assert discarded is None
    assert error is not None and "increase" in error
    decision, error = update_guard_sample(guard, 3.0, 27.0)
    assert decision is not None
    assert decision.predicted_temperature_c == pytest.approx(37.0)
    assert error is None
