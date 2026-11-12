"""阻抗控制律的单元测试。"""

import math

import pytest

from humanoid_knee_impedance.controller import impedance_torque


def test_zero_error_zero_velocity_returns_zero():
    assert impedance_torque(0.3, 0.0, 0.3, 12.0, 2.0, 8.0) == 0.0


def test_positive_deviation_produces_restoring_torque():
    torque = impedance_torque(0.5, 0.0, 0.3, 12.0, 2.0, 8.0)
    assert torque == pytest.approx(-2.4)


def test_damping_opposes_velocity():
    torque = impedance_torque(0.3, 1.5, 0.3, 12.0, 2.0, 8.0)
    assert torque == pytest.approx(-3.0)


def test_torque_is_clamped_to_limit():
    torque = impedance_torque(2.4, 0.0, 0.0, 12.0, 2.0, 8.0)
    assert torque == -8.0
    torque = impedance_torque(-2.4, 0.0, 0.0, 12.0, 2.0, 8.0)
    assert torque == 8.0


def test_nan_and_inf_inputs_return_zero():
    assert impedance_torque(float('nan'), 0.0, 0.3, 12.0, 2.0, 8.0) == 0.0
    assert impedance_torque(0.3, float('inf'), 0.3, 12.0, 2.0, 8.0) == 0.0
    assert impedance_torque(0.3, 0.0, float('nan'), 12.0, 2.0, 8.0) == 0.0
    assert impedance_torque(float('inf'), float('nan'), 0.3,
                            12.0, 2.0, 8.0) == 0.0


def test_invalid_gains_raise_value_error():
    with pytest.raises(ValueError):
        impedance_torque(0.0, 0.0, 0.0, -1.0, 2.0, 8.0)
    with pytest.raises(ValueError):
        impedance_torque(0.0, 0.0, 0.0, 12.0, -0.1, 8.0)
    with pytest.raises(ValueError):
        impedance_torque(0.0, 0.0, 0.0, 12.0, 2.0, 0.0)


def test_output_is_finite_for_extreme_but_valid_inputs():
    torque = impedance_torque(1e6, -1e6, 0.3, 12.0, 2.0, 8.0)
    assert math.isfinite(torque)
    assert abs(torque) <= 8.0
