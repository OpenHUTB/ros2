import math

import pytest

from mujoco_musculoskeletal_elbow_nn_control.controller import (
    AntagonisticPDController,
    ControlState,
)
from mujoco_musculoskeletal_elbow_nn_control.safety import ActivationLimiter


def test_flexion_error_activates_flexor() -> None:
    command = AntagonisticPDController().command(ControlState(1.0, 0.4, 0.0))
    assert command[0] > command[1]
    assert all(0.0 <= value <= 1.0 for value in command)


def test_extension_error_activates_extensor() -> None:
    command = AntagonisticPDController().command(ControlState(0.2, 1.0, 0.0))
    assert command[1] > command[0]


def test_activation_limiter_rejects_non_finite_values() -> None:
    with pytest.raises(ValueError, match="finite"):
        ActivationLimiter().apply((math.nan, 0.0))


def test_invalid_controller_parameters_are_rejected() -> None:
    with pytest.raises(ValueError):
        AntagonisticPDController(kp=0.0)
