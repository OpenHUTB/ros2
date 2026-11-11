import math

import pytest

from mujoco_musculoskeletal_arm_reach.controller import (
    ArmControlState,
    MuscleArmController,
    MUSCLE_NAMES,
)


def state(**overrides):
    values = {
        'target_x_m': 0.38,
        'target_z_m': -0.17,
        'shoulder_rad': 0.0,
        'elbow_rad': 0.0,
        'shoulder_velocity_rad_s': 0.0,
        'elbow_velocity_rad_s': 0.0,
    }
    values.update(overrides)
    return ArmControlState(**values)


def test_command_has_six_bounded_activations():
    command = MuscleArmController().command(state())
    assert len(command) == len(MUSCLE_NAMES) == 6
    assert all(0.0 <= value <= 1.0 for value in command)


def test_expected_agonist_is_activated_for_reaching_target():
    command = MuscleArmController().command(state())
    shoulder_flexor, shoulder_extensor, elbow_flexor, elbow_extensor = command[:4]
    assert shoulder_extensor > shoulder_flexor
    assert elbow_flexor > elbow_extensor


def test_nan_control_state_is_rejected():
    with pytest.raises(ValueError, match='finite'):
        MuscleArmController().command(state(shoulder_rad=math.nan))


def test_unreachable_target_is_rejected_by_controller():
    with pytest.raises(ValueError):
        MuscleArmController().command(state(target_x_m=1.0))
