from math import isclose

import pytest

from mujoco_musculoskeletal_arm_reach.kinematics import (
    ArmGeometry,
    forward_kinematics,
    inverse_kinematics,
)


def test_forward_and_inverse_kinematics_round_trip():
    geometry = ArmGeometry()
    for shoulder, elbow in ((0.0, 0.5), (-0.3, 1.5), (0.4, 1.0)):
        x, z = forward_kinematics(shoulder, elbow, geometry)
        solved_shoulder, solved_elbow = inverse_kinematics(x, z, geometry)
        x2, z2 = forward_kinematics(
            solved_shoulder, solved_elbow, geometry
        )
        assert isclose(x, x2, abs_tol=1e-9)
        assert isclose(z, z2, abs_tol=1e-9)


def test_unreachable_target_is_rejected():
    with pytest.raises(ValueError, match='outside'):
        inverse_kinematics(1.0, 0.0)


def test_invalid_link_length_is_rejected():
    with pytest.raises(ValueError):
        ArmGeometry(upper_arm_length_m=0.0)
