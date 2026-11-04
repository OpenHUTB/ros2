from math import isclose, nan

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


@pytest.mark.parametrize('target', ((nan, -0.2), (0.3, nan)))
def test_nonfinite_target_is_rejected(target):
    with pytest.raises(ValueError, match='finite'):
        inverse_kinematics(*target)


def test_shoulder_limit_is_enforced():
    with pytest.raises(ValueError, match='outside'):
        inverse_kinematics(0.0, -0.58)


def test_reach_min_uses_elbow_limit():
    geometry = ArmGeometry()
    assert geometry.reach_min_m == pytest.approx(0.238, abs=0.002)


def test_invalid_link_length_is_rejected():
    with pytest.raises(ValueError):
        ArmGeometry(upper_arm_length_m=0.0)
