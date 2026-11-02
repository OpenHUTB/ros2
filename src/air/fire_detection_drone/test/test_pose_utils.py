import math

import pytest
from fire_detection_drone.pose_utils import (
    enu_twist_to_ned,
    ned_pose_to_enu,
    normalize_angle,
    yaw_from_quaternion,
)


def test_yaw_from_identity_quaternion():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)


def test_yaw_from_ninety_degree_rotation():
    angle = math.pi / 2.0
    yaw = yaw_from_quaternion(
        0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0)
    )
    assert yaw == pytest.approx(angle)


def test_ned_pose_to_enu():
    x, y, z, yaw = ned_pose_to_enu(1.0, 2.0, 29.0, 0.3)
    assert (x, y, z, yaw) == pytest.approx((1.0, -2.0, -29.0, -0.3))


def test_enu_twist_to_ned():
    values = enu_twist_to_ned(1.0, 2.0, 0.5, 0.4)
    assert values == pytest.approx((1.0, -2.0, -0.5, -0.4))


def test_normalize_angle_wraps():
    assert abs(normalize_angle(3.0 * math.pi)) == pytest.approx(math.pi)
    assert abs(normalize_angle(-3.0 * math.pi)) == pytest.approx(math.pi)
