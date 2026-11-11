import math

import pytest

from airsim_ros2_bringup.transforms import (
    airsim_quat_to_ros_quat_flu,
    body_flu_to_airsim,
    body_ned_to_flu,
    enu_to_ned,
    ned_to_enu,
    quat_xyzw_to_matrix,
    world_enu_to_body_flu,
    yaw_from_quat_xyzw,
)


def test_position_round_trip():
    ned = (1.0, 2.0, 3.0)
    enu = ned_to_enu(ned)
    assert enu == pytest.approx((2.0, 1.0, -3.0))
    assert enu_to_ned(enu) == pytest.approx(ned)


def test_body_axis_conversions_round_trip():
    body_ned = (1.0, 2.0, 3.0)
    body_flu = body_ned_to_flu(body_ned)
    assert body_flu == pytest.approx((1.0, -2.0, -3.0))
    assert body_flu_to_airsim(body_flu) == pytest.approx(body_ned)


def test_level_north_heading_becomes_enu_yaw_90_degrees():
    quaternion = airsim_quat_to_ros_quat_flu((1.0, 0.0, 0.0, 0.0))
    assert yaw_from_quat_xyzw(quaternion) == pytest.approx(math.pi / 2.0)
    assert quaternion == pytest.approx(
        (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    )


def test_world_velocity_is_expressed_in_body_frame():
    orientation = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    body_velocity = world_enu_to_body_flu((0.0, 2.0, 0.0), orientation)
    assert body_velocity == pytest.approx((2.0, 0.0, 0.0))


def test_quaternion_rejects_zero_norm():
    with pytest.raises(ValueError):
        airsim_quat_to_ros_quat_flu((0.0, 0.0, 0.0, 0.0))


def test_quaternion_matrix_is_orthonormal():
    quaternion = airsim_quat_to_ros_quat_flu(
        (0.9238795, 0.0, 0.0, 0.3826834)
    )
    rotation = quat_xyzw_to_matrix(quaternion)
    transpose_product = tuple(
        tuple(
            sum(rotation[index][row] * rotation[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )
    expected = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    for row in range(3):
        assert transpose_product[row] == pytest.approx(expected[row], abs=1e-6)
