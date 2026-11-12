"""Unit tests for AirSim IMU coordinate conversions."""

from __future__ import annotations

import math

import pytest

from airsim_ros2_imu.transforms import (
    airsim_quat_to_ros_quat_flu,
    body_frd_to_flu,
)


def test_body_frd_to_flu() -> None:
    assert body_frd_to_flu((1.0, 2.0, 3.0)) == (1.0, -2.0, -3.0)


@pytest.mark.parametrize("value", [(1.0, 2.0), (1.0, 2.0, 3.0, 4.0)])
def test_body_conversion_rejects_wrong_length(value) -> None:
    with pytest.raises(ValueError, match="three-element"):
        body_frd_to_flu(value)


def test_north_facing_airsim_body_has_positive_enu_yaw() -> None:
    x, y, z, w = airsim_quat_to_ros_quat_flu((1.0, 0.0, 0.0, 0.0))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    assert yaw == pytest.approx(math.pi / 2.0)


def test_quaternion_is_normalized() -> None:
    quaternion = airsim_quat_to_ros_quat_flu((2.0, 0.0, 0.0, 0.0))
    norm = math.sqrt(sum(component * component for component in quaternion))
    assert norm == pytest.approx(1.0)


def test_zero_quaternion_is_rejected() -> None:
    with pytest.raises(ValueError, match="zero norm"):
        airsim_quat_to_ros_quat_flu((0.0, 0.0, 0.0, 0.0))
