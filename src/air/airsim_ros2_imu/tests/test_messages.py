"""Tests for IMU message construction and source timestamp tracking."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from airsim_ros2_imu.imu import (
    SourceTimestampTracker,
    airsim_stamp_to_time,
    build_imu_message,
    validate_parameters,
)


def _sample(timestamp: int = 1_500_000_001):
    return SimpleNamespace(
        time_stamp=timestamp,
        orientation=SimpleNamespace(
            w_val=1.0,
            x_val=0.0,
            y_val=0.0,
            z_val=0.0,
        ),
        angular_velocity=SimpleNamespace(
            x_val=1.0,
            y_val=2.0,
            z_val=3.0,
        ),
        linear_acceleration=SimpleNamespace(
            x_val=4.0,
            y_val=5.0,
            z_val=6.0,
        ),
    )


def test_build_imu_message_fields() -> None:
    stamp = airsim_stamp_to_time(1_500_000_001)
    message = build_imu_message(
        _sample(),
        frame_id="imu_link",
        stamp=stamp,
        orientation_variance=0.1,
        angular_velocity_variance=0.2,
        linear_acceleration_variance=0.3,
    )
    assert message.header.frame_id == "imu_link"
    assert message.header.stamp.sec == 1
    assert message.header.stamp.nanosec == 500_000_001
    assert message.orientation.x == pytest.approx(0.0)
    assert message.orientation.y == pytest.approx(0.0)
    assert message.orientation.z == pytest.approx(math.sqrt(0.5))
    assert message.orientation.w == pytest.approx(math.sqrt(0.5))
    assert message.angular_velocity.x == pytest.approx(1.0)
    assert message.angular_velocity.y == pytest.approx(-2.0)
    assert message.angular_velocity.z == pytest.approx(-3.0)
    assert message.linear_acceleration.x == pytest.approx(4.0)
    assert message.linear_acceleration.y == pytest.approx(-5.0)
    assert message.linear_acceleration.z == pytest.approx(-6.0)
    for index in (0, 4, 8):
        assert message.orientation_covariance[index] == pytest.approx(0.1)
        assert message.angular_velocity_covariance[index] == pytest.approx(0.2)
        assert message.linear_acceleration_covariance[index] == pytest.approx(
            0.3
        )


def test_source_timestamp_tracker_rejects_duplicate() -> None:
    tracker = SourceTimestampTracker()
    assert tracker.observe(100, 1.0)
    assert not tracker.observe(100, 1.1)
    assert tracker.age(1.5) == pytest.approx(0.5)
    assert tracker.observe(101, 1.6)
    assert tracker.age(1.6) == pytest.approx(0.0)


def test_source_timestamp_tracker_accepts_source_reset() -> None:
    tracker = SourceTimestampTracker()
    assert tracker.observe(100, 1.0)
    assert tracker.observe(10, 2.0)


@pytest.mark.parametrize(
    "overrides",
    [
        {"port": 0},
        {"port": 65536},
        {"publish_rate": float("nan")},
        {"reconnect_period": float("inf")},
        {"stale_timeout": 0.0},
        {"orientation_variance": -1.0},
        {"angular_velocity_variance": float("nan")},
        {"linear_acceleration_variance": float("-inf")},
    ],
)
def test_invalid_parameters_are_rejected(overrides) -> None:
    parameters = {
        "port": 41451,
        "publish_rate": 50.0,
        "reconnect_period": 2.0,
        "stale_timeout": 0.25,
        "orientation_variance": 0.001,
        "angular_velocity_variance": 0.0001,
        "linear_acceleration_variance": 0.01,
    }
    parameters.update(overrides)
    with pytest.raises(ValueError):
        validate_parameters(**parameters)
