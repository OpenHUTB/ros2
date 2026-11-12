"""IMU message construction and source timestamp tracking."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu

from .transforms import airsim_quat_to_ros_quat_flu, body_frd_to_flu


def validate_parameters(
    *,
    port: int,
    publish_rate: float,
    reconnect_period: float,
    stale_timeout: float,
    orientation_variance: float,
    angular_velocity_variance: float,
    linear_acceleration_variance: float,
) -> None:
    """Validate numeric parameters before creating timers or messages."""
    if not 1 <= port <= 65535:
        raise ValueError("port must be in [1, 65535]")
    for name, value in (
        ("publish_rate", publish_rate),
        ("reconnect_period", reconnect_period),
        ("stale_timeout", stale_timeout),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    for name, value in (
        ("orientation_variance", orientation_variance),
        ("angular_velocity_variance", angular_velocity_variance),
        ("linear_acceleration_variance", linear_acceleration_variance),
    ):
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")


def airsim_stamp_to_time(timestamp_ns: int) -> Time:
    """Map an AirSim nanosecond timestamp to a ROS builtin_interfaces Time."""
    timestamp_ns = int(timestamp_ns)
    if timestamp_ns <= 0:
        raise ValueError("AirSim timestamp must be positive nanoseconds")
    return Time(
        sec=timestamp_ns // 1_000_000_000,
        nanosec=timestamp_ns % 1_000_000_000,
    )


def build_imu_message(
    sample: Any,
    *,
    frame_id: str,
    stamp: Time,
    orientation_variance: float,
    angular_velocity_variance: float,
    linear_acceleration_variance: float,
) -> Imu:
    """Convert one AirSim ImuData sample to a sensor_msgs/Imu message."""
    orientation = airsim_quat_to_ros_quat_flu(
        (
            sample.orientation.w_val,
            sample.orientation.x_val,
            sample.orientation.y_val,
            sample.orientation.z_val,
        )
    )
    angular_velocity = body_frd_to_flu(
        (
            sample.angular_velocity.x_val,
            sample.angular_velocity.y_val,
            sample.angular_velocity.z_val,
        )
    )
    linear_acceleration = body_frd_to_flu(
        (
            sample.linear_acceleration.x_val,
            sample.linear_acceleration.y_val,
            sample.linear_acceleration.z_val,
        )
    )

    message = Imu()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.orientation.x = orientation[0]
    message.orientation.y = orientation[1]
    message.orientation.z = orientation[2]
    message.orientation.w = orientation[3]
    message.angular_velocity.x = angular_velocity[0]
    message.angular_velocity.y = angular_velocity[1]
    message.angular_velocity.z = angular_velocity[2]
    message.linear_acceleration.x = linear_acceleration[0]
    message.linear_acceleration.y = linear_acceleration[1]
    message.linear_acceleration.z = linear_acceleration[2]

    for index in (0, 4, 8):
        message.orientation_covariance[index] = orientation_variance
        message.angular_velocity_covariance[index] = (
            angular_velocity_variance
        )
        message.linear_acceleration_covariance[index] = (
            linear_acceleration_variance
        )
    return message


@dataclass
class SourceTimestampTracker:
    """Track whether AirSim returned a new source sample."""

    last_source_stamp: Optional[int] = None
    last_new_sample_time: Optional[float] = None

    def observe(self, source_stamp: int, now: float) -> bool:
        """Return true only for a new timestamp and record its arrival time."""
        source_stamp = int(source_stamp)
        if source_stamp <= 0:
            self.last_new_sample_time = now
            return True
        if self.last_source_stamp is not None:
            if source_stamp == self.last_source_stamp:
                return False
            if source_stamp < self.last_source_stamp:
                self.last_source_stamp = source_stamp
                self.last_new_sample_time = now
                return True
        self.last_source_stamp = source_stamp
        self.last_new_sample_time = now
        return True

    def age(self, now: float) -> float:
        """Return seconds since the most recent accepted source sample."""
        if self.last_new_sample_time is None:
            return float("inf")
        return max(0.0, now - self.last_new_sample_time)

    def reset(self) -> None:
        """Forget the previous stream after reconnecting."""
        self.last_source_stamp = None
        self.last_new_sample_time = None
