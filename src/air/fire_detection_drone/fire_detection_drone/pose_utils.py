"""Small coordinate and quaternion helpers shared by ROS-facing code."""

from __future__ import annotations

import math


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the interval (-pi, pi]."""

    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract planar yaw from a quaternion."""

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def ned_pose_to_enu(
    x: float, y: float, z: float, yaw: float
) -> tuple[float, float, float, float]:
    """Convert a planar NED pose (AirSim) to ROS REP-103 ENU."""

    return x, -y, -z, -yaw


def enu_twist_to_ned(
    linear_x: float, linear_y: float, linear_z: float, angular_z: float
) -> tuple[float, float, float, float]:
    """Convert a body-frame ENU Twist to AirSim's body-frame NED Twist."""

    return linear_x, -linear_y, -linear_z, -angular_z
