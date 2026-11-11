"""Coordinate conversions used by the AirSim ROS 2 bridge.

AirSim uses a world NED frame and a body FRD frame. ROS REP-103 uses a
world ENU frame and a body FLU frame. Keeping these conversions in one
small pure-Python module makes them testable without running a simulator.
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

Vector3 = Tuple[float, float, float]
QuaternionXYZW = Tuple[float, float, float, float]


def _as_vector3(value: Sequence[float]) -> Vector3:
    if len(value) != 3:
        raise ValueError("expected a three-element vector")
    vector = (float(value[0]), float(value[1]), float(value[2]))
    if not all(math.isfinite(item) for item in vector):
        raise ValueError("vector contains a non-finite value")
    return vector


def ned_to_enu(value: Sequence[float]) -> Vector3:
    """Convert a world NED vector to world ENU."""
    north, east, down = _as_vector3(value)
    return (east, north, -down)


def enu_to_ned(value: Sequence[float]) -> Vector3:
    """Convert a world ENU vector to world NED."""
    east, north, up = _as_vector3(value)
    return (north, east, -up)


def body_ned_to_flu(value: Sequence[float]) -> Vector3:
    """Convert body FRD coordinates to ROS body FLU coordinates."""
    forward, right, down = _as_vector3(value)
    return (forward, -right, -down)


def body_flu_to_airsim(value: Sequence[float]) -> Vector3:
    """Convert a ROS body FLU command to AirSim body FRD."""
    forward, left, up = _as_vector3(value)
    return (forward, -left, -up)


def quat_wxyz_to_matrix(
    quaternion: Sequence[float],
) -> Tuple[Tuple[float, float, float], ...]:
    """Convert an AirSim (w, x, y, z) quaternion to a rotation matrix."""
    if len(quaternion) != 4:
        raise ValueError("expected a four-element quaternion")
    w, x, y, z = (float(item) for item in quaternion)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("quaternion is invalid or has zero norm")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def matrix_to_quat_xyzw(
    matrix: Sequence[Sequence[float]],
) -> QuaternionXYZW:
    """Convert a 3x3 rotation matrix to a normalized ROS quaternion."""
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("expected a 3x3 matrix")
    m = tuple(tuple(float(item) for item in row) for row in matrix)
    trace = m[0][0] + m[1][1] + m[2][2]

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m[2][1] - m[1][2]) / scale
        y = (m[0][2] - m[2][0]) / scale
        z = (m[1][0] - m[0][1]) / scale
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        scale = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / scale
        x = 0.25 * scale
        y = (m[0][1] + m[1][0]) / scale
        z = (m[0][2] + m[2][0]) / scale
    elif m[1][1] > m[2][2]:
        scale = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / scale
        x = (m[0][1] + m[1][0]) / scale
        y = 0.25 * scale
        z = (m[1][2] + m[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / scale
        x = (m[0][2] + m[2][0]) / scale
        y = (m[1][2] + m[2][1]) / scale
        z = 0.25 * scale

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / norm, y / norm, z / norm, w / norm)


def airsim_quat_to_ros_quat_flu(
    quaternion_wxyz: Sequence[float],
) -> QuaternionXYZW:
    """Convert AirSim body-FRD/world-NED attitude to ROS body-FLU/world-ENU."""
    r_ned = quat_wxyz_to_matrix(quaternion_wxyz)
    r_enu = (
        (r_ned[1][0], r_ned[1][1], r_ned[1][2]),
        (r_ned[0][0], r_ned[0][1], r_ned[0][2]),
        (-r_ned[2][0], -r_ned[2][1], -r_ned[2][2]),
    )
    r_flu = (
        (r_enu[0][0], -r_enu[0][1], -r_enu[0][2]),
        (r_enu[1][0], -r_enu[1][1], -r_enu[1][2]),
        (r_enu[2][0], -r_enu[2][1], -r_enu[2][2]),
    )
    return matrix_to_quat_xyzw(r_flu)


def quat_xyzw_to_matrix(
    quaternion: Sequence[float],
) -> Tuple[Tuple[float, float, float], ...]:
    """Convert a ROS (x, y, z, w) quaternion to a rotation matrix."""
    if len(quaternion) != 4:
        raise ValueError("expected a four-element quaternion")
    x, y, z, w = (float(item) for item in quaternion)
    return quat_wxyz_to_matrix((w, x, y, z))


def world_enu_to_body_flu(
    vector_world_enu: Sequence[float],
    orientation_xyzw: Sequence[float],
) -> Vector3:
    """Express a world ENU vector in the ROS body FLU frame."""
    east, north, up = _as_vector3(vector_world_enu)
    rotation = quat_xyzw_to_matrix(orientation_xyzw)
    return (
        rotation[0][0] * east
        + rotation[1][0] * north
        + rotation[2][0] * up,
        rotation[0][1] * east
        + rotation[1][1] * north
        + rotation[2][1] * up,
        rotation[0][2] * east
        + rotation[1][2] * north
        + rotation[2][2] * up,
    )


def yaw_from_quat_xyzw(quaternion: Sequence[float]) -> float:
    """Return the ENU yaw angle in radians."""
    x, y, z, w = (float(item) for item in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
