#!/usr/bin/env python3
"""Frame conversions shared by the simulator bridges.

The controller and the ROS nodes use the ROS REP-103 convention: the world
frame is ENU (x east, y north, z up) and the body frame is FLU (x forward,
y left, z up).  AirSim publishes and accepts NED / FRD quantities, so the
bridge has to convert positions, velocities and attitudes in both directions.

The conversion is a fixed 180 deg rotation about the axis ``(1, 1, 0) / sqrt(2)``
for the world frame, combined with ``diag(1, -1, -1)`` for the body frame.
Keeping it in one module makes the bridge and its unit tests share exactly the
same implementation.
"""

from __future__ import annotations

import math

import numpy as np

# NED -> ENU, applied to world-frame vectors.
NED_TO_ENU = np.array(
    [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
)

# FLU -> FRD, applied to body-frame vectors.
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def ned_to_enu(vector) -> np.ndarray:
    """Convert a world-frame NED vector (position or velocity) to ENU."""
    return NED_TO_ENU @ np.asarray(vector, dtype=float)


def enu_to_ned(vector) -> np.ndarray:
    """Convert a world-frame ENU vector (position or velocity) to NED."""
    return NED_TO_ENU.T @ np.asarray(vector, dtype=float)


def quat_wxyz_to_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Rotation matrix for a unit quaternion in ``(w, x, y, z)`` order."""
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
    )


def matrix_to_rpy(rotation: np.ndarray):
    """Roll, pitch, yaw of a rotation matrix in the standard z-y-x convention."""
    r = np.asarray(rotation, dtype=float)
    sin_pitch = -r[2, 0]
    if abs(sin_pitch) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sin_pitch)
        roll = math.atan2(r[0, 1], r[1, 1])
        yaw = 0.0
        return roll, pitch, yaw
    roll = math.atan2(r[2, 1], r[2, 2])
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(r[1, 0], r[0, 0])
    return roll, pitch, yaw


def airsim_orientation_to_rpy(w: float, x: float, y: float, z: float):
    """AirSim FRD-in-NED quaternion -> FLU-in-ENU roll, pitch, yaw."""
    r_ned_frd = quat_wxyz_to_matrix(w, x, y, z)
    r_enu_flu = NED_TO_ENU @ r_ned_frd @ FLU_TO_FRD
    return matrix_to_rpy(r_enu_flu)


def rpy_to_quat_xyzw(roll: float, pitch: float, yaw: float):
    """Standard z-y-x Euler angles -> quaternion ``(x, y, z, w)``."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_wxyz_to_rpy(w: float, x: float, y: float, z: float):
    """Quaternion in ``(w, x, y, z)`` order -> roll, pitch, yaw."""
    return matrix_to_rpy(quat_wxyz_to_matrix(w, x, y, z))


def wrap_pi(angle: float) -> float:
    """Wrap an angle to ``[-pi, pi)``."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi
