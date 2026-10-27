"""IMU frame conventions and the single conversion used by the ROS adapter.

``sensor_msgs/Imu`` follows REP-103: the body frame is FLU (x forward, y left,
z up) and ``linear_acceleration`` includes gravity, so a level stationary IMU
reports ``(0, 0, +g)``.

``ComplementaryAttitude`` is written in that FLU frame: its tilt estimate is
``roll = atan2(ay, az)``, which is only correct when ``+g`` sits on z.

``NavigationEKF`` is written in the aerospace FRD frame (x forward, y right,
z down): it expects the specific force at rest to be ``(0, 0, -g)`` and a
body-FRD to NED rotation, so that ``rotation @ force + GRAVITY_NED`` cancels.

The adapter below is the only place where the two conventions meet: the
attitude filter keeps the raw REP-103 sample, and the EKF receives the
converted FRD specific force and FRD rotation.
"""

from __future__ import annotations

import numpy as np

from .attitude import euler_to_rotation

# FLU (x forward, y left, z up) -> FRD (x forward, y right, z down).
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def specific_force_flu_to_frd(accel_flu: np.ndarray) -> np.ndarray:
    """Convert a REP-103 (FLU, ``+g`` at rest) specific force to FRD (``-g``)."""
    return FLU_TO_FRD @ np.asarray(accel_flu, dtype=float)


def tilt_flu_to_frd(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float]:
    """Convert FLU tilt angles to the FRD angles used by the EKF rotation.

    Flipping y and z leaves the roll axis unchanged but reverses the pitch axis,
    so the pitch sign has to be inverted. A level attitude stays level, which
    keeps the rotation at identity for the stationary case.
    """
    return float(roll), float(-pitch), float(yaw)


def imu_to_ekf(
    accel_flu: np.ndarray, roll: float, pitch: float, yaw: float
) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(specific_force_frd, rotation_frd_to_ned)`` for the EKF."""
    force_frd = specific_force_flu_to_frd(accel_flu)
    roll_frd, pitch_frd, yaw_frd = tilt_flu_to_frd(roll, pitch, yaw)
    return force_frd, euler_to_rotation(roll_frd, pitch_frd, yaw_frd)


def step_imu(attitude, ekf, accel_flu, gyro_flu, mag_flu, dt):
    """Run one IMU sample through the same chain as the ROS callback.

    ``attitude`` consumes the raw REP-103 sample, ``ekf`` consumes the converted
    FRD quantities. ``ekf`` may be ``None`` before the first GPS fix.
    """
    roll, pitch, yaw = attitude.update(gyro_flu, accel_flu, mag_flu, dt)
    if ekf is not None:
        force_frd, rotation = imu_to_ekf(accel_flu, roll, pitch, yaw)
        ekf.predict(force_frd, rotation, dt)
    return roll, pitch, yaw
