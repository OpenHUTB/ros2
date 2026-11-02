"""IMU and position frame conventions and the conversions used by the adapter.

``sensor_msgs/Imu`` follows REP-103: the body frame is FLU (x forward, y left,
z up) and ``linear_acceleration`` includes gravity, so a level stationary IMU
reports ``(0, 0, +g)``.

``ComplementaryAttitude`` is written in that FLU frame: its tilt estimate is
``roll = atan2(ay, az)``, which is only correct when ``+g`` sits on z.

``NavigationEKF`` is written in the aerospace FRD frame (x forward, y right,
z down): it expects the specific force at rest to be ``(0, 0, -g)`` and a
body-FRD to NED rotation, so that ``rotation @ force + GRAVITY_NED`` cancels.

The same filter keeps its position and velocity in NED, so every position
measurement must arrive as ``[north, east]`` and every published pose must be
converted back to ENU. Both directions live here as well, for the same reason:
this module is the only place where the external conventions meet the filter's.

The adapter below is the only place where the two conventions meet: the
attitude filter keeps the raw REP-103 sample, and the EKF receives the
converted FRD specific force and FRD rotation.
"""

from __future__ import annotations

import numpy as np

from .attitude import euler_to_rotation

# FLU (x forward, y left, z up) -> FRD (x forward, y right, z down).
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])

# Mean Earth radius used by the local flat-earth GPS projection.
EARTH_RADIUS_M = 6371000.0


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


def geodetic_to_ned(
    latitude: float,
    longitude: float,
    origin_latitude: float,
    origin_longitude: float,
) -> np.ndarray:
    """Project a GPS fix onto the local tangent plane as ``[north, east]``.

    The order matches the EKF state (``x[0] = north``, ``x[1] = east``). Handing
    ``[east, north]`` to :meth:`NavigationEKF.update_position` moves a purely
    eastward displacement into the internal north state, and no later ENU
    conversion can undo that mix-up.
    """
    north = np.radians(latitude - origin_latitude) * EARTH_RADIUS_M
    east = (
        np.radians(longitude - origin_longitude)
        * EARTH_RADIUS_M
        * np.cos(np.radians(origin_latitude))
    )
    return np.array([float(north), float(east)])


def ned_to_enu(ned: np.ndarray) -> np.ndarray:
    """Convert an NED vector to ENU (``[east, north, -down]``) for publication."""
    north, east, down = (float(value) for value in np.asarray(ned, dtype=float))
    return np.array([east, north, -down])


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
