"""Small attitude helpers used by the EKF demo."""

from __future__ import annotations

import math

import numpy as np


GRAVITY_NED = np.array([0.0, 0.0, 9.80665])


def euler_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Body-to-NED rotation matrix using Z-Y-X Euler angles."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


class ComplementaryAttitude:
    """Gyro integration with accelerometer and magnetometer correction."""

    def __init__(self, alpha: float = 0.98) -> None:
        self.alpha = float(alpha)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update(
        self,
        gyro: np.ndarray,
        accel: np.ndarray,
        mag: np.ndarray,
        dt: float,
    ) -> tuple[float, float, float]:
        p, q, r = (float(value) for value in gyro)
        self.roll += p * dt
        self.pitch += q * dt
        self.yaw += r * dt

        ax, ay, az = (float(value) for value in accel)
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm > 1e-6:
            roll_acc = math.atan2(ay, az)
            pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))
            self.roll = self.alpha * self.roll + (1.0 - self.alpha) * roll_acc
            self.pitch = self.alpha * self.pitch + (1.0 - self.alpha) * pitch_acc

        mx, my, _ = (float(value) for value in mag)
        if math.hypot(mx, my) > 1e-6:
            # For R = Rz(yaw)Ry(pitch)Rx(roll), R^T * [1, 0, 0]
            # has a y component of -sin(yaw).
            yaw_mag = math.atan2(-my, mx)
            error = _wrap_pi(yaw_mag - self.yaw)
            self.yaw += (1.0 - self.alpha) * error

        self.roll = _wrap_pi(self.roll)
        self.pitch = _wrap_pi(self.pitch)
        self.yaw = _wrap_pi(self.yaw)
        return self.roll, self.pitch, self.yaw

    def rotation(self) -> np.ndarray:
        return euler_to_rotation(self.roll, self.pitch, self.yaw)


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
