"""IMU 坐标系约定与适配层的回归测试。

对应评审意见：``on_imu`` 曾把同一个 accel 同时喂给姿态校正和 EKF，而两者对
静止时重力符号的约定相反。这里覆盖 REP-103(FLU) -> FRD 的转换，以及水平静止
输入贯穿 ``step_imu``（即 ``on_imu`` 的处理链）后不应产生加速度。
"""

import math
import unittest

import numpy as np

from uav_ekf_state_estimation.attitude import (
    GRAVITY_NED,
    ComplementaryAttitude,
    euler_to_rotation,
)
from uav_ekf_state_estimation.ekf import NavigationEKF
from uav_ekf_state_estimation.frames import (
    FLU_TO_FRD,
    imu_to_ekf,
    specific_force_flu_to_frd,
    step_imu,
)

G = 9.80665


class FramesTest(unittest.TestCase):
    def test_flu_to_frd_flips_y_and_z(self):
        self.assertTrue(np.allclose(np.diag(FLU_TO_FRD), [1.0, -1.0, -1.0]))
        force = specific_force_flu_to_frd(np.array([0.0, 0.0, G]))
        self.assertTrue(np.allclose(force, [0.0, 0.0, -G]))

    def test_level_static_input_cancels_gravity(self):
        force, rotation = imu_to_ekf(np.array([0.0, 0.0, G]), 0.0, 0.0, 0.0)
        self.assertTrue(np.allclose(rotation, np.eye(3)))
        self.assertTrue(np.allclose(rotation @ force + GRAVITY_NED, np.zeros(3)))

    def test_roll_keeps_its_sign(self):
        roll = math.radians(12.0)
        accel = np.array([0.0, G * math.sin(roll), G * math.cos(roll)])
        force, rotation = imu_to_ekf(accel, roll, 0.0, 0.0)
        self.assertTrue(np.allclose(rotation, euler_to_rotation(roll, 0.0, 0.0)))
        self.assertTrue(
            np.allclose(rotation @ force + GRAVITY_NED, np.zeros(3), atol=1e-9)
        )

    def test_pitch_sign_is_converted(self):
        pitch = math.radians(9.0)
        # 静止时的 FLU 比力为 (g sinθ, 0, g cosθ)，姿态滤波给出 -θ。
        accel = np.array([G * math.sin(pitch), 0.0, G * math.cos(pitch)])
        force, rotation = imu_to_ekf(accel, 0.0, -pitch, 0.0)
        self.assertTrue(np.allclose(rotation, euler_to_rotation(0.0, pitch, 0.0)))
        self.assertTrue(
            np.allclose(rotation @ force + GRAVITY_NED, np.zeros(3), atol=1e-9)
        )

    def test_stationary_imu_through_callback_chain_stays_zero(self):
        attitude = ComplementaryAttitude()
        ekf = NavigationEKF(np.zeros(3))
        for _ in range(200):
            step_imu(
                attitude,
                ekf,
                np.array([0.0, 0.0, G]),
                np.zeros(3),
                np.array([1.0, 0.0, 0.0]),
                0.01,
            )
        self.assertLess(float(np.linalg.norm(ekf.velocity)), 1e-6)
        self.assertLess(float(np.linalg.norm(ekf.position)), 1e-9)
        self.assertAlmostEqual(attitude.roll, 0.0, places=6)
        self.assertAlmostEqual(attitude.pitch, 0.0, places=6)

    def test_pipeline_runs_before_first_gps_fix(self):
        attitude = ComplementaryAttitude()
        roll, pitch, _ = step_imu(
            attitude, None, np.array([0.0, 0.0, G]), np.zeros(3), np.ones(3), 0.01
        )
        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
