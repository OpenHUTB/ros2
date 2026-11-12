"""四元数基础运算的单元测试（不依赖 ROS）。"""

import math

import numpy as np

from humanoid_imu_attitude.quat_math import (
    accel_tilt_quat,
    euler_from_quat,
    gravity_in_body,
    integrate_gyro,
    quat_from_axis_angle,
    quat_from_euler,
    quat_mul,
    quat_normalize,
    rotate_vector,
    wrap_angle,
)


def test_euler_roundtrip():
    for angles in [(0.0, 0.0, 0.0), (0.3, -0.2, 0.9), (-1.2, 0.4, -2.5)]:
        q = quat_from_euler(*angles)
        assert np.allclose(euler_from_quat(q), angles, atol=1e-9)


def test_normalize_is_unit():
    q = quat_normalize([2.0, -1.0, 0.5, 3.0])
    assert abs(np.linalg.norm(q) - 1.0) < 1e-12


def test_zero_quaternion_falls_back_to_identity():
    assert np.allclose(quat_normalize([0.0, 0.0, 0.0, 0.0]), [1.0, 0.0, 0.0, 0.0])


def test_rotate_vector_yaw_90_degrees():
    q = quat_from_euler(0.0, 0.0, math.pi / 2.0)
    assert np.allclose(rotate_vector(q, [1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)


def test_gravity_in_body_matches_tilt():
    # 绕 y 轴俯仰 +30° 时，机体系重力应约为 [-0.5, 0, 0.866]
    q = quat_from_euler(0.0, math.radians(30.0), 0.0)
    expected = np.array([-math.sin(math.radians(30.0)), 0.0, math.cos(math.radians(30.0))])
    assert np.allclose(gravity_in_body(q), expected, atol=1e-9)


def test_constant_yaw_rate_integration():
    q = np.array([1.0, 0.0, 0.0, 0.0])
    rate = math.pi / 2.0  # 90°/s，积分 1 s
    for _ in range(200):
        q = integrate_gyro(q, [0.0, 0.0, rate], 0.005)
    assert abs(math.degrees(euler_from_quat(q)[2]) - 90.0) < 0.05


def test_quat_mul_matches_composition():
    qa = quat_from_euler(0.2, 0.1, 0.0)
    qb = quat_from_euler(0.0, -0.3, 0.4)
    v = np.array([0.3, -0.7, 0.5])
    assert np.allclose(rotate_vector(quat_mul(qa, qb), v),
                       rotate_vector(qa, rotate_vector(qb, v)), atol=1e-9)


def test_accel_tilt_quat():
    q_true = quat_from_euler(0.25, -0.15, 0.0)
    q_est = accel_tilt_quat(gravity_in_body(q_true))
    assert np.allclose(euler_from_quat(q_est)[:2], (0.25, -0.15), atol=1e-9)


def test_wrap_angle():
    assert abs(wrap_angle(math.radians(190.0)) - math.radians(-170.0)) < 1e-12
    assert abs(wrap_angle(math.radians(-190.0)) - math.radians(170.0)) < 1e-12


def test_axis_angle_identity_for_zero_angle():
    assert np.allclose(quat_from_axis_angle([0.0, 1.0, 0.0], 0.0), [1.0, 0.0, 0.0, 0.0])
