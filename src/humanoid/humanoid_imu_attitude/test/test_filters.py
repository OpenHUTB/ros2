"""两种滤波器的收敛性与轨迹一致性测试（不依赖 ROS）。"""

import math

import numpy as np

from humanoid_imu_attitude.quat_math import (
    complementary_step,
    euler_from_quat,
    gravity_in_body,
    integrate_gyro,
    madgwick_step,
    quat_from_euler,
)
from humanoid_imu_attitude.trajectory import attitude_at, body_rates_at


def _tilt_error_deg(q, q_true):
    roll, pitch, _ = euler_from_quat(q)
    roll_true, pitch_true, _ = euler_from_quat(q_true)
    return abs(math.degrees(roll - roll_true)), abs(math.degrees(pitch - pitch_true))


def test_complementary_converges_on_static_tilt():
    q_true = quat_from_euler(0.25, -0.15, 0.0)
    acc = gravity_in_body(q_true)
    q = np.array([1.0, 0.0, 0.0, 0.0])
    for _ in range(2000):  # 10 s @ 200 Hz
        q = complementary_step(q, [0.0, 0.0, 0.0], acc, 0.005, 0.98)
    roll_err, pitch_err = _tilt_error_deg(q, q_true)
    assert roll_err < 0.5
    assert pitch_err < 0.5


def test_madgwick_converges_on_static_tilt():
    q_true = quat_from_euler(0.25, -0.15, 0.0)
    acc = gravity_in_body(q_true)
    q = np.array([1.0, 0.0, 0.0, 0.0])
    for _ in range(4000):  # 20 s @ 200 Hz
        q = madgwick_step(q, [0.0, 0.0, 0.0], acc, 0.005, 0.05)
    roll_err, pitch_err = _tilt_error_deg(q, q_true)
    assert roll_err < 1.0
    assert pitch_err < 1.0


def test_trajectory_rates_match_attitude():
    """用解析角速度积分 1 s，应与轨迹真值一致。"""
    profile = 'fast'
    dt = 0.001
    q = quat_from_euler(*attitude_at(0.0, profile))
    for i in range(1000):
        t = i * dt
        q = integrate_gyro(q, body_rates_at(t, profile), dt)
    q_true = quat_from_euler(*attitude_at(1.0, profile))
    roll_err, pitch_err = _tilt_error_deg(q, q_true)
    assert roll_err < 0.2
    assert pitch_err < 0.2


def test_complementary_tracks_dynamic_with_small_lag():
    """连续动态输入下，互补滤波误差应有界（alpha=0.98 时滞后可见但不发散）。"""
    profile = 'fast'
    q_true = quat_from_euler(*attitude_at(0.0, profile))
    q = q_true.copy()
    worst = 0.0
    for i in range(2000):
        t = i * 0.005
        q_true = quat_from_euler(*attitude_at(t, profile))
        acc = gravity_in_body(q_true)
        q = complementary_step(q, body_rates_at(t, profile), acc, 0.005, 0.98)
        worst = max(worst, max(_tilt_error_deg(q, q_true)))
    assert worst < 30.0
