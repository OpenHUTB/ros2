"""四元数运算与两种姿态滤波（纯 NumPy，不依赖 ROS，便于单测）。

约定：四元数按 [w, x, y, z] 排列；欧拉角按 ZYX（先 roll，再 pitch，最后 yaw）。
加速度计读数使用重力单位向量（静止时指向 +z，模长为 1），由调用方归一化。
"""

import math

import numpy as np


def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_from_axis_angle(axis, angle):
    """轴角转四元数；角度为弧度。"""
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12 or abs(angle) < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / norm
    half = 0.5 * angle
    s = math.sin(half)
    return np.array([math.cos(half), axis[0] * s, axis[1] * s, axis[2] * s])


def quat_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(0.5 * roll), math.sin(0.5 * roll)
    cp, sp = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
    cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def euler_from_quat(q):
    q = quat_normalize(q)
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def rotate_vector(q, v):
    """把机体系向量旋转到世界系：R(q) v。"""
    qv = np.array([0.0, v[0], v[1], v[2]])
    return quat_mul(quat_mul(q, qv), quat_conj(q))[1:]


def gravity_in_body(q):
    """机体坐标系下的重力单位向量，即加速度计静止时的读数方向。"""
    return rotate_vector(quat_conj(q), np.array([0.0, 0.0, 1.0]))


def integrate_gyro(q, gyro, dt):
    """用机体角速度积分四元数：q ⊗ exp(ω·dt/2)。"""
    omega = np.asarray(gyro, dtype=float)
    rate = np.linalg.norm(omega)
    if rate < 1e-9:
        return quat_normalize(q)
    return quat_normalize(quat_mul(q, quat_from_axis_angle(omega / rate, rate * dt)))


def accel_tilt_quat(acc):
    """只用加速度计估计 roll/pitch，yaw 置 0。"""
    a = np.asarray(acc, dtype=float)
    norm = np.linalg.norm(a)
    if norm < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    ax, ay, az = a / norm
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return quat_from_euler(roll, pitch, 0.0)


def complementary_step(q, gyro, acc, dt, alpha):
    """互补滤波一步：陀螺仪积分后，用加速度计按 (1-alpha) 的比例校正 roll/pitch。

    alpha 越大越信任陀螺仪，加速度计的截止频率越低；yaw 没有观测量，只跟随陀螺仪。
    """
    q_gyro = integrate_gyro(q, gyro, dt)
    a = np.asarray(acc, dtype=float)
    norm = np.linalg.norm(a)
    if norm < 1e-6:
        return q_gyro
    a = a / norm
    g_body = gravity_in_body(q_gyro)
    axis = np.cross(a, g_body)
    sin_angle = np.linalg.norm(axis)
    if sin_angle < 1e-9:
        return q_gyro
    angle = math.atan2(sin_angle, float(np.clip(np.dot(a, g_body), -1.0, 1.0)))
    correction = quat_from_axis_angle(axis / sin_angle, (1.0 - alpha) * angle)
    return quat_normalize(quat_mul(q_gyro, correction))


def madgwick_step(q, gyro, acc, dt, beta):
    """Madgwick IMU 版（无磁力计）一步更新，按原始论文的梯度下降形式实现。"""
    q = quat_normalize(q)
    a = np.asarray(acc, dtype=float)
    norm = np.linalg.norm(a)
    if norm < 1e-6:
        return integrate_gyro(q, gyro, dt)
    ax, ay, az = a / norm
    w, x, y, z = q
    gx, gy, gz = gyro

    # 目标函数：估计重力方向与实测重力方向之差
    f = np.array([
        2.0 * (x * z - w * y) - ax,
        2.0 * (w * x + y * z) - ay,
        2.0 * (0.5 - x * x - y * y) - az,
    ])
    # 目标函数对四元数的雅可比
    jacobian = np.array([
        [-2.0 * y, 2.0 * z, -2.0 * w, 2.0 * x],
        [2.0 * x, 2.0 * w, 2.0 * z, 2.0 * y],
        [0.0, -4.0 * x, -4.0 * y, 0.0],
    ])
    gradient = jacobian.T @ f
    grad_norm = np.linalg.norm(gradient)
    if grad_norm > 1e-12:
        gradient = gradient / grad_norm

    q_dot = 0.5 * quat_mul(q, np.array([0.0, gx, gy, gz])) - beta * gradient
    return quat_normalize(q + q_dot * dt)


def wrap_angle(angle):
    """把角度差折算到 [-pi, pi]。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
