"""姿态轨迹定义、角速度解析式和带噪声的 IMU 合成。

三种预设对应不同的动态强度，用来观察滤波器在“静止 / 慢速 / 快速”下的差异。
"""

import math

import numpy as np

# amplitude: 幅度（弧度）；freq: 频率（Hz）；yaw_ratio: yaw 幅度相对 roll 的比例
PROFILES = {
    'static': {'amplitude': 0.0, 'freq': 0.0, 'yaw_ratio': 0.0},
    'slow': {'amplitude': math.radians(8.0), 'freq': 0.25, 'yaw_ratio': 0.5},
    'fast': {'amplitude': math.radians(20.0), 'freq': 1.2, 'yaw_ratio': 0.4},
}


def attitude_at(t, profile):
    """返回 t 时刻的真实姿态 (roll, pitch, yaw)，单位弧度。"""
    cfg = PROFILES[profile]
    amplitude = cfg['amplitude']
    freq = cfg['freq']
    if amplitude <= 0.0 or freq <= 0.0:
        return 0.0, 0.0, 0.0
    omega = 2.0 * math.pi * freq
    roll = amplitude * math.sin(omega * t)
    pitch = 0.6 * amplitude * math.sin(omega * t + 0.8)
    yaw = cfg['yaw_ratio'] * amplitude * math.sin(0.5 * omega * t)
    return roll, pitch, yaw


def body_rates_at(t, profile):
    """真实机体角速度 (p, q, r)，由欧拉角解析求导后经 ZYX 变换得到。"""
    cfg = PROFILES[profile]
    amplitude = cfg['amplitude']
    freq = cfg['freq']
    if amplitude <= 0.0 or freq <= 0.0:
        return np.zeros(3)
    omega = 2.0 * math.pi * freq
    roll_dot = amplitude * omega * math.cos(omega * t)
    pitch_dot = 0.6 * amplitude * omega * math.cos(omega * t + 0.8)
    yaw_dot = cfg['yaw_ratio'] * amplitude * 0.5 * omega * math.cos(0.5 * omega * t)

    roll, pitch, _ = attitude_at(t, profile)
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    # ZYX 欧拉角速度到机体角速度的变换
    p = roll_dot - yaw_dot * sp
    q = pitch_dot * cr + yaw_dot * sr * cp
    r = -pitch_dot * sr + yaw_dot * cr * cp
    return np.array([p, q, r])
