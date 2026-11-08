# -*- coding: utf-8 -*-
"""侧倾特征构造：把 IMU 读数与当前指令拼成网络输入。

9 维输入（按物理量程归一化，尺度在训练前固定，不依赖数据统计）：
    [横滚角, 俯仰角, 横滚角速度, 俯仰角速度, 偏航角速度, 纵向加速度, 侧向加速度, 车速, 转向指令]
前 8 维全部来自机体系 IMU，第 9 维是当前转向指令（上游规划已知量）。
ROS 节点直接用 from_imu 组装，仿真环境用 build 组装，两者共用同一套归一化。
"""
import math

import numpy as np


def roll_pitch(quat):
    """由机体系四元数 (w, x, y, z) 解算横滚角与俯仰角（rad）。"""
    qw, qx, qy, qz = quat
    roll = math.atan2(2.0 * (qw * qx + qy * qz),
                      1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
    return roll, pitch


def from_imu(quat, gyro, acc, speed, steer):
    """把一帧 IMU 读数拼成 9 维物理量特征（未归一化）。"""
    roll, pitch = roll_pitch(quat)
    return np.array([
        roll, pitch,
        gyro[0], gyro[1], gyro[2],
        acc[0], acc[1],
        float(speed), float(steer),
    ], dtype=np.float64)


def normalize(raw, cfg):
    raw = np.asarray(raw, dtype=np.float64).reshape(-1)
    if raw.size != cfg.feature_dim:
        raise ValueError("特征维度应为 {}，实际 {}".format(cfg.feature_dim, raw.size))
    scale = np.asarray(cfg.scales, dtype=np.float64)
    return np.clip(raw / scale, -3.0, 3.0)


def build(env, steer, cfg):
    """从仿真环境读取 IMU 并返回 (raw, normalized)；raw 保留物理量便于日志核对。"""
    imu = env.imu()
    raw = from_imu(imu["quat"], imu["gyro"], imu["acc"],
                   env.forward_speed(), steer)
    return raw, normalize(raw, cfg)
