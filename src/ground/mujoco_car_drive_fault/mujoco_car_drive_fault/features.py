# -*- coding: utf-8 -*-
"""失效特征构造：把 IMU、轮速与当前指令拼成网络输入。

9 维输入（按物理量程归一化，尺度在训练前固定，不依赖数据统计）：
    [横向偏差, 航向角, 偏航角速度, 侧向加速度, 车速, 左轮实际转速, 右轮实际转速,
     左轮指令, 右轮指令]
第 1~5 维来自姿态/位置观测，第 6~7 维是轮速传感器实测，最后两维是下发给
驱动器的目标转速（已下发指令，控制器自己知道）。ROS 节点与仿真环境共用同一套
组特征与归一化。
"""
import numpy as np


def from_state(y, yaw, gyro, acc, speed, w_left, w_right, cmd_left, cmd_right):
    """把一帧观测拼成 9 维物理量特征（未归一化）。"""
    return np.array([
        float(y), float(yaw), float(gyro[2]), float(acc[1]), float(speed),
        float(w_left), float(w_right), float(cmd_left), float(cmd_right),
    ], dtype=np.float64)


def normalize(raw, cfg):
    raw = np.asarray(raw, dtype=np.float64).reshape(-1)
    if raw.size != cfg.feature_dim:
        raise ValueError("特征维度应为 {}，实际 {}".format(cfg.feature_dim, raw.size))
    scale = np.asarray(cfg.scales, dtype=np.float64)
    return np.clip(raw / scale, -3.0, 3.0)


def build(env, cmd_l, cmd_r, cfg):
    """从仿真环境读取状态并返回 (raw, normalized)。"""
    imu = env.imu()
    wl, wr = env.wheel_speeds()
    raw = from_state(env.lateral_offset(), env.yaw(), imu["gyro"], imu["acc"],
                     env.forward_speed(), wl, wr, cmd_l, cmd_r)
    return raw, normalize(raw, cfg)
