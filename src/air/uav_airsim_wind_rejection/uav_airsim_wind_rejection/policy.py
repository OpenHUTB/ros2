"""抗风专家策略：PD 位置保持 + 风速前馈。

专家能看到真实风速（用于前馈），网络只能看到 IMU 与位置误差，
所以网络必须自己从 IMU 的倾斜与抖动里「猜」出风有多大、往哪吹。
"""
from __future__ import annotations

import numpy as np

KP = 0.55          # 位置误差 -> 补偿速度
KD = 0.20          # 速度阻尼
# 实测标定：SimpleFlight 内置速度环会抵消大部分风，
# 20 m/s 风仅产生约 1.6 m/s 等效漂移，故前馈系数取 0.08 而非 1.0。
WIND_GAIN = 0.08
CMD_MAX = 3.0      # 补偿速度上限 m/s


def expert_action(err_xy, vel_xy, wind_xy):
    """返回世界系补偿速度 (dvx, dvy)，单位 m/s。

    wind_xy 是空气流动方向（世界系），无人机被推往该方向，
    因此前馈项取负号：朝逆风方向出力才能抵消漂移。
    """
    dv = KP * np.asarray(err_xy, dtype=np.float64) - KD * np.asarray(vel_xy, dtype=np.float64)
    dv = dv - WIND_GAIN * np.asarray(wind_xy, dtype=np.float64)
    return np.clip(dv, -CMD_MAX, CMD_MAX)


def encode(action):
    """归一化到 [-1, 1]。"""
    return np.asarray(action, dtype=np.float32) / CMD_MAX


def decode(norm_action):
    return np.asarray(norm_action, dtype=np.float64) * CMD_MAX
