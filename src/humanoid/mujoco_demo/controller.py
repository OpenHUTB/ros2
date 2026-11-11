# -*- coding: utf-8 -*-
"""人形机器人站立控制（纯函数，不依赖 mujoco）.

思路很简单，也是本页想讲清楚的两件事：

1. **PD 位置控制**：给每个关节一个目标角 ``q_des``（站立就是 0），
   按 ``tau = kp * (q_des - q) - kd * qd`` 算力矩；
2. **重力补偿**：光靠 PD，关节会被自重压出稳态误差（腿会"软"下去）。
   把 ``mj_rne`` 算出的偏置力 ``qfrc_bias`` 前馈加上，PD 只需要修小误差。

两个函数都只吃数组、返回数组，方便单测。
"""
from __future__ import annotations

import math


def clamp(value, limit):
    return max(-limit, min(limit, value))


def pd_torque(q, qd, q_des, kp, kd, bias=None):
    """逐关节算力矩；bias 给定时做重力/科氏前馈."""
    if not (len(q) == len(qd) == len(q_des) == len(kp) == len(kd)):
        raise ValueError("q/qd/q_des/kp/kd 长度必须一致")
    tau = []
    for i in range(len(q)):
        value = kp[i] * (q_des[i] - q[i]) - kd[i] * qd[i]
        if bias is not None:
            value += bias[i]
        tau.append(value)
    return tau


def saturate(tau, limit):
    return [clamp(t, limit) for t in tau]


def tilt_deg(quat_wxyz):
    """骨盆相对竖直方向的倾角（度）。四元数用 mujoco 的 wxyz 顺序."""
    w, x, y, z = quat_wxyz
    # 世界 z 轴在机体系下的分量 = 旋转矩阵第三列
    r20 = 2.0 * (x * z - w * y)
    r21 = 2.0 * (y * z + w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, r22))))


def is_standing(height, base_height, tolerance=0.1, max_tilt_deg=12.0, quat_wxyz=None):
    """是否算"站住了"：高度不低于基准的 (1-tolerance)，且倾角不超过阈值."""
    if height < base_height * (1.0 - tolerance):
        return False
    if quat_wxyz is not None and tilt_deg(quat_wxyz) > max_tilt_deg:
        return False
    return True
