"""
expert.py —— 专家控制器（行为克隆的"老师"）
==============================================
基于躯干前/后/左/右倾角，按比例输出关节修正目标角。
这是一个简单但稳定的 PD 比例控制器，作为端到端网络学习的目标。
"""
from __future__ import annotations
import numpy as np
from .sim import DEFAULT_STAND_TARGET

K_TILT_PITCH = 1.2   # 前后倾增益
K_TILT_ROLL = 1.0    # 左右倾增益


def expert_target(tilt_pitch: float, tilt_roll: float) -> np.ndarray:
    """
    输入：躯干前后倾（rad）、左右倾（rad）
    输出：15 维关节目标角（叠加在站姿上）
    """
    t = DEFAULT_STAND_TARGET.copy()
    # 前后倾：调整髋pitch和踝pitch
    corr_pitch = K_TILT_PITCH * tilt_pitch
    t[1] += corr_pitch   # hip_pitch_l
    t[6] += corr_pitch   # hip_pitch_r
    t[3] += 0.5 * corr_pitch   # ankle_pitch_l
    t[8] += 0.5 * corr_pitch   # ankle_pitch_r
    # 左右倾：调整髋roll和踝roll
    corr_roll = K_TILT_ROLL * tilt_roll
    t[0] -= corr_roll    # hip_roll_l
    t[5] += corr_roll    # hip_roll_r
    t[4] -= 0.5 * corr_roll   # ankle_roll_l
    t[9] += 0.5 * corr_roll   # ankle_roll_r
    return t
