"""
expert.py —— 专家控制器（行为克隆的"老师"）
==============================================
输出 15 维【力矩增量】：叠加在关节 PD 之上，由踝关节把骨盆水平位移推回。

设计依据（见 README "为什么原来的 teacher 站不住"）：
    纯位置 PD 不含任何水平方向的约束，腿部下沉会带来骨盆沿 x 的漂移，
    而脚掌支撑多边形一旦失去骨盆投影就必然翻倒（实测 1.29~1.33 s）。
    给踝 pitch 叠加 (kv*vx + kx*x) 正反馈 + 倾角项后，仿真内可稳站 30 s、
    抗 50 N 推扰恢复，力矩噪声 ±30% 仍稳定。

常量取名 `K_*`，量纲：K_VX [N·m/(m/s)]、K_X [N·m/m]、K_TILT [N·m/rad]。
"""
from __future__ import annotations
import numpy as np

# —— 前后方向（已验证组合）——
K_VX = 1500.0        # 骨盆水平速度反馈
K_X = 800.0          # 骨盆水平位移反馈
K_TILT = 300.0       # 躯干前后倾反馈
K_TILT_RATE = 30.0   # 倾角速度反馈

# —— 左右方向（小增益，仅抑制 roll 漂移）——
K_ROLL = 200.0
K_ROLL_RATE = 20.0

ANKLE_PITCH_L, ANKLE_PITCH_R = 3, 8
HIP_ROLL_L, ANKLE_ROLL_L = 0, 4
HIP_ROLL_R, ANKLE_ROLL_R = 5, 9


def expert_action(sim) -> np.ndarray:
    """根据当前状态输出 15 维力矩增量（N·m）。"""
    u = np.zeros(15, dtype=np.float64)

    # 前后：踝策略。正号方向已由仿真扫描确定（负号或加到髋关节都会发散）
    bal_pitch = (K_VX * sim.pelvis_vx + K_X * sim.pelvis_x
                 + K_TILT * sim.torso_tilt() + K_TILT_RATE * sim.tilt_rate)
    u[ANKLE_PITCH_L] += bal_pitch
    u[ANKLE_PITCH_R] += bal_pitch

    # 左右：髋/踝 roll 反向配对，抑制侧向漂移
    roll = sim.torso_tilt_y()
    bal_roll = K_ROLL * roll + K_ROLL_RATE * float(sim.data.qvel[3])
    u[HIP_ROLL_L] -= bal_roll
    u[HIP_ROLL_R] += bal_roll
    u[ANKLE_ROLL_L] -= 0.5 * bal_roll
    u[ANKLE_ROLL_R] += 0.5 * bal_roll
    return u


def expert_target(tilt_pitch: float, tilt_roll: float) -> np.ndarray:
    """（保留）旧的角度型专家接口，仅做倾角比例修正，已不再用于采集。

    它在 0 N 推力下 1.33 s 就会倒，保留只为向后兼容。
    """
    from .sim import DEFAULT_STAND_TARGET
    t = DEFAULT_STAND_TARGET.copy()
    corr_pitch = 1.2 * tilt_pitch
    t[1] += corr_pitch
    t[6] += corr_pitch
    t[3] += 0.5 * corr_pitch
    t[8] += 0.5 * corr_pitch
    corr_roll = 1.0 * tilt_roll
    t[0] -= corr_roll
    t[5] += corr_roll
    t[4] -= 0.5 * corr_roll
    t[9] += 0.5 * corr_roll
    return t
