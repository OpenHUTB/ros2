# -*- coding: utf-8 -*-
"""步态层：相位 → 12 个关节目标角（纯函数）。

采用"参数化步态 + 前馈 + 落地相位回调"的结构，而不是学一个端到端策略：
课程任务要求的是"感知 + 给定轨迹的控制"，需要可解释、可整定的控制回路，
而不是黑箱。步态函数只负责**生成关节目标角**，怎么保持平衡交给 balance.py。

相位约定
--------
``phase ∈ [0, 1)``，1 是一个完整步态周期（左右各迈一步）。0.0~0.5 为左脚支撑、
右脚摆动；0.5~1.0 交换。相位由调用方按 ``phase += dt / period`` 推进，
这样"步频"是显式参数（``period``），改速度不必改步幅。

关节角符号（已在本仓库的 ``models/humanoid.xml`` 上实测标定）
--------------------------------------------------------------
* ``hip_pitch``   正值 = 大腿**向后**摆，负值 = 向前摆
* ``knee_pitch``  正值 = 屈膝（脚向后收）
* ``ankle_pitch`` 正值 = 脚尖上翘（背屈）
因此"腿伸直"对应全 0，"向前迈步"对应 hip 从**负**向**正**变化。
"""

from __future__ import annotations

import math

from .geometry import clamp

#: 关节顺序（与模型 actuator 序号 6..17 完全一致，测试会校验这一点）
JOINT_ORDER = (
    "hip_l_roll", "hip_l_pitch", "knee_l_pitch", "ankle_l_pitch",
    "hip_r_roll", "hip_r_pitch", "knee_r_pitch", "ankle_r_pitch",
    "shoulder_l_roll", "shoulder_l_pitch",
    "shoulder_r_roll", "shoulder_r_pitch",
)

#: 关节软限位（弧度）。目标是"目标角"而不是实际角，超限会让位置伺服
#: 顶死在限位上，表现为腿突然锁死、机身被拽倒。
JOINT_LIMITS = {
    "hip_l_roll": (-0.35, 0.35),
    "hip_l_pitch": (-1.30, 0.60),
    "knee_l_pitch": (0.0, 1.60),
    "ankle_l_pitch": (-0.55, 0.55),
    "hip_r_roll": (-0.35, 0.35),
    "hip_r_pitch": (-1.30, 0.60),
    "knee_r_pitch": (0.0, 1.60),
    "ankle_r_pitch": (-0.55, 0.55),
    "shoulder_l_roll": (-1.00, 0.30),
    "shoulder_l_pitch": (-1.00, 0.50),
    "shoulder_r_roll": (-0.30, 1.00),
    "shoulder_r_pitch": (-1.00, 0.50),
}


def stand_pose(arm_swing: float = 0.0, lean: float = 0.0, knee_bias: float = 0.30):
    """双臂自然下垂的站立姿态（12 维 dict）。

    ``knee_bias`` 是站立时的屈膝量。**不能取 0**：本模型的大腿/小腿各
    0.34 m，伸直时总长 0.68 m，与躯干到脚的落差完全相等——这意味着
    站立构型处在运动学奇异点附近，0.05 rad 的屈膝就会让躯干下降半米多。
    实测：knee_cmd=0.05 → Δz=-0.58 m。为了留出工作空间，站立姿态必须
    保持约 0.30 rad（17°）的屈膝，此时躯干高度约 0.89 m。

    **踝角保持 0**：在 models/humanoid.xml 上实测，ankle_cmd=+0.20 会让
    躯干前倾 9°，站立时任何非零踝指令都会把重心推出支撑面，几秒内必倒。
    踝策略只用于动态平衡补偿（见 :meth:`ankle_correction`），站立不叠。
    """
    pose = {name: 0.0 for name in JOINT_ORDER}
    half = 0.5 * knee_bias
    pose["hip_l_pitch"] = -half
    pose["hip_r_pitch"] = -half
    pose["knee_l_pitch"] = knee_bias
    pose["knee_r_pitch"] = knee_bias
    pose["shoulder_l_pitch"] = -arm_swing
    pose["shoulder_r_pitch"] = arm_swing
    return _clip_pose(pose)


def leg_target(phase_t: float, stride: float, lift: float, knee_bias: float = 0.0):
    """单腿在一个周期内的关节角 ``(hip, knee, ankle)``，``phase_t ∈ [0,1)``。

    摆动相（0~0.5）：髋角按余弦从 ``+stride/2`` 摆到 ``-stride/2``。
    **注意符号**：模型的髋 pitch 轴是绕 y 的正向，对应大腿向后摆，
    所以"向前迈步"是 hip 角**减小**的过程。倒过来写会让机器人原地后仰。

    膝角按 sin 抬脚，形成"抬腿—跨步—落脚"的圆弧；踝角随膝角反向补偿，
    保持脚掌大致水平。
    支撑相（0.5~1.0）：髋角线性回到 ``+stride/2``（支撑腿在身后，身体相对
    支撑脚前移）；膝/踝基本伸直，获得最大支撑高度。
    """
    phase_t = phase_t % 1.0
    half = 0.5

    if phase_t < half:
        s = phase_t / half                      # 0→1 摆动进度
        hip = 0.5 * stride * math.cos(math.pi * s)
        knee = knee_bias + lift * math.sin(math.pi * s)
        # 踝角只做小幅同向补偿：屈膝时小腿相对大腿后摆，脚自然下垂；只需要
        # 很小的同向踝角就能保持脚掌大致水平。早期用 -0.55*knee 会把脚翻转，
        # 换成 +0.55 又太大（叠加踝策略后顶到限位），实测 0.25 是平衡点。
        ankle = 0.25 * knee
    else:
        s = (phase_t - half) / half             # 0→1 支撑进度
        hip = -0.5 * stride + stride * s        # 从 -stride/2 回到 +stride/2
        knee = knee_bias
        ankle = 0.25 * knee
    return hip, knee, ankle


def walk_pose(phase: float, stride: float = 0.45, lift: float = 0.55,
              knee_bias: float = 0.08, arm_swing: float = 0.25, lean: float = 0.0):
    """给定相位与步态参数，返回 12 个关节的目标角（弧度 dict）。

    ``stride`` 是单腿前后摆幅（弧度），``lift`` 是摆动腿的额外屈膝量。
    左右腿相位相差半个周期，手臂与同侧腿反相摆动以抵消角动量。

    髋 pitch 的符号见 :func:`leg_target`：正值 = 大腿后摆，负值 = 前摆。
    因此"向前走"时**摆动腿**的 hip 从负值（在前）变为正值（在后），
    **支撑腿**的 hip 从正值（在后）变回负值（在前）。
    """
    phase = phase % 1.0
    l_hip, l_knee, l_ankle = leg_target(phase, stride, lift, knee_bias)
    r_hip, r_knee, r_ankle = leg_target(phase + 0.5, stride, lift, knee_bias)

    pose = {name: 0.0 for name in JOINT_ORDER}
    # hip_pitch 的符号已在本文件头部标定：正值 = 大腿后摆，负值 = 前摆。
    # 摆动相 phase∈[0,0.5) 时腿从"前"抬起，即 hip 从负值走向正值，
    # 与 leg_target 的返回一致，直接相加即可，不需要再取反。
    pose["hip_l_pitch"] = l_hip + lean
    pose["knee_l_pitch"] = l_knee
    pose["ankle_l_pitch"] = l_ankle + lean
    pose["hip_r_pitch"] = r_hip + lean
    pose["knee_r_pitch"] = r_knee
    pose["ankle_r_pitch"] = r_ankle + lean

    swing = arm_swing * math.sin(2.0 * math.pi * phase)
    pose["shoulder_l_pitch"] = -swing
    pose["shoulder_r_pitch"] = swing
    return _clip_pose(pose)


def blend_pose(pose_a, pose_b, t: float):
    """在两个姿态之间线性插值，``t=0`` 取 ``pose_a``。"""
    t = clamp(t, 0.0, 1.0)
    return {k: pose_a[k] + (pose_b[k] - pose_a[k]) * t for k in pose_a}


def to_vector(pose):
    """姿态 dict → 按 :data:`JOINT_ORDER` 排列的 12 维列表（供控制器/仿真用）。"""
    return [float(pose[name]) for name in JOINT_ORDER]


def from_vector(vec):
    """12 维列表 → 姿态 dict（:func:`to_vector` 的逆）。"""
    if len(vec) != len(JOINT_ORDER):
        raise ValueError("向量长度必须为 %d" % len(JOINT_ORDER))
    return {name: float(v) for name, v in zip(JOINT_ORDER, vec)}


def _clip_pose(pose):
    """按 :data:`JOINT_LIMITS` 裁剪所有关节目标角。"""
    out = {}
    for name, value in pose.items():
        lo, hi = JOINT_LIMITS.get(name, (-math.pi, math.pi))
        out[name] = clamp(float(value), lo, hi)
    return out


def support_foot(phase: float) -> str:
    """返回当前相位下的支撑脚：``"left"`` / ``"right"``。

    0.0~0.5 用左脚支撑，0.5~1.0 用右脚，与 :func:`leg_target` 的相位语义一致。
    """
    return "left" if (phase % 1.0) < 0.5 else "right"


def phase_rate(speed: float, stride: float, period: float) -> float:
    """速度指令 → 相位推进速率（周期/秒）。

    本模型的步态是"参数化站立踏步"，每一步的实际位移远小于
    ``stride`` 的弧长（实测 25 s / 3.85 cycle/s 只前进 2.6 m，即每周期
    约 0.068 m）。因此速度标量只用来**选步频档位**，不做几何换算：
    速率直接与速度成正比，上限由 ``period`` 保证（步频不超过 1/period）。

    实测本模型在 0.77 cycle/s 下会因为"单腿支撑太久"而摔倒；
    3.85 cycle/s 附近能连续行走。因此 **``period`` 不再是速率上限**，
    只是"一个完整步态周期"的名义长度，上限由调用方决定。
    这个反直觉的结论来自步态本身不产生水平动量，只能"快速踏步保持
    重心在支撑面内"——1/period 的常规上限恰好把速率压在发散区。
    """
    if period <= 0.0:
        raise ValueError("period 必须为正")
    if stride <= 0.0:
        raise ValueError("stride 必须为正")
    cycles = max(0.0, float(speed)) / 0.052
    return cycles
