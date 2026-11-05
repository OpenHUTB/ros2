# -*- coding: utf-8 -*-
"""humanoid_fall_protection 的默认参数。

所有参数都可以被 config/params.yaml 覆盖（standalone 与 ROS2 共用同一份 yaml）。
单位统一用 SI：米、秒、弧度、牛顿、牛米；只有对外的阈值角度用「度」，
避免在 yaml 里写 0.5236 这种看不出量级的数。
"""

#: 15 个受控关节，顺序与 models/humanoid_fall.xml 中的 actuator 声明一致
JOINT_NAMES = (
    "hip_roll_l", "hip_pitch_l", "knee_l", "ankle_pitch_l", "ankle_roll_l",
    "hip_roll_r", "hip_pitch_r", "knee_r", "ankle_pitch_r", "ankle_roll_r",
    "shoulder_pitch_l", "elbow_l", "shoulder_pitch_r", "elbow_r", "neck",
)

#: 站立姿态（微屈膝，避免膝关节奇异）
STAND_POSE = {
    "hip_pitch_l": 0.15, "knee_l": -0.30, "ankle_pitch_l": 0.15,
    "hip_pitch_r": 0.15, "knee_r": -0.30, "ankle_pitch_r": 0.15,
}

#: 前扑保护姿态。
#: 手向前伸（肩 pitch 取负 = 手臂前摆）、肘微屈吸收冲击、抬头避免面部着地、
#: 屈膝把重心压低。这组目标角不是拍脑袋定的：肩 -1.45 rad 让手臂接近水平前伸，
#: 保证肩到手掌的 0.58 m 长度能在地面之前触地；肘 -0.65 rad 留出缓冲行程
#: 而不是硬撑（硬撑时峰值力反而更高，见 docs 的阈值扫描表）。
PROTECT_FORWARD = {
    "shoulder_pitch_l": -1.20, "shoulder_pitch_r": -1.20,
    "elbow_l": -0.65, "elbow_r": -0.65,
    "neck": -0.35,
    "hip_pitch_l": 1.20, "hip_pitch_r": 1.20,
    "knee_l": -1.30, "knee_r": -1.30,
    "ankle_pitch_l": 0.30, "ankle_pitch_r": 0.30,
}

#: 后仰保护姿态：收下巴（neck 正 = 头前倾）、手臂后摆撑地、屈髋蜷缩
PROTECT_BACKWARD = {
    "shoulder_pitch_l": 0.90, "shoulder_pitch_r": 0.90,
    "elbow_l": -0.55, "elbow_r": -0.55,
    "neck": 0.55,
    "hip_pitch_l": 0.95, "hip_pitch_r": 0.95,
    "knee_l": -1.05, "knee_r": -1.05,
    "ankle_pitch_l": 0.15, "ankle_pitch_r": 0.15,
}

#: 默认参数表。键名与 config/params.yaml 一一对应。
DEFAULTS = {
    # --- 仿真 ---
    "sim_rate": 500.0,            # 物理步进频率 (Hz)，等于模型 timestep 的倒数
    "publish_rate": 30.0,         # ROS2 话题发布频率 (Hz)
    "visualize": True,

    # --- 站立 PD ---
    "kp": 120.0,                  # 关节比例增益 (N·m/rad)
    "kv": 6.0,                    # 关节微分增益 (N·m·s/rad)
    "torque_limit": 300.0,        # 关节力矩限幅 (N·m)

    # --- 站姿维持：踝关节质心/倾角反馈 ---
    # 只有关节 PD 时机器人 1.8 s 就会自己前倾倒地（见 docs 的对照实验），
    # 这三项是「站得住」的前提，不是跌倒保护的一部分。
    "ankle_lean_gain": 300.0,     # 倾角 → 踝力矩 (N·m/rad)
    "ankle_lean_rate_gain": 60.0,  # 倾角速率 → 踝力矩 (N·m·s/rad)
    "ankle_com_vel_gain": 900.0,  # 质心水平速度 → 踝力矩 (N·m·s/m)

    # --- 跌倒判据（参考 Asimov 1 safe-fall，角速度阈值按本站场景调整）---
    # 倾角 30° 与其部署默认一致；角速度取其训练范围上限附近 3.0 rad/s，
    # 原因是本站是站立场景：2.0 rad/s 会被推力冲击的瞬时角速度误触发
    # （150 N 明明站得住却进入保护），实测 3.0 才能区分 150 N 与 200 N。
    "tilt_threshold_deg": 30.0,   # 躯干倾角阈值 (deg)
    "tilt_rate_threshold": 3.0,   # 躯干倾角速率阈值 (rad/s)
    "detector_mode": "or",        # "or" = 任一满足即触发；"and" = 同时满足

    # --- 保护动作 ---
    "protect_enable": True,
    "protect_kp": 1200.0,         # 保护阶段的关节增益（要高，才来得及摆臂）
    "protect_kv": 60.0,
    "protect_direction": "auto",  # auto / forward / backward

    # --- 扰动 ---
    "push_force": 300.0,          # 推扰力 (N)，+x 为向前推（人向前扑）
    "push_duration": 0.08,        # 持续时间 (s)
    "push_delay": 1.0,            # 起始时刻 (s)，先站 1 s 再推

    # --- 判定与统计 ---
    "settle_time": 3.5,           # 单次仿真时长 (s)
    "head_contact_height": 0.12,  # 头部球心低于此高度视为「贴地」(m)
}

#: 三种被比较的策略
STRATEGIES = ("rigid", "limp", "protect")

#: 策略的中文名（报告、图表、ROS2 参数都用它）
STRATEGY_LABELS = {
    "rigid": "僵直",
    "limp": "瘫软",
    "protect": "保护",
}

#: 报告中使用的完整中文名
STRATEGY_FULL = {
    "rigid": "僵直（冻结站姿，僵硬倒下）",
    "limp": "瘫软（力矩置零，断电式倒地）",
    "protect": "保护（屈膝蜷身，落地分散冲击）",
}
