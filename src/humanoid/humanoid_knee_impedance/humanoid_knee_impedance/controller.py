"""膝关节阻抗控制律。

纯函数实现，便于单元测试与复用。阻抗模型为虚拟弹簧-阻尼：

    tau = kp * (q_des - q) - kd * qd

再按力矩上限截断。注意这里只做 PD，不含重力前馈，因此存在与 kp 成
反比的稳态误差——这正是本模块要展示的参数整定现象之一。
"""

import math


def impedance_torque(angle, velocity, target, kp, kd, torque_limit):
    """计算阻抗控制力矩。

    参数:
        angle: 当前关节角 (rad)。
        velocity: 当前关节角速度 (rad/s)。
        target: 目标平衡角 (rad)。
        kp: 虚拟弹簧刚度 (N·m/rad)，必须非负。
        kd: 虚拟阻尼系数 (N·m·s/rad)，必须非负。
        torque_limit: 力矩上限 (N·m)，必须为正。

    返回:
        截断后的关节力矩 (N·m)。输入含 NaN/Inf 时返回 0.0，
        避免非法传感器读数被执行器直接放大。
    """
    if kp < 0.0 or kd < 0.0:
        raise ValueError('kp 与 kd 必须为非负数')
    if torque_limit <= 0.0:
        raise ValueError('torque_limit 必须为正数')
    if not (math.isfinite(angle) and math.isfinite(velocity)
            and math.isfinite(target)):
        return 0.0
    torque = kp * (target - angle) - kd * velocity
    return max(-torque_limit, min(torque_limit, torque))
