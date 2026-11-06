# -*- coding: utf-8 -*-
"""PD 控制器 -> 拮抗肌肉激活 的纯逻辑 (不依赖 ROS2 / MuJoCo, 便于单元测试)."""


def clamp(x, lo=0.0, hi=1.0):
    return max(lo, min(hi, x))


def pd_to_activation(error_rad, vel, kp, kd, max_torque, limit_reached=False):
    """
    将跟踪误差通过 PD 转化为拮抗肌肉激活 (flexor/extensor ∈ [0,1])。

    参数:
      error_rad    : 目标角度 - 当前角度 [rad]
      vel          : 关节角速度 [rad/s]
      kp, kd       : 比例 / 微分增益
      max_torque   : 肌肉等效最大力矩 [N·m] (激活归一化分母)
      limit_reached: 是否已到达屈曲限位
    返回:
      (flexor, extensor) 激活值
    """
    torque = kp * error_rad - kd * vel

    # 数值检查: 非法输入 -> 安全停止 (必须在 clamp 之前判断, 否则 nan 会被钳制)
    if not _isfinite(torque):
        return 0.0, 0.0

    # 激活归一化与拮抗分配: 正力矩 -> 屈肌, 负力矩 -> 伸肌
    if torque >= 0.0:
        flex = clamp(torque / max_torque)
        ext = 0.0
    else:
        flex = 0.0
        ext = clamp(-torque / max_torque)

    # 屈曲限位保护: 已到限位且仍需屈曲 -> 停止屈肌
    if limit_reached and torque > 0:
        flex = 0.0

    return flex, ext


def _isfinite(x):
    import math
    return math.isfinite(x)
