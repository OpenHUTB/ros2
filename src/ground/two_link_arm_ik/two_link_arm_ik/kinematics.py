"""二连杆机械臂的正向与逆向运动学。

约定：基座固定在原点，第一关节相对 x 轴正方向转 q1，第二关节相对第一连杆
转 q2（肘部向上为 q2 > 0）。画布 y 轴向下，绘制时对 y 取反。
"""

import math

from . import config


def forward_kinematics(q1, q2, l1=None, l2=None):
    """返回 (肘部, 末端) 两个点坐标。"""
    l1 = config.L1 if l1 is None else l1
    l2 = config.L2 if l2 is None else l2
    elbow = (l1 * math.cos(q1), l1 * math.sin(q1))
    a2 = q1 + q2
    tip = (elbow[0] + l2 * math.cos(a2), elbow[1] + l2 * math.sin(a2))
    return elbow, tip


def reachable(target, l1=None, l2=None):
    """目标是否落在工作空间内（含边界容差）。"""
    l1 = config.L1 if l1 is None else l1
    l2 = config.L2 if l2 is None else l2
    r = math.hypot(target[0], target[1])
    return abs(l1 - l2) - 1e-9 <= r <= l1 + l2 + 1e-9


def inverse_kinematics(target, elbow_up=True, l1=None, l2=None):
    """两连杆解析逆解，返回 (q1, q2)；目标不可达时返回 None。"""
    l1 = config.L1 if l1 is None else l1
    l2 = config.L2 if l2 is None else l2
    x, y = target
    r2 = x * x + y * y
    r = math.sqrt(r2)
    if r < 1e-9:
        # 原点处有无穷多解，取全伸展姿态
        return (0.0, 0.0)
    if not reachable(target, l1, l2):
        return None
    cos_q2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    cos_q2 = max(-1.0, min(1.0, cos_q2))
    q2 = math.acos(cos_q2)
    if not elbow_up:
        q2 = -q2
    q1 = math.atan2(y, x) - math.atan2(
        l2 * math.sin(q2), l1 + l2 * math.cos(q2))
    q1 = wrap(q1)
    return (q1, q2)


def wrap(a):
    """把角度折到 (-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def clamp_joints(q1, q2):
    """按关节限位裁剪，返回 (q1, q2, 是否被裁剪)。"""
    c1 = max(config.Q1_MIN, min(config.Q1_MAX, q1))
    c2 = max(config.Q2_MIN, min(config.Q2_MAX, q2))
    clipped = abs(c1 - q1) > 1e-9 or abs(c2 - q2) > 1e-9
    return c1, c2, clipped
