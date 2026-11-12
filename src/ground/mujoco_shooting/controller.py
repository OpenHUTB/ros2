# -*- coding: utf-8 -*-
"""打靶的控制逻辑（纯函数，不依赖 mujoco，可单独单测）.

坐标约定：地面 XY 平面，车在原点，+X 为正前方，+Y 为左侧；
``yaw`` 是炮塔绕 Z 轴的角度（弧度），目标方位 = atan2(y, x)。
"""
from __future__ import annotations

import math

# 六个靶子：前三近、后三远（与 main.py 里的 t0..t5 顺序一致）
TARGETS = [
    (1.55, -1.25), (2.00, 0.00), (1.55, 1.25),
    (3.00, -1.35), (3.40, 0.00), (3.00, 1.35),
]

# mujoco qpos/qvel 布局：炮塔 yaw 在 0，轮子 1~4，靶子 5~10，球从 11 开始（每个自由体 7/6 维）
TARGET_QPOS0 = 5
BALL_Q0 = 11
BALL_V0 = 11
BALL_DOF_Q = 7
BALL_DOF_V = 6

TURRET_YAW_QPOS = 0
TURRET_YAW_QVEL = 0

# 靶子被击倒的判据：绕 Y 轴的铰链角超过该阈值（rad）
DOWN_THRESHOLD = 0.25

MUZZLE_OFFSET = (0.28, 0.0)      # 炮塔相对车体
BARREL_LENGTH = 0.35             # 炮口相对炮塔
MUZZLE_Z = 0.17                  # 出膛高度（m）
TARGET_Z = 0.25                  # 命中高度（m）
DEFAULT_FLIGHT_TIME = 0.45       # 默认飞行时间（s），力度按距离换算
GRAVITY = 9.81


def clamp(value, low=-15.0, high=15.0):
    return max(low, min(high, value))


def wrap_pi(angle):
    """折到 (-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def aim_error(yaw, target):
    """炮塔当前朝向与目标方位的夹角（已 wrap）."""
    tx, ty = target
    return wrap_pi(math.atan2(ty, tx) - yaw)


def down_flags(qpos, count=len(TARGETS), threshold=DOWN_THRESHOLD):
    """每个靶子是否已倒下（按铰链角绝对值判断）."""
    return [abs(float(qpos[TARGET_QPOS0 + i])) > threshold for i in range(count)]


def all_down(qpos, count=len(TARGETS), threshold=DOWN_THRESHOLD):
    return all(down_flags(qpos, count, threshold))


def select_target(yaw, targets=TARGETS, down=None):
    """选"还立着、且与炮塔朝向夹角最小"的靶子；全倒或没有可打的返回 -1.

    这正是文档里"炮塔像雷达一样识别还立着的靶子"的朴素实现：
    扫描到哪个就先打哪个，避免炮塔跨越大角度来回摆。
    """
    down = down or [False] * len(targets)
    best_index, best_err = -1, float("inf")
    for i, target in enumerate(targets):
        if down[i]:
            continue
        err = abs(aim_error(yaw, target))
        if err < best_err:
            best_index, best_err = i, err
    return best_index


def turret_command(yaw, yaw_rate, target, kp=6.0, kd=2.0, limit=15.0):
    """炮塔的 P-D 速度指令（速度执行器）."""
    return clamp(kp * aim_error(yaw, target) - kd * yaw_rate, -limit, limit)


def is_aligned(yaw, yaw_rate, target, angle_tol=0.05, rate_tol=0.05):
    return abs(aim_error(yaw, target)) < angle_tol and abs(yaw_rate) < rate_tol


def muzzle_position(yaw):
    """炮口的世界坐标 (x, y)（只用于发射起点与距离计算）."""
    radius = MUZZLE_OFFSET[0] + BARREL_LENGTH
    return (radius * math.cos(yaw), radius * math.sin(yaw))


def launch_velocity(muzzle_xy, target_xy, flight_time=DEFAULT_FLIGHT_TIME,
                    muzzle_z=MUZZLE_Z, target_z=TARGET_Z, gravity=GRAVITY, boost=1.0):
    """给定飞行时间，算出打到目标所需的初速度 (vx, vy, vz) 与水平距离.

    ``boost`` 用来补枪：打偏后按比例加大水平速度（配合固定飞行时间，
    弹道整体更"平"一点，容错更大）。
    """
    dx = target_xy[0] - muzzle_xy[0]
    dy = target_xy[1] - muzzle_xy[1]
    dist = math.hypot(dx, dy)
    if dist <= 1e-9:
        return 0.0, 0.0, 0.0, 0.0
    speed = dist / flight_time * boost
    vx = speed * dx / dist
    vy = speed * dy / dist
    vz = (target_z - muzzle_z + 0.5 * gravity * flight_time ** 2) / flight_time
    return vx, vy, vz, dist
