#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手势指令 -> 桥接层消息的**纯逻辑翻译**（不 import rospy，可离线单测）。

桥接层契约（``src/air/carlair_ros_bridge``）
------------------------------------------
===============  =========================  ============================================
话题              类型                        语义
===============  =========================  ============================================
``/uav/cmd_vel``   ``geometry_msgs/Twist``    机体系：``x`` 前 / ``y`` 左 / ``z`` 上，
                                             ``angular.z`` 偏航角速度 (rad/s)
``/uav/goal``      ``geometry_msgs/Point``    ENU 世界系目标点
``/uav/action``    ``std_msgs/String``        ``takeoff`` / ``land`` / ``hover``
                                             （本模块给桥接层补的动作通道）
``/uav/status``    ``std_msgs/String``        ``READY`` / ``HOVER`` / ``VELOCITY`` / ...
===============  =========================  ============================================

两条必须遵守的桥接层安全语义
----------------------------
1. ``rate/cmd_timeout``（默认 0.5 s）内没有**新**的速度指令就自动悬停 —— 所以一次
   手势动作必须"按帧重复下发"，不能只发一条。
2. ``control/cmd_duration``（默认 0.1 s）是**单条**速度指令的作用时长 —— 所以
   N = ``duration × hz`` 条、周期 ``1/hz < cmd_timeout`` 的指令流，才能让飞机
   连续运动 ``duration`` 秒（``hz=20`` → 周期 0.05 s，安全）。

因此一次手势被拆成四类：
* 动作类（``takeoff`` / ``land`` / ``hover``）→ 一条 ``/uav/action``
* 速度类（``forward`` / ``up`` / ``down`` / ``yaw_left`` / ``yaw_right``）→ N 条 ``/uav/cmd_vel``
* 目标类（``home``）→ 一条 ``/uav/goal``
* 急停（``estop``）→ 若干条零速度 + 一条 ``/uav/action``
"""
from __future__ import annotations

# --------------------------------------------------------------------------- #
# 指令 token：与 gesture_control_drone/scripts/gesture_map.py 保持一致
# （tests/test_gesture_bridge_local.py 会读对方源码做契约校验，防止两边漂移）
# --------------------------------------------------------------------------- #
CMD_TAKEOFF = "takeoff"
CMD_LAND = "land"
CMD_HOVER = "hover"
CMD_FORWARD = "forward"
CMD_UP = "up"
CMD_DOWN = "down"
CMD_YAW_LEFT = "yaw_left"
CMD_YAW_RIGHT = "yaw_right"
CMD_HOME = "home"
CMD_ESTOP = "estop"

ALL_COMMANDS = (CMD_TAKEOFF, CMD_LAND, CMD_HOVER, CMD_FORWARD, CMD_UP,
                CMD_DOWN, CMD_YAW_LEFT, CMD_YAW_RIGHT, CMD_HOME, CMD_ESTOP)

#: 走 ``/uav/action`` 的指令
ACTION_COMMANDS = (CMD_TAKEOFF, CMD_LAND, CMD_HOVER)
#: 走 ``/uav/cmd_vel`` 指令流（机体系速度）
VELOCITY_COMMANDS = (CMD_FORWARD, CMD_UP, CMD_DOWN, CMD_YAW_LEFT, CMD_YAW_RIGHT)
#: 走 ``/uav/goal`` 的指令
GOAL_COMMANDS = (CMD_HOME,)

#: 急停允许的落地动作（与 gesture_control_drone 的 estop_action 对齐）
ESTOP_ACTIONS = ("hover", "land")

KIND_ACTION = "action"
KIND_VELOCITY = "velocity"
KIND_GOAL = "goal"
KIND_ESTOP = "estop"


def is_command(text):
    """是不是本模块认识的手势指令 token。"""
    return text in ALL_COMMANDS


def kind_of(command):
    """指令属于哪一类通道；未知返回 ``None``。"""
    if command in ACTION_COMMANDS:
        return KIND_ACTION
    if command in VELOCITY_COMMANDS:
        return KIND_VELOCITY
    if command in GOAL_COMMANDS:
        return KIND_GOAL
    if command == CMD_ESTOP:
        return KIND_ESTOP
    return None


def body_velocity(command, speed=0.8, yaw_rate=0.5):
    """速度类指令 -> 机体系 ``(vx, vy, vz, wz)``（ROS 约定：前/左/上，rad/s）。

    * ``forward``   -> ``(+speed, 0, 0, 0)``
    * ``up``        -> ``(0, 0, +speed, 0)``   （``z`` 向上为正）
    * ``down``      -> ``(0, 0, -speed, 0)``
    * ``yaw_left``  -> ``(0, 0, 0, +yaw_rate)``（ROS ``angular.z`` 正向 = 逆时针 = 左转）
    * ``yaw_right`` -> ``(0, 0, 0, -yaw_rate)``
    * ``estop``     -> 全零（刹停）

    非速度类返回 ``None``；``speed`` / ``yaw_rate`` 取绝对值，避免符号写错。
    """
    v = abs(float(speed))
    w = abs(float(yaw_rate))
    table = {
        CMD_FORWARD: (v, 0.0, 0.0, 0.0),
        CMD_UP: (0.0, 0.0, v, 0.0),
        CMD_DOWN: (0.0, 0.0, -v, 0.0),
        CMD_YAW_LEFT: (0.0, 0.0, 0.0, w),
        CMD_YAW_RIGHT: (0.0, 0.0, 0.0, -w),
    }
    if command == CMD_ESTOP:
        return (0.0, 0.0, 0.0, 0.0)
    return table.get(command)


def stream_count(duration, hz):
    """一次动作要发多少条速度指令：``round(duration × hz)``，至少 1 条。"""
    duration = abs(float(duration))
    hz = float(hz)
    if hz <= 0:
        raise ValueError("stream_hz 必须大于 0，收到 %r" % hz)
    return max(1, int(round(duration * hz)))


def action_for(command, estop_action="hover"):
    """动作类指令 -> ``/uav/action`` 的字符串；非动作类返回 ``None``。"""
    if command in ACTION_COMMANDS:
        return command
    if command == CMD_ESTOP:
        action = str(estop_action or "hover").lower()
        if action not in ESTOP_ACTIONS:
            raise ValueError("estop_action 必须是 %s 之一，收到 %r"
                             % (list(ESTOP_ACTIONS), estop_action))
        return action
    return None


def goal_for(home_altitude=3.0):
    """``home`` -> ENU 目标点 ``(0, 0, home_altitude)``。"""
    return (0.0, 0.0, abs(float(home_altitude)))


def plan(command, speed=0.8, yaw_rate=0.5, duration=1.0, stream_hz=20.0,
         home_altitude=3.0, estop_action="hover", estop_zeros=5):
    """把一条手势指令翻译成"要发什么"的完整计划（纯数据，便于断言与日志）。

    返回 ``dict``：
    * ``kind``       : ``action`` / ``velocity`` / ``goal`` / ``estop``
    * ``velocities`` : 需要依次下发的 ``(vx, vy, vz, wz)`` 列表（其余类为空）
    * ``action``     : 需要发的 ``/uav/action`` 字符串（如 ``takeoff`` / ``hover``）
    * ``goal``       : 需要发的 ``(x, y, z)``（仅 ``home``）
    * ``hz``         : 指令流频率（发送间隔 ``1/hz``）
    """
    kind = kind_of(command)
    if kind is None:
        return None

    if kind == KIND_VELOCITY:
        vel = body_velocity(command, speed=speed, yaw_rate=yaw_rate)
        return {"kind": kind, "velocities": [vel] * stream_count(duration, stream_hz),
                "action": None, "goal": None, "hz": float(stream_hz)}

    if kind == KIND_ACTION:
        return {"kind": kind, "velocities": [], "action": action_for(command),
                "goal": None, "hz": float(stream_hz)}

    if kind == KIND_GOAL:
        return {"kind": kind, "velocities": [], "action": None,
                "goal": goal_for(home_altitude), "hz": float(stream_hz)}

    # estop：先按同样频率发若干条零速度刹住，再交给动作通道
    zeros = max(1, int(estop_zeros))
    return {"kind": kind, "velocities": [(0.0, 0.0, 0.0, 0.0)] * zeros,
            "action": action_for(command, estop_action=estop_action),
            "goal": None, "hz": float(stream_hz)}


def describe_map():
    """可读的"指令 -> 通道"映射，供节点启动日志打印。"""
    out = []
    for command in ALL_COMMANDS:
        if command in ACTION_COMMANDS:
            out.append("%s->/uav/action" % command)
        elif command in VELOCITY_COMMANDS:
            out.append("%s->/uav/cmd_vel" % command)
        elif command in GOAL_COMMANDS:
            out.append("%s->/uav/goal" % command)
        else:
            out.append("%s->cmd_vel(0)+/uav/action" % command)
    return "，".join(out)
