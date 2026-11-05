#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手势 / 指令词表 —— **全模块唯一的一份定义**。

为什么单独抽一个文件
--------------------
早期版本里感知节点发布的是中文手势名（``"张开手掌"`` / ``"握拳"`` / …），
而控制节点比较的是英文指令（``"open"`` / ``"fist"`` / ``"ok"``），两者交集为空，
**手势永远不会触发任何飞行动作**；而且 ``main.py`` 里又写了第三套词表。
把词表集中到这里、两端 ``import`` 同一个常量，这类"各写一套"的 bug
在结构上就不可能再发生。

约定
----
* **线上传输一律用 ASCII token**（``PALM_OPEN`` / ``takeoff`` …），
  与界面显示语言解耦，避免中文编码、大小写、别名带来的歧义；
* 中文只用于日志与界面显示（:func:`gesture_cn` / :func:`command_cn`）；
* 每条指令都必须有执行实现（``drone_client.AirSimDrone`` 初始化时自检），
  每个手势都必须映射到已知指令（本文件末尾自检）。
"""
from __future__ import annotations

# --------------------------------------------------------------------------- #
# 手势 token（感知节点产出）
# --------------------------------------------------------------------------- #
GESTURE_PALM = "PALM_OPEN"          # 张开手掌
GESTURE_FIST = "FIST"               # 握拳
GESTURE_POINT = "POINT"             # 食指指向
GESTURE_VICTORY = "VICTORY"         # 胜利手势
GESTURE_THUMB_UP = "THUMB_UP"       # 拇指向上
GESTURE_THUMB_DOWN = "THUMB_DOWN"   # 拇指向下
GESTURE_OK = "OK"                   # OK 手势
GESTURE_NONE = "NONE"               # 未检测到手势

ALL_GESTURES = (GESTURE_PALM, GESTURE_FIST, GESTURE_POINT, GESTURE_VICTORY,
                GESTURE_THUMB_UP, GESTURE_THUMB_DOWN, GESTURE_OK, GESTURE_NONE)

# --------------------------------------------------------------------------- #
# 指令 token（控制节点执行）
# --------------------------------------------------------------------------- #
CMD_TAKEOFF = "takeoff"       # 解锁 + 起飞 + 升到设定高度 + 悬停
CMD_LAND = "land"             # 降落
CMD_HOVER = "hover"           # 原地悬停（稳定）
CMD_FORWARD = "forward"       # 机体系向前
CMD_UP = "up"                 # 上升一档
CMD_DOWN = "down"             # 下降一档
CMD_YAW_LEFT = "yaw_left"     # 左偏航（键盘/话题可直接调用）
CMD_YAW_RIGHT = "yaw_right"   # 右偏航（键盘/话题可直接调用）
CMD_HOME = "home"             # 返航到原点上方
CMD_ESTOP = "estop"           # 紧急停止（按 estop_action 决定悬停/降落/锁桨）
CMD_NONE = "none"             # "没有动作"，控制节点会忽略

ALL_COMMANDS = (CMD_TAKEOFF, CMD_LAND, CMD_HOVER, CMD_FORWARD, CMD_UP,
                CMD_DOWN, CMD_YAW_LEFT, CMD_YAW_RIGHT, CMD_HOME, CMD_ESTOP)

# 手势 -> 指令：**唯一映射表**（7 个手势 -> 7 个动作）
GESTURE_TO_COMMAND = {
    GESTURE_PALM: CMD_TAKEOFF,
    GESTURE_FIST: CMD_LAND,
    GESTURE_POINT: CMD_FORWARD,
    GESTURE_VICTORY: CMD_UP,
    GESTURE_THUMB_DOWN: CMD_DOWN,
    GESTURE_THUMB_UP: CMD_HOVER,
    GESTURE_OK: CMD_ESTOP,
}

# 反向映射：指令 -> 手势（供工具/测试/单机演示使用）
COMMAND_TO_GESTURE = {v: k for k, v in GESTURE_TO_COMMAND.items()}

GESTURE_CN = {
    GESTURE_PALM: "张开手掌",
    GESTURE_FIST: "握拳",
    GESTURE_POINT: "食指指向",
    GESTURE_VICTORY: "胜利手势",
    GESTURE_THUMB_UP: "拇指向上",
    GESTURE_THUMB_DOWN: "拇指向下",
    GESTURE_OK: "OK 手势",
    GESTURE_NONE: "未检测到手势",
}

COMMAND_CN = {
    CMD_TAKEOFF: "起飞",
    CMD_LAND: "降落",
    CMD_HOVER: "悬停",
    CMD_FORWARD: "前进",
    CMD_UP: "上升",
    CMD_DOWN: "下降",
    CMD_YAW_LEFT: "左转",
    CMD_YAW_RIGHT: "右转",
    CMD_HOME: "返航",
    CMD_ESTOP: "紧急停止",
    CMD_NONE: "等待",
}


def gesture_to_command(gesture):
    """手势 token -> 指令 token；未知手势返回 ``CMD_NONE``。"""
    return GESTURE_TO_COMMAND.get(gesture, CMD_NONE)


def is_command(text):
    """判断一个字符串是不是本模块认识的指令 token。"""
    return text in ALL_COMMANDS


def gesture_cn(gesture):
    """手势 token -> 中文（仅用于日志/界面）。"""
    return GESTURE_CN.get(gesture, "未知手势(%s)" % gesture)


def command_cn(command):
    """指令 token -> 中文（仅用于日志/界面）。"""
    return COMMAND_CN.get(command, "未知指令(%s)" % command)


def describe_map():
    """返回 "手势->指令" 的可读串，供节点启动日志打印。"""
    return "，".join("%s->%s" % (gesture_cn(g), command_cn(c))
                     for g, c in GESTURE_TO_COMMAND.items())


# --------------------------------------------------------------------------- #
# 模块级自检：词表内部不能自相矛盾
# --------------------------------------------------------------------------- #
_unknown = sorted(set(GESTURE_TO_COMMAND.values()) - set(ALL_COMMANDS))
if _unknown:                                      # pragma: no cover - 防御性检查
    raise RuntimeError("GESTURE_TO_COMMAND 映射到了未知指令：%s" % _unknown)
_bad_keys = sorted(set(GESTURE_TO_COMMAND) - set(ALL_GESTURES))
if _bad_keys:                                     # pragma: no cover
    raise RuntimeError("GESTURE_TO_COMMAND 含未知手势：%s" % _bad_keys)
