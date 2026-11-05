#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""keymap.py —— 键盘按键到推扰方向角的映射（无 ROS 依赖，便于单元测试）
====================================================================

坐标约定：0° = +x 前向，+y 为机器人左侧（右手系），
故 45° = 左前、135° = 左后、225° = 右后、315° = 右前。

本模块从 ``push_teleop_node`` 中拆出：遥控节点在模块级 import rclpy，
在无 ROS 环境（如 ``test/test_standalone.py``）下无法导入；把纯映射逻辑
独立出来既可单测，也能覆盖方向键这类容易写错大小写的分支。
"""

from __future__ import annotations

# 按键 -> 方向角(度)。方向键由 ANSI 转义序列解析得到，键名保持大写 KEY_*。
KEY_DIRECTIONS = {
    'w': 0.0,          # 前
    's': 180.0,        # 后
    'a': 90.0,         # 左
    'd': 270.0,        # 右
    'q': 45.0,         # 左前
    'e': 225.0,        # 右后
    'KEY_UP': 0.0,
    'KEY_DOWN': 180.0,
    'KEY_LEFT': 90.0,
    'KEY_RIGHT': 270.0,
}

# 方向角 -> 中文说明（日志与测试断言共用）
DIRECTION_LABELS = {
    0.0: '前',
    45.0: '左前',
    90.0: '左',
    135.0: '左后',
    180.0: '后',
    225.0: '右后',
    270.0: '右',
    315.0: '右前',
}

# 方向键键名（供测试与文档遍历）
ARROW_KEYS = ('KEY_UP', 'KEY_DOWN', 'KEY_LEFT', 'KEY_RIGHT')


def normalize_key(key):
    """把原始按键统一成查表用的键名。

    方向键（``KEY_UP`` 等）必须保持大写；普通字母键转小写，
    使大写锁定时输入的 ``W`` 与 ``w`` 等价。
    """
    if not key:
        return ''
    return key if key.startswith('KEY_') else key.lower()


def key_to_direction(key):
    """返回按键对应的方向角（度）；未映射的按键返回 ``None``。"""
    return KEY_DIRECTIONS.get(normalize_key(key))


def direction_label(deg):
    """返回方向角的中文说明，未知角度返回 ``'?'``。"""
    return DIRECTION_LABELS.get(float(deg), '?')
