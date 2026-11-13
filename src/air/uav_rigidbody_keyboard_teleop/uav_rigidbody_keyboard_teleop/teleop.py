#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘遥控指令映射：按键集合 -> 归一化遥控指令字典。

按键约定（与上游无人机键盘遥控模块保持一致的直观布局）：
    W / S      俯仰：W 低头前飞，S 抬头减速/后移
    A / D      滚转：A 左滚，D 右滚
    ↑ / ↓     油门：↑ 增加总推力，↓ 减小总推力
    Q / E      偏航：Q 左转，E 右转
    Space      急停：所有力矩指令归零，油门回到悬停
"""

from __future__ import annotations

from dataclasses import dataclass

# 按键 -> (通道, 方向系数)
KEY_BINDINGS = {
    "w": ("pitch", 1.0),
    "s": ("pitch", -1.0),
    "a": ("roll", -1.0),
    "d": ("roll", 1.0),
    "up": ("throttle", 1.0),
    "down": ("throttle", -1.0),
    "q": ("yaw", -1.0),
    "e": ("yaw", 1.0),
}


@dataclass
class TeleopState:
    """遥控状态：由当前按下的按键集合刷新得到。"""

    throttle: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0

    def as_cmd(self) -> dict:
        return {
            "throttle": float(self.throttle),
            "pitch": float(self.pitch),
            "roll": float(self.roll),
            "yaw": float(self.yaw),
        }


def keys_to_cmd(keys: set[str]) -> dict:
    """把当前按下的按键集合转换为遥控指令字典。

    同一通道多键同时按下时取代数和，并裁剪到 [-1, 1]。
    """
    state = TeleopState()
    for key in keys:
        binding = KEY_BINDINGS.get(key)
        if binding is None:
            continue
        channel, sign = binding
        setattr(state, channel, getattr(state, channel) + sign)
    state.throttle = _clip(state.throttle)
    state.pitch = _clip(state.pitch)
    state.roll = _clip(state.roll)
    state.yaw = _clip(state.yaw)
    return state.as_cmd()


def _clip(value: float) -> float:
    return max(-1.0, min(1.0, value))
