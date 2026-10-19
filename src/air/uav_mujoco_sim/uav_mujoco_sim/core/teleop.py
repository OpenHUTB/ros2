#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘遥控按键映射，与上游 ``uav_keyboard_control`` 一致（ROS 节点与独立演示共用）。

W/S、A/D、R/F、Q/E 依次对应 ±linear.x、±linear.y、±linear.z、±angular.z；
其它键或无按键 = 悬停（全零）。完整按键说明见 ``key_help()``。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass
class TeleopConfig:
    horiz_speed: float = 1.0      # m/s
    vert_speed: float = 0.6       # m/s
    yaw_rate: float = 0.6         # rad/s

    def to_dict(self) -> Dict[str, float]:
        return {"horiz_speed": self.horiz_speed,
                "vert_speed": self.vert_speed,
                "yaw_rate": self.yaw_rate}


#: 按键 → (vx, vy, vz, yaw_rate) 符号组合，幅值由 TeleopConfig 提供
KEY_SIGNS: Dict[str, Tuple[float, float, float, float]] = {
    "w": (1, 0, 0, 0),
    "s": (-1, 0, 0, 0),
    "a": (0, 1, 0, 0),
    "d": (0, -1, 0, 0),
    "r": (0, 0, 1, 0),
    "f": (0, 0, -1, 0),
    "q": (0, 0, 0, 1),      # 机头右转
    "e": (0, 0, 0, -1),     # 机头左转
}

#: 退出键
EXIT_KEYS = ("\x1b", "\x03")   # ESC / Ctrl-C


def command_from_keys(keys, cfg: TeleopConfig) -> Tuple[float, float, float, float]:
    """由按下的按键集合算出 (vx, vy, vz, yaw_rate)；同轴反向键互相抵消（W+S = 悬停）。"""
    sx = sy = sz = sw = 0.0
    for k in keys:
        k = (k or "").lower()
        if k in KEY_SIGNS:
            a, b, c, d = KEY_SIGNS[k]
            sx += a
            sy += b
            sz += c
            sw += d
    return (sx * cfg.horiz_speed, sy * cfg.horiz_speed,
            sz * cfg.vert_speed, sw * cfg.yaw_rate)


def key_help() -> str:
    """返回按键说明文本（终端与文档共用）。"""
    return (
        "  W / S   前进 / 后退      A / D   左移 / 右移\n"
        "  R / F   上升 / 下降      Q / E   右转 / 左转\n"
        "  松开所有键 = 悬停        ESC 或 Ctrl-C = 退出"
    )
