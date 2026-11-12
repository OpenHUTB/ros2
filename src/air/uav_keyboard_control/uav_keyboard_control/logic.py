"""键盘控制纯逻辑：按键映射 + 状态锁存 + 超时悬停。

不 import rclpy / geometry_msgs / 终端，可在没 ROS 的环境下单测。

坐标约定（机体系）：
    x 前 / y 左 / z 上
    W/S 前后，A/D 左右，R/F 升降，Q/E 偏航（Q=左转，+angular.z（逆时针））
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple

# 按键 -> (轴, 符号)。符号 +1 走该轴正方向。
KEY_MAP = {
    "w": ("forward", +1), "s": ("forward", -1),
    "a": ("left", +1),    "d": ("left", -1),
    "r": ("up", +1),      "f": ("up", -1),
    "q": ("yaw", +1),     "e": ("yaw", -1),
}

# 退出键
EXIT_KEYS = ("\x03",)  # 只 Ctrl-C 退出，避免方向键误触


@dataclass
class Speeds:
    forward: float = 2.0   # m/s, W/S
    side: float = 0.5      # m/s, A/D
    vert: float = 1.0      # m/s, R/F
    yaw: float = 0.4       # rad/s, Q/E


def key_to_cmd(key: Optional[str], speeds: Speeds) -> Optional[Tuple[float, float, float, float]]:
    """单个按键 -> (vx, vy, vz, wz)。未知键返回 None；空键返回全零。

    前向和侧移用不同参数，不再共用 horiz_speed。
    """
    if key is None:
        return (0.0, 0.0, 0.0, 0.0)
    k = key.lower()
    if k not in KEY_MAP:
        return None
    axis, sign = KEY_MAP[k]
    vx = speeds.forward * sign if axis == "forward" else 0.0
    vy = speeds.side * sign if axis == "left" else 0.0
    vz = speeds.vert * sign if axis == "up" else 0.0
    wz = speeds.yaw * sign if axis == "yaw" else 0.0
    return (vx, vy, vz, wz)


@dataclass
class KeyLatch:
    """状态锁存：最近一次有效按键的指令保持 hold_timeout 秒，超时自动悬停。

    用法：
        latch = KeyLatch(hold_timeout=0.3)
        latch.on_key(key, now)   # 每次读到键就调，None 也调
        cmd = latch.current(now) # 每个发布周期调，拿当前该发的 (vx,vy,vz,wz)
    """
    speeds: Speeds = field(default_factory=Speeds)
    hold_timeout: float = 0.3
    _last_cmd: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    _last_key_time: float = -1e9

    def on_key(self, key: Optional[str], now: float) -> None:
        """非阻塞读到一个字符（可能 None）。未知键/转义序列不刷新锁存。"""
        if key is None:
            return
        if key in EXIT_KEYS:
            return
        cmd = key_to_cmd(key, self.speeds)
        if cmd is None:
            # 方向键转义序列等，按"无有效键"处理，不刷新锁存
            return
        self._last_cmd = cmd
        self._last_key_time = now

    def current(self, now: float) -> Tuple[float, float, float, float]:
        """当前时刻该发的指令：距上次有效键在 hold_timeout 内就保持，否则全零悬停。"""
        if now - self._last_key_time <= self.hold_timeout:
            return self._last_cmd
        return (0.0, 0.0, 0.0, 0.0)
