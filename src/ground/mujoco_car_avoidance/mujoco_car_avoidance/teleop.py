# -*- coding: utf-8 -*-
"""键盘遥控（交互模式）：Linux 用 termios 非阻塞读取，Windows 用 msvcrt。

只做"按键 -> (线速度, 角速度)"的映射，不含任何避障逻辑；
避障由 runner 在执行前对速度指令做限幅，因此手动模式同样撞不到障碍物。
"""

import sys
import time

from . import config

KEY_FORWARD = ("w", "W")
KEY_BACKWARD = ("s", "S")
KEY_LEFT = ("a", "A")
KEY_RIGHT = ("d", "D")
KEY_STOP = (" ",)
KEY_QUIT = ("q", "Q", "\x1b", "\x03")


class KeyboardTeleop:
    """非阻塞键盘输入。"""

    def __init__(self, forward_speed: float = config.LANE_HALF_WIDTH_M * 0.0 + 0.6,
                 backward_speed: float = 0.4,
                 turn_rate: float = 1.0,
                 hold_s: float = config.TELEOP_HOLD_S,
                 enabled: bool = True):
        self.forward_speed = float(forward_speed)
        self.backward_speed = float(backward_speed)
        self.turn_rate = float(turn_rate)
        self.hold_s = float(hold_s)
        self.enabled = bool(enabled)
        self.quit_requested = False

        self._linear = 0.0
        self._angular = 0.0
        self._last_key_time = 0.0
        self._fd = None
        self._settings = None
        self._mode = "none"

    # ------------------------------------------------------------------ 初始化
    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

    def open(self) -> None:
        if not self.enabled:
            return
        if sys.platform.startswith("linux") or sys.platform == "darwin":
            try:
                import termios
                import tty

                self._fd = sys.stdin.fileno()
                self._settings = termios.tcgetattr(self._fd)
                tty.setcbreak(self._fd)
                self._mode = "termios"
            except Exception:
                self._mode = "none"
        elif sys.platform.startswith("win"):
            try:
                import msvcrt  # noqa: F401

                self._mode = "msvcrt"
            except Exception:
                self._mode = "none"

    def close(self) -> None:
        if self._mode == "termios" and self._settings is not None and self._fd is not None:
            try:
                import termios

                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._settings)
            except Exception:
                pass
        self._mode = "none"

    # ------------------------------------------------------------------ 读取
    def _read_key(self):
        if self._mode == "termios":
            import select

            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if ready:
                return sys.stdin.read(1)
            return None
        if self._mode == "msvcrt":
            import msvcrt

            if msvcrt.kbhit():
                return msvcrt.getwch()
            return None
        return None

    def poll(self):
        """返回 (线速度, 角速度)；无按键时返回上一次的指令（保持键按住的效果）。"""
        if not self.enabled:
            return (self._linear, self._angular)

        key = self._read_key()
        now = time.perf_counter()
        if key is not None:
            self._last_key_time = now
            if key in KEY_QUIT:
                self.quit_requested = True
            elif key in KEY_FORWARD:
                self._linear, self._angular = self.forward_speed, 0.0
            elif key in KEY_BACKWARD:
                self._linear, self._angular = -self.backward_speed, 0.0
            elif key in KEY_LEFT:
                self._linear, self._angular = self.forward_speed * 0.5, self.turn_rate
            elif key in KEY_RIGHT:
                self._linear, self._angular = self.forward_speed * 0.5, -self.turn_rate
            elif key in KEY_STOP:
                self._linear, self._angular = 0.0, 0.0

        # 超时未按键则回到静止
        if now - self._last_key_time > self.hold_s:
            self._linear, self._angular = 0.0, 0.0
        return (self._linear, self._angular)

    @property
    def available(self) -> bool:
        return self._mode != "none"
