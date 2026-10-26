# -*- coding: utf-8 -*-
"""
keyboard.py —— 跨平台非阻塞按键读取（Linux termios / Windows msvcrt）。

供 standalone 采集（collect_data.py）与交互入口（main.py）复用；
ROS2 遥控节点不使用本工具（节点内部自行打开 /dev/tty）。
"""

import os
import sys


class KeyReader:
    """Linux 用 termios raw 模式；Windows 用 msvcrt 非阻塞读取。"""

    def __init__(self):
        self._platform = sys.platform
        self._fd = None
        self._old = None
        if self._platform.startswith("linux"):
            import termios
            import tty
            try:
                self._fd = os.open("/dev/tty", os.O_RDONLY)
            except OSError:
                self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setraw(self._fd)

    def get_key(self, timeout=0.03):
        if self._platform.startswith("linux"):
            import select
            import termios
            import tty
            r, _, _ = select.select([self._fd], [], [], timeout)
            if not r:
                return None
            ch = os.read(self._fd, 1)
            if not ch:
                return None
            try:
                return ch.decode("utf-8")
            except UnicodeDecodeError:
                return None
        if self._platform.startswith("win"):
            import msvcrt
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch == '\x00' or ch == '\xe0':
                    ch2 = msvcrt.getwch()
                    return {'H': 'up', 'P': 'down', 'K': 'left', 'M': 'right'}.get(ch2, ch2)
                return ch
            return None
        return None

    def close(self):
        if self._fd is not None and self._old is not None:
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
            if self._fd != sys.stdin.fileno():
                os.close(self._fd)
