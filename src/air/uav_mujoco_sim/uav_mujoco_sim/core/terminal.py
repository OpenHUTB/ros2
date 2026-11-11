#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""终端原始按键读取：纯标准库实现（不依赖 ROS 与第三方库），供 ROS 键盘节点与独立演示共用。

取键 fd 优先 ``/dev/tty``（与 stdin 是否重定向无关），退到 stdin。终端只把输入交给
**前台**进程组，``ros2 launch`` 拉起的子进程不在其中，此时 ``is_tty`` 为假、``error``
记下原因，由调用方给出提示。
"""
from __future__ import annotations

import os
import select
import signal
import sys
from typing import List

#: 控制终端
TTY_PATH = "/dev/tty"


class RawKeyReader:
    """把终端切成非规范、非回显模式，逐个读出按键；用完需 ``restore()``。

    ``is_tty`` 为 False 时 ``read()`` 恒返回空列表，``source``/``error`` 供日志与排查。
    """

    def __init__(self, warn=None, tty_path: str = ""):
        # 默认值在函数体里解析：参数默认在定义时求值，模块级 TTY_PATH 改动不生效，
        # 且"拿不到终端"的用例会去开真的 /dev/tty、把调用者的终端切成 cbreak。
        tty_path = tty_path or TTY_PATH
        self._fd = None
        self._own_fd = False
        self._termios = None
        self._old = None
        self.is_tty = False
        #: 实际取键的来源
        self.source = "none"
        #: 不可用的原因
        self.error = ""

        if os.name == "posix" and tty_path:
            try:
                self._fd = os.open(tty_path, os.O_RDONLY | os.O_NONBLOCK)
                self._own_fd = True
                self.source = tty_path
            except OSError:
                self._fd = None
        if self._fd is None:
            try:
                if sys.stdin.isatty():
                    self._fd = sys.stdin.fileno()
                    self.source = "stdin"
            except (ValueError, OSError):
                self._fd = None
        if self._fd is None:
            self.source = "none"
            self.error = "no tty"
            return
        if not self._is_foreground(self._fd):
            self.error = "not foreground: tty pgrp != own pgrp"
            self.source = "none"
            self._release()
            return

        try:
            import termios
            import tty
            self._termios = termios
            self._old = termios.tcgetattr(self._fd)
            # 只屏蔽这一步：tcsetattr 给后台进程组发 SIGTTOU，默认动作是停住进程，
            # Python 层看不到异常。
            old_handler = signal.signal(signal.SIGTTOU, signal.SIG_IGN)
            try:
                tty.setcbreak(self._fd)
            finally:
                signal.signal(signal.SIGTTOU, old_handler)
        except Exception as e:                              # noqa: BLE001
            if warn:
                warn("无法进入原始终端模式：%s" % e)
            self.error = "%s: %s" % (type(e).__name__, e)
            self.source = "none"
            self._release()
            return
        self.is_tty = True

    @staticmethod
    def _is_foreground(fd: int) -> bool:
        """本进程能否从这个终端取输入。

        内核只把输入交给前台进程组；前台进程组未设置时放行（刚 open 的 pty），
        因此只在"已设置且不是自己"时判否。取不到也按放行处理。
        """
        if os.name != "posix":
            return True
        try:
            fg = os.tcgetpgrp(fd)
        except OSError:
            return True
        return fg <= 0 or fg == os.getpgrp()

    def _release(self) -> None:
        if self._own_fd and self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
        self._fd = None
        self._own_fd = False

    def read(self) -> List[str]:
        """非阻塞读出当前已到达的按键。"""
        if not self.is_tty:
            return []
        out: List[str] = []
        try:
            while select.select([self._fd], [], [], 0)[0]:
                ch = os.read(self._fd, 1).decode("utf-8", "replace")
                if not ch:
                    break
                out.append(ch)
        except (OSError, ValueError) as e:
            self.is_tty = False
            self.error = "%s: %s" % (type(e).__name__, e)
        return out

    def restore(self) -> None:
        if self._old is not None and self._termios is not None \
                and self._fd is not None:
            try:
                self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN,
                                        self._old)
            except Exception:                              # noqa: BLE001
                pass
            self._old = None
        self._release()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.restore()
        return False
