# -*- coding: utf-8 -*-
"""跨平台非阻塞键盘读取（Linux termios / Windows msvcrt）。"""

import os
import select
import sys

if os.name == "nt":
    import msvcrt
else:
    import termios
    import tty


class KeyReader:
    def __init__(self, stream=None):
        self.stream = stream or sys.stdin
        self._fd = None
        self._saved = None
        self._win = (os.name == "nt")

    def __enter__(self):
        if not self._win:
            self._fd = self.stream.fileno()
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)      # cbreak 保留 ISIG，Ctrl-C 仍可中断
        return self

    def __exit__(self, *exc):
        self.restore()
        return False

    def restore(self):
        if not self._win and self._saved is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
            except Exception:
                pass
            self._saved = None

    def poll(self, timeout=0.0):
        if self._win:
            return self._poll_windows()
        return self._poll_posix(timeout)

    def _poll_posix(self, timeout):
        if self._fd is None:
            self._fd = self.stream.fileno()
        ready, _, _ = select.select([self._fd], [], [], timeout)
        if not ready:
            return []
        return [chr(b) for b in os.read(self._fd, 64)]

    def _poll_windows(self):
        out = []
        while msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):
                if msvcrt.kbhit():
                    msvcrt.getwch()
                continue
            out.append(ch)
        return out


def drain_stdin():
    if os.name == "nt":
        import msvcrt
        while msvcrt.kbhit():
            msvcrt.getwch()
        return
    try:
        termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    except Exception:
        pass
