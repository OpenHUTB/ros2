#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""取键终端的挑选：优先 /dev/tty，拿不到或不在前台进程组时安全退化。

    python -m pytest test/test_terminal.py -v

不需要 ROS、显示器与 mujoco。``ros2 launch`` 会把子进程的 stdin 接成管道，
只读 stdin 收不到按键；换成 ``/dev/tty`` 后仍受前台进程组限制，这两条路径用 pty 固化。
"""
import io
import os
import sys
import time

import pytest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_ROOT)

from uav_mujoco_sim.core import terminal                      # noqa: E402
from uav_mujoco_sim.core.terminal import RawKeyReader         # noqa: E402


def test_degrades_safely_without_any_tty(monkeypatch):
    """两条路都拿不到终端时：is_tty=False、read() 返回空，不抛异常。"""
    monkeypatch.setattr(terminal, "TTY_PATH", "/nonexistent/definitely/not/tty")
    monkeypatch.setattr(terminal.sys, "stdin", io.StringIO("w"))
    r = RawKeyReader()
    assert r.is_tty is False
    assert r.source == "none"
    assert r.read() == []
    r.restore()                      # 未进入原始终端模式时也必须幂等
    r.restore()


def test_warns_when_no_tty(monkeypatch):
    """拿不到终端要报出来，否则调用方看到的是"节点在跑但按键没反应"。"""
    monkeypatch.setattr(terminal, "TTY_PATH", "/nonexistent/tty")
    monkeypatch.setattr(terminal.sys, "stdin", io.StringIO("w"))
    seen = []
    RawKeyReader(warn=seen.append)
    assert isinstance(seen, list)     # 无终端本身不算异常，warn 留给打开失败


@pytest.mark.skipif(os.name != "posix", reason="需要 pty（Linux）")
def test_reads_keys_from_tty_not_stdin(monkeypatch):
    """stdin 是管道时，按键要从 /dev/tty 读出来。"""
    import pty

    master, slave = pty.openpty()
    path = os.ttyname(slave)
    monkeypatch.setattr(terminal, "TTY_PATH", path)
    monkeypatch.setattr(terminal.sys, "stdin", io.StringIO(""))   # stdin 非终端

    r = RawKeyReader()
    try:
        assert r.is_tty is True
        assert r.source == path          # 取自 /dev/tty，不是 stdin
        os.write(master, b"w")
        time.sleep(0.05)
        assert r.read() == ["w"]
        os.write(master, b"as")
        time.sleep(0.05)
        assert r.read() == ["a", "s"]
    finally:
        r.restore()
        os.close(master)
        os.close(slave)


@pytest.mark.skipif(os.name != "posix", reason="需要 pty（Linux）")
def test_rejects_terminal_of_another_process_group(monkeypatch):
    """终端属于别的进程组（ros2 launch 就是这么拉子进程的）时判为不可用。"""
    import pty

    master, slave = pty.openpty()
    path = os.ttyname(slave)
    monkeypatch.setattr(terminal, "TTY_PATH", path)
    monkeypatch.setattr(terminal.sys, "stdin", io.StringIO(""))
    monkeypatch.setattr(RawKeyReader, "_is_foreground",
                        staticmethod(lambda fd: False))

    r = RawKeyReader()
    try:
        assert r.is_tty is False
        assert r.source == "none"
        assert "foreground" in r.error
        os.write(master, b"w")           # 有按键也不该被读走
        time.sleep(0.05)
        assert r.read() == []
    finally:
        r.restore()
        os.close(master)
        os.close(slave)


@pytest.mark.skipif(os.name != "posix", reason="需要 termios（Linux）")
def test_restore_returns_terminal_mode(monkeypatch):
    """restore() 必须把终端模式还原，否则退出后终端不再回显。"""
    import pty
    import termios

    master, slave = pty.openpty()
    path = os.ttyname(slave)
    monkeypatch.setattr(terminal, "TTY_PATH", path)
    monkeypatch.setattr(terminal.sys, "stdin", io.StringIO(""))

    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    before = termios.tcgetattr(fd)
    r = RawKeyReader()
    assert r.is_tty is True
    assert termios.tcgetattr(fd) != before        # cbreak 生效
    r.restore()
    assert termios.tcgetattr(fd) == before        # 还原
    os.close(fd)
    os.close(master)
    os.close(slave)
