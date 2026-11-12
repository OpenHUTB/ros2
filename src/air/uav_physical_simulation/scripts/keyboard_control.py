#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无人机键盘控制的按键映射与 ROS 发布节点。

对应课程任务（1）的「键盘控制」部分。

设计要点
--------
1. ``key_to_cmd`` 是**纯函数**：不依赖 ROS、不依赖终端，可在无仿真环境的
   机器上单元测试。独立模式与 ROS 模式共同调用它，保证两条路径行为一致。

2. ``termios`` / ``tty`` 是 POSIX 专属模块，Windows 上没有。因此只在
   ``RawKeyReader`` 内部导入，写在文件顶层会导致整个包在 Windows 上无法导入。

3. 节点以固定周期持续发布（含零速度），而不是「按键触发一次」。桥接层有
   「超过 cmd_timeout 无指令则自动悬停」的保护，稀疏发布会触发该保护，
   表现为间断性掉高度。
"""

from __future__ import annotations

import os
import select
import sys

# 机体系约定：x 前 / y 左 / z 上
KEY_FORWARD, KEY_BACKWARD = "W", "S"
KEY_LEFT, KEY_RIGHT = "A", "D"
KEY_UP, KEY_DOWN = "R", "F"
KEY_YAW_RIGHT, KEY_YAW_LEFT = "Q", "E"

VALID_KEYS = {
    KEY_FORWARD, KEY_BACKWARD,
    KEY_LEFT, KEY_RIGHT,
    KEY_UP, KEY_DOWN,
    KEY_YAW_RIGHT, KEY_YAW_LEFT,
}


def key_to_cmd(keys, horiz=2.0, vert=1.0, yaw=1.0):
    """按键状态 → 机体系速度指令 ``(vx, vy, vz, wz)``。

    Parameters
    ----------
    keys : dict or set
        按键集合。传 ``set``/``list`` 或 ``dict``（值为真表示按下）均可。
    horiz, vert, yaw : float
        水平速度、垂直速度、偏航角速度的上限。

    Returns
    -------
    tuple of float
        ``(vx, vy, vz, wz)``；松开所有键时返回全零（悬停）。
    """
    if isinstance(keys, dict):
        pressed = {k for k, v in keys.items() if v}
    else:
        pressed = set(keys or ())

    def on(key):
        return 1.0 if key in pressed else 0.0

    vx = horiz * (on(KEY_FORWARD) - on(KEY_BACKWARD))
    vy = horiz * (on(KEY_LEFT) - on(KEY_RIGHT))
    vz = vert * (on(KEY_UP) - on(KEY_DOWN))
    wz = yaw * (on(KEY_YAW_RIGHT) - on(KEY_YAW_LEFT))
    return vx, vy, vz, wz


def normalize_key(ch):
    """把终端读到的字符规范化为大写按键符号（不支持则返回空串）。"""
    if not ch:
        return ""
    return ch.upper() if ch.upper() in VALID_KEYS or ch == "\x1b" else ""


class RawKeyReader:
    """终端 raw 模式单键读取器。

    Linux 终端默认行缓冲（要按回车才响应）。用 ``termios`` + ``tty`` 进入
    raw 模式即可单键即时响应。用 ``with`` 语句保证异常退出时也能恢复终端设置，
    不会把用户的终端卡在 raw 模式。
    """

    def __enter__(self):
        # POSIX 专属模块，只在函数内导入以免 Windows 上导入整个包即失败。
        import termios
        import tty

        self._termios = termios
        self._tty = tty
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setraw(self._fd)
        return self

    def __exit__(self, *exc):
        try:
            self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old)
        except Exception:
            # 恢复失败不应掩盖原始异常
            pass
        return False

    def get_key(self, timeout=0.03):
        """非阻塞读一个字符；超时返回空串。"""
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if not r:
            return ""
        return sys.stdin.read(1)


def publish_loop(pub, twist_cls, horiz, vert, yaw, dt, stop_flag, log=None):
    """固定周期发布循环（独立模式与 ROS 模式共用）。

    ``stop_flag`` 是形如 ``lambda: bool`` 的可调用对象，返回 True 时退出。
    """
    import rospy

    with RawKeyReader() as reader:
        held = set()
        while not stop_flag():
            ch = reader.get_key(timeout=dt * 0.5)
            if ch:
                if ch == "\x1b":            # ESC
                    held.clear()
                    if log:
                        log("[退出] 收到 ESC")
                    break
                k = normalize_key(ch)
                if k in VALID_KEYS:
                    # 单键点动：本次读到的键即为当前按下的键
                    held = {k}
                else:
                    held = set()
            else:
                held = set()               # 无按键输入 → 松开 → 悬停

            vx, vy, vz, wz = key_to_cmd(held, horiz, vert, yaw)
            tw = twist_cls()
            tw.linear.x, tw.linear.y, tw.linear.z = vx, vy, vz
            tw.angular.z = wz
            pub.publish(tw)

            if log and (vx or vy or vz or wz):
                log("[控制] 按键={} → vx={:+.2f} vy={:+.2f} vz={:+.2f} wz={:+.2f}"
                    .format("".join(sorted(held)), vx, vy, vz, wz))

            rospy.sleep(dt)

    # 退出时发零速度，确保悬停
    tw = twist_cls()
    pub.publish(tw)
