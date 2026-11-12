#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘控制 ROS2 节点：终端按键 -> /uav/cmd_vel。

- 状态锁存 + 超时悬停：松开按键后保持 hold_timeout 秒，超时自动发零速。
- 速度参数全部从 ROS 参数读（见 config/keyboard.yaml）。
- 终端读键走 /dev/tty（Linux raw 模式），Windows 不支持（ROS 本来就在 Linux 跑）。
"""
from __future__ import annotations

import os
import select
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from . import logic as L

HELP_TEXT = """
============================================================
  无人机键盘控制（机体系速度，发布到 /uav/cmd_vel）
------------------------------------------------------------
  W / S     前进 / 后退
  A / D     左移 / 右移
  R / F     上升 / 下降
  Q / E     左转 / 右转
  松开按键   保持 0.3s 后自动悬停
  Ctrl-C 退出
============================================================
"""


class RawKeyReader:
    """Linux raw 模式读键，优先 /dev/tty，回退 stdin。"""

    def __init__(self):
        self._close_fd = False
        try:
            self._fd = os.open("/dev/tty", os.O_RDONLY)
            self._close_fd = True
        except OSError:
            self._fd = sys.stdin.fileno()
        if not os.isatty(self._fd):
            raise SystemExit(
                "uav_keyboard_control：需要可交互的终端才能读键盘，"
                "请在前台终端直接运行，不要用 nohup / systemd / 管道。"
            )

    def __enter__(self):
        import termios
        import tty
        self._old = termios.tcgetattr(self._fd)
        tty.setraw(self._fd)
        return self

    def __exit__(self, *exc):
        import termios
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
        if self._close_fd:
            os.close(self._fd)

    def get_key(self, timeout=0.05):
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


class KeyboardNode(Node):
    def __init__(self):
        super().__init__("uav_keyboard_control")
        self.declare_parameter("forward_speed", 2.0)
        self.declare_parameter("side_speed", 0.5)
        self.declare_parameter("vert_speed", 1.0)
        self.declare_parameter("yaw_rate", 0.4)
        self.declare_parameter("hold_timeout", 0.3)
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("cmd_vel_topic", "/uav/cmd_vel")

        speeds = L.Speeds(
            forward=float(self.get_parameter("forward_speed").value),
            side=float(self.get_parameter("side_speed").value),
            vert=float(self.get_parameter("vert_speed").value),
            yaw=float(self.get_parameter("yaw_rate").value),
        )
        self.latch = L.KeyLatch(
            speeds=speeds,
            hold_timeout=float(self.get_parameter("hold_timeout").value),
        )
        self._pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self._period = 1.0 / max(float(self.get_parameter("publish_hz").value), 1.0)

    def publish_now(self, now: float) -> None:
        vx, vy, vz, wz = self.latch.current(now)
        t = Twist()
        t.linear.x = vx
        t.linear.y = vy
        t.linear.z = vz
        t.angular.z = wz
        self._pub.publish(t)


def main():
    rclpy.init()
    node = KeyboardNode()
    print(HELP_TEXT)
    try:
        with RawKeyReader() as reader:
            while rclpy.ok():
                t0 = time.monotonic()
                key = reader.get_key(timeout=0.03)
                if key in L.EXIT_KEYS:
                    print("\n退出键盘控制")
                    break
                node.latch.on_key(key, t0)
                node.publish_now(t0)
                # 按发布周期节流
                dt = time.monotonic() - t0
                if dt < node._period:
                    time.sleep(node._period - dt)
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C")
    finally:
        # 退出前补发一条零速，避免桥接层超时前还带着速度
        node._pub.publish(Twist())
        time.sleep(0.05)  # 等 DDS 把零速消息发出去再销毁节点
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
