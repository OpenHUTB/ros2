#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘控制节点：终端按键 -> /uav/cmd_vel（机体系速度指令）.

按键映射（机体系：x 前 / y 左 / z 上；速度单位为 m/s，偏航角速度为 rad/s）：
    W/S   前进 / 后退
    A/D   左移 / 右移
    R/F   上升 / 下降
    Q/E   右转 / 左转（Q=+yaw_rate 机头右转，遵循 AirSim 机体系 yaw_rate 约定）
    其它键 / 松开    悬停（速度清零）
    ESC 或 Ctrl-C     退出

发布：
    /uav/cmd_vel   geometry_msgs/Twist   机体系速度指令（默认 30 Hz）

依赖：
    需要 carlair_ros_bridge 已启动并订阅 /uav/cmd_vel；本模块只发指令，不直连仿真器。
    这正是任务①"无人机仿真 + 键盘控制"的分工：仿真/坐标换算在桥接层，键盘只管下发。
"""
from __future__ import annotations

import os
import select
import sys

import rospy
from geometry_msgs.msg import Twist

# 按键 -> (轴, 符号)。符号 +1 表示该轴正方向，-1 表示负方向。
KEY_MAP = {
    "w": ("forward", +1), "s": ("forward", -1),
    "a": ("left", +1), "d": ("left", -1),
    "r": ("up", +1), "f": ("up", -1),
    "q": ("yaw", +1), "e": ("yaw", -1),
}

HELP_TEXT = """
============================================================
  无人机键盘控制（机体系速度，发布到 /uav/cmd_vel）
------------------------------------------------------------
  W / S     前进 / 后退
  A / D     左移 / 右移
  R / F     上升 / 下降
  Q / E     右转 / 左转
  松开按键   悬停
  ESC       退出
============================================================
"""


def key_to_cmd(key, horiz_speed=2.0, vert_speed=1.0, yaw_rate=1.0):
    """单个按键 -> 机体系速度 (vx, vy, vz, wz)；无按键返回全零；未知键返回 None.

    纯函数：不依赖 ROS / 终端，便于单元测试。
    """
    if not key:
        return (0.0, 0.0, 0.0, 0.0)
    k = key.lower()
    if k not in KEY_MAP:
        return None
    axis, sign = KEY_MAP[k]
    vx = horiz_speed * sign if axis == "forward" else 0.0
    vy = horiz_speed * sign if axis == "left" else 0.0
    vz = vert_speed * sign if axis == "up" else 0.0
    wz = yaw_rate * sign if axis == "yaw" else 0.0
    return (vx, vy, vz, wz)


class RawKeyReader(object):
    """Linux 终端原始模式按键读取（不回显、不需回车）.

    用 ``with RawKeyReader() as reader:`` 保证退出时恢复终端设置，
    即使 Ctrl-C 抛 KeyboardInterrupt 也会被 __exit__ 恢复。
    """

    def __init__(self, stream=None):
        self.stream = stream if stream is not None else sys.stdin
        self._fd = self.stream.fileno()
        self._old = None

    def __enter__(self):
        import termios
        import tty
        self._old = termios.tcgetattr(self._fd)
        tty.setraw(self._fd)
        return self

    def __exit__(self, *exc):
        import termios
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def get_key(self, timeout=0.05):
        """非阻塞读一个字符；超时或无输入返回 None."""
        r, _, _ = select.select([self.stream], [], [], timeout)
        if not r:
            return None
        ch = os.read(self._fd, 1)
        if not ch:
            return None
        try:
            return ch.decode("utf-8")
        except UnicodeDecodeError:
            return None


class KeyboardController(object):
    """读取按键并把速度指令发布到 /uav/cmd_vel."""

    def __init__(self):
        self.horiz = float(rospy.get_param("control/horiz_speed", 2.0))
        self.vert = float(rospy.get_param("control/vert_speed", 1.0))
        self.yaw = float(rospy.get_param("control/yaw_rate", 1.0))
        topic = rospy.get_param("topic/cmd_vel", "/uav/cmd_vel")
        rate_hz = float(rospy.get_param("rate/publish_hz", 30.0))
        self.pub = rospy.Publisher(topic, Twist, queue_size=1)
        self.rate = rospy.Rate(rate_hz)

    # ------------------------------------------------------------------ 核心
    def build_twist(self, cmd):
        """(vx, vy, vz, wz) -> geometry_msgs/Twist."""
        vx, vy, vz, wz = cmd
        t = Twist()
        t.linear.x = vx
        t.linear.y = vy
        t.linear.z = vz
        t.angular.z = wz
        return t

    def step_once(self, key):
        """把一次按键转换成一个 Twist 消息（独立成函数，便于无 ROS 的单元测试）."""
        cmd = key_to_cmd(key, self.horiz, self.vert, self.yaw)
        if cmd is None:          # 未知键（如方向键的转义序列）-> 按松开处理，悬停
            cmd = (0.0, 0.0, 0.0, 0.0)
        return self.build_twist(cmd)

    # ------------------------------------------------------------------ 主循环
    def spin(self):
        print(HELP_TEXT)
        with RawKeyReader() as reader:
            while not rospy.is_shutdown():
                key = reader.get_key(timeout=0.03)
                if key == "\x1b":            # ESC 退出
                    print("\n已退出键盘控制")
                    break
                self.pub.publish(self.step_once(key))
                self.rate.sleep()


def main():
    rospy.init_node("uav_keyboard_control", anonymous=False)
    node = KeyboardController()
    try:
        node.spin()
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，已退出")


if __name__ == "__main__":
    main()
