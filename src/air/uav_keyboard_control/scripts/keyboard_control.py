#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘控制节点：终端按键 -> /uav/cmd_vel（机体系速度指令）.

按键映射（机体系：x 前 / y 左 / z 上；速度单位为 m/s，偏航角速度为 rad/s）：
    W/S   前进 / 后退
    A/D   左移 / 右移
    R/F   上升 / 下降
    Q/E   右转 / 左转（Q=+yaw_rate，机头右转；E=-yaw_rate，机头左转）
    空格   悬停（速度清零）

各轴速度默认对齐部署侧安全限幅（max_vx=2.0, max_vy=0.5, max_yaw=0.4），
保证人工采集的动作标签幅值与部署模型输出尺度一致，避免训练标签被部署端钳位。

Linux 原始终端没有按键松开事件，这里用"状态锁存 + 超时悬停"：
- 按住按键时终端自动重复上报字符，节点持续保持该指令；
- 松开按键（或焦点丢失导致收不到字符）后，超过 hold_timeout 秒没有新按键，
  自动清零悬停，避免节点一直保持上一条指令把无人机带飞。

发布：
    /uav/cmd_vel   geometry_msgs/Twist   机体系速度（默认 30 Hz）

依赖：
    需要 carlair_ros_bridge 已启动并订阅 /uav/cmd_vel；本模块只发指令，不直连仿真器。
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
  ESC / Ctrl-C 退出
============================================================
"""


def key_to_cmd(key, forward_speed=2.0, side_speed=0.5,
               vert_speed=1.0, yaw_rate=0.4):
    """单个按键 -> 机体系速度 (vx, vy, vz, wz)；无按键返回全零；未知键返回 None.

    纯函数：不依赖 ROS / 终端，便于单元测试。各轴速度分别对应部署限幅，
    保证采集标签尺度与部署模型输出一致。
    """
    if not key:
        return (0.0, 0.0, 0.0, 0.0)
    k = key.lower()
    if k not in KEY_MAP:
        return None
    axis, sign = KEY_MAP[k]
    vx = vy = vz = wz = 0.0
    if axis == "forward":
        vx = forward_speed * sign
    elif axis == "left":
        vy = side_speed * sign
    elif axis == "up":
        vz = vert_speed * sign
    else:  # yaw
        wz = yaw_rate * sign
    return (vx, vy, vz, wz)


class RawKeyReader(object):
    """Linux 终端原始模式按键读取（不回显、不需回车）.

    roslaunch 启动的节点 stdin 指向 /dev/null（ROS 的已知限制，这也是
    teleop_twist_keyboard 通常用 rosrun 跑的原因）。为满足"支持 launch 启动"，
    这里优先直接打开控制终端 ``/dev/tty`` 读键；没有控制终端时回退到 stdin。
    用 ``with RawKeyReader() as reader:`` 保证退出/异常时恢复终端设置。
    """

    def __init__(self):
        self._close_fd = False
        try:
            self._fd = os.open("/dev/tty", os.O_RDONLY)
            self._close_fd = True
        except OSError:
            self._fd = sys.stdin.fileno()

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
        """非阻塞读一个字符；超时或无输入返回 None."""
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


class KeyboardController(object):
    """读取按键并把速度指令发布到 /uav/cmd_vel（锁存 + 超时悬停）."""

    def __init__(self):
        self.forward = float(rospy.get_param("control/forward_speed", 2.0))
        self.side = float(rospy.get_param("control/side_speed", 0.5))
        self.vert = float(rospy.get_param("control/vert_speed", 1.0))
        self.yaw = float(rospy.get_param("control/yaw_rate", 0.4))
        topic = rospy.get_param("topic/cmd_vel", "/uav/cmd_vel")
        rate_hz = float(rospy.get_param("rate/publish_hz", 30.0))
        self.hold_timeout = float(rospy.get_param("control/hold_timeout", 0.3))
        self.pub = rospy.Publisher(topic, Twist, queue_size=1)
        self.rate = rospy.Rate(rate_hz)
        # 状态锁存：记录最近一次有效指令与时间；超时无新按键则清零悬停。
        self._current = (0.0, 0.0, 0.0, 0.0)
        self._last_active = rospy.Time(0)

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

    def _cmd(self, key):
        return key_to_cmd(key, self.forward, self.side, self.vert, self.yaw)

    def step_once(self, key):
        """把一次按键转换成一个 Twist（独立成函数，便于无 ROS 的单元测试）."""
        cmd = self._cmd(key)
        if cmd is None:          # 未知键（如方向键的转义序列）-> 按悬停处理
            cmd = (0.0, 0.0, 0.0, 0.0)
        return self.build_twist(cmd)

    def feed_key(self, key, now=None):
        """并入一个读到的按键，返回当前应发布的指令（锁存 + 超时悬停）.

        - 有效键：更新指令并刷新活动时间；
        - 空格/未知键：立即清零悬停；
        - None（无新按键）：在 hold_timeout 内保持上一条指令，超时则清零悬停。
        ``now`` 用于注入当前时间，便于离线测试。
        """
        if now is None:
            now = rospy.Time.now()
        if key is None:
            if (now - self._last_active).to_sec() > self.hold_timeout:
                self._current = (0.0, 0.0, 0.0, 0.0)
            return self._current
        cmd = self._cmd(key)
        if cmd is None:
            self._current = (0.0, 0.0, 0.0, 0.0)
        else:
            self._current = cmd
        self._last_active = now
        return self._current

    # ------------------------------------------------------------------ 主循环
    def spin(self):
        print(HELP_TEXT)
        with RawKeyReader() as reader:
            while not rospy.is_shutdown():
                key = reader.get_key(timeout=0.03)
                if key in ("\x1b", "\x03"):  # ESC / Ctrl-C 退出
                    print("\n已退出键盘控制")
                    break
                cmd = self.feed_key(key, now=rospy.Time.now())
                self.pub.publish(self.build_twist(cmd))
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
