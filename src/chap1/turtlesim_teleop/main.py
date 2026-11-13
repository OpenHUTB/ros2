#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""turtlesim 小海龟键盘遥控节点（第一章实验）

用方向键或 WASD 控制 turtlesim 中的小海龟前后移动、左右转向，
速度指令发布到 /turtle1/cmd_vel（geometry_msgs/Twist）。

运行方式一（前台运行，键盘输入直接可用）：
    rosrun turtlesim_teleop main.py

运行方式二（一键启动 turtlesim + 遥控节点，会弹出 xterm 窗口）：
    roslaunch turtlesim_teleop main.launch

按键说明：
    方向键 / WASD   前进、后退、左转、右转
    空格            立即停车
    + / -           加速 / 减速（步长 0.5）
    q               退出（退出前自动发布零速度，避免海龟继续运动）

兼容 Python 2.7（ROS Kinetic）与 Python 3（ROS Noetic）。
"""
from __future__ import print_function

import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

# 方向键的转义序列
ARROW_UP = '\x1b[A'
ARROW_DOWN = '\x1b[B'
ARROW_RIGHT = '\x1b[C'
ARROW_LEFT = '\x1b[D'

# 按键 -> (线速度系数, 角速度系数)
# 线速度为正表示前进，角速度为正表示左转（逆时针）
KEY_COMMANDS = {
    ARROW_UP: (1.0, 0.0),
    ARROW_DOWN: (-1.0, 0.0),
    ARROW_LEFT: (0.0, 1.0),
    ARROW_RIGHT: (0.0, -1.0),
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    ' ': (0.0, 0.0),
}

# 退出按键：q
QUIT_KEYS = ('q', 'Q')


def command_for_key(key):
    """返回按键对应的速度系数 (线速度系数, 角速度系数)，未知按键返回 None。

    该函数只做按键到速度的映射，不依赖 ROS，便于单独验证。
    """
    if key is None:
        return None
    return KEY_COMMANDS.get(key.lower() if len(key) == 1 else key)


def read_key(timeout=0.1):
    """从标准输入读取一个按键（非阻塞），没有输入时返回 None。

    方向键会返回 3 个字符的转义序列，例如上方向键返回 '\\x1b[A'。
    """
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None

    char = sys.stdin.read(1)
    if char == '\x1b':
        # 方向键的转义序列还剩下 2 个字符，尽量一次读完
        for _ in range(2):
            ready, _, _ = select.select([sys.stdin], [], [], 0.02)
            if not ready:
                break
            char += sys.stdin.read(1)
    return char


def publish_cmd(pub, linear, angular):
    """发布一条速度消息。"""
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    pub.publish(msg)


def main():
    rospy.init_node('turtlesim_teleop')

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    linear_speed = rospy.get_param('~linear_speed', 2.0)   # 线速度 m/s
    angular_speed = rospy.get_param('~angular_speed', 2.0)  # 角速度 rad/s

    # roslaunch 启动的节点拿不到终端 stdin，必须用 xterm 前台运行
    if not sys.stdin.isatty():
        rospy.logerr('当前标准输入不是终端，读不到键盘输入。'
                     '请用 rosrun 前台运行，或给 launch 节点加 launch-prefix="xterm -e"。')
        return

    # 让发布者与 turtlesim 订阅端先建立连接，避免第一条消息丢失
    rospy.sleep(1.0)

    print('小海龟键盘遥控已启动：')
    print('  方向键 / WASD  移动')
    print('  空格           停车')
    print('  + / -          加速 / 减速')
    print('  q              退出')
    print('当前速度：线速度 %.1f m/s，角速度 %.1f rad/s'
          % (linear_speed, angular_speed))

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        # cbreak 模式：按键立即生效，不需要按回车
        tty.setcbreak(sys.stdin.fileno())

        while not rospy.is_shutdown():
            key = read_key(timeout=0.1)
            if key is None:
                continue

            if key in QUIT_KEYS:
                rospy.loginfo('收到退出按键，停车退出')
                break

            if key in ('+', '='):
                linear_speed = min(linear_speed + 0.5, 5.0)
                angular_speed = min(angular_speed + 0.5, 5.0)
                print('加速：线速度 %.1f m/s，角速度 %.1f rad/s'
                      % (linear_speed, angular_speed))
                continue

            if key in ('-', '_'):
                linear_speed = max(linear_speed - 0.5, 0.5)
                angular_speed = max(angular_speed - 0.5, 0.5)
                print('减速：线速度 %.1f m/s，角速度 %.1f rad/s'
                      % (linear_speed, angular_speed))
                continue

            command = command_for_key(key)
            if command is None:
                print('未识别的按键，使用方向键、WASD、空格、+/- 或 q')
                continue

            linear = command[0] * linear_speed
            angular = command[1] * angular_speed
            publish_cmd(pub, linear, angular)

            if command == (0.0, 0.0):
                print('停车')
            else:
                print('运动：线速度 %+.1f m/s，角速度 %+.1f rad/s'
                      % (linear, angular))
    except KeyboardInterrupt:
        rospy.loginfo('收到 Ctrl+C，停车退出')
    finally:
        # 恢复终端设置，并发布零速度让海龟停下
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        publish_cmd(pub, 0.0, 0.0)


if __name__ == '__main__':
    main()
