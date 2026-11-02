#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘遥控（termios，无 GUI 依赖）。

按键
----
    w / s   前进 / 后退
    a / d   左转 / 右转
    空格    立即停止
    q       退出

为什么不用 teleop_twist_keyboard：
    那个包需要额外安装（虚拟机里不一定有），而且按键是"逐步加速"的，
    做定位实验时不好控制恒定速度。这里直接用 termios 改终端为原始模式，
    按住键持续发送速度指令，松开（停止按键）后由节点超时自动停车。

用法：
    rosrun lidar_mcl_localization teleop.py
    rosrun lidar_mcl_localization teleop.py _speed:=0.4 _turn:=0.8
"""

import os
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


def get_key(timeout=0.1):
    """非阻塞读一个字符；没有按键则返回空串。"""
    dr, _dw, _de = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return ""


def main():
    rospy.init_node("mcl_teleop")
    speed = float(rospy.get_param("~speed", 0.35))
    turn = float(rospy.get_param("~turn", 0.80))
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    print("=" * 56)
    print("键盘遥控：w/s 前后，a/d 转向，空格停车，q 退出")
    print("  线速度 %.2f m/s   角速度 %.2f rad/s" % (speed, turn))
    print("=" * 56)

    old = termios.tcgetattr(sys.stdin)
    cmd = Twist()
    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.is_shutdown():
            key = get_key(0.1)
            if not key:
                # 没有按键：保持上一帧指令不变（roslaunch 里 Ctrl+C 也不会突然停）
                pub.publish(cmd)
                continue

            if key == "q":
                break
            if key == " ":
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            elif key in ("w", "W"):
                cmd.linear.x = speed
            elif key in ("s", "S"):
                cmd.linear.x = -speed
            elif key in ("a", "A"):
                cmd.angular.z = turn
            elif key in ("d", "D"):
                cmd.angular.z = -turn
            pub.publish(cmd)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        pub.publish(Twist())
        print("\n已停车，退出。")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(0)
