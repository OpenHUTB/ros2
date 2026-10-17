#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人形机器人键盘遥控
w/up 前进  s/down 后退  a/left 左转  d/right 右转  空格急停  q退出
发布 /cmd_vel
"""
import sys
import rospy
import tty
import termios
import select
from geometry_msgs.msg import Twist

TARGET_SPEED = 0.8
TARGET_TURN = 0.8


def get_key(timeout):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            ch = sys.stdin.read(1)
            if ch == "\x1b":
                ch2 = sys.stdin.read(1)
                if ch2 == "[":
                    ch3 = sys.stdin.read(1)
                    return {"A": "up", "B": "down",
                            "C": "right", "D": "left"}.get(ch3)
                return None
            return ch
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rospy.init_node("humanoid_teleop")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    v = 0.0
    omega = 0.0
    print("w前进 s后退 a左转 d右转 空格急停 q退出")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        key = get_key(0.05)
        if key == "q":
            break
        elif key in ("w", "up"):
            v = TARGET_SPEED
        elif key in ("s", "down"):
            v = -TARGET_SPEED
        elif key in ("a", "left"):
            omega = TARGET_TURN
        elif key in ("d", "right"):
            omega = -TARGET_TURN
        elif key == " ":
            v = 0.0
            omega = 0.0
        else:
            v *= 0.9
            omega *= 0.8
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = omega
        pub.publish(msg)
        rate.sleep()
    pub.publish(Twist())


if __name__ == "__main__":
    main()
