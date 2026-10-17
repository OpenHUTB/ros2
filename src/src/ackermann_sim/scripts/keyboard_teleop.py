#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
keyboard_teleop.py — 键盘遥控阿克曼小车 (ROS1 Noetic)

按键:
  w / ↑   前进
  s / ↓   刹车/倒车
  a / ←   左转
  d / →   右转
  空格    急停
  q       退出

发布:
  /cmd_vel  (geometry_msgs/Twist)
      linear.x  = 目标速度 v (m/s)
      angular.z = 前轮转角 delta (rad)
"""

import sys
import math
import rospy
import tty
import termios
import select
from geometry_msgs.msg import Twist

# 可调参数
TARGET_SPEED = 1.5    # m/s
MAX_STEER = 0.45      # rad (~25.8°)


def get_key(timeout):
    """读一个按键, 超时返回 None"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            ch = sys.stdin.read(1)
            # 方向键是 3 字节前缀
            if ch == "\x1b":
                ch2 = sys.stdin.read(1)
                if ch2 == "[":
                    ch3 = sys.stdin.read(1)
                    return {"A": "up", "B": "down", "C": "right", "D": "left"}.get(ch3, None)
                return None
            return ch
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rospy.init_node("ackermann_teleop")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    v = 0.0
    delta = 0.0

    print("""
阿克曼小车键盘遥控
  w/↑  前进   s/↓  刹车/倒车
  a/←  左转   d/→  右转
  空格 急停    q  退出
""")

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        key = get_key(0.05)
        if key == "q":
            break
        elif key == "w" or key == "up":
            v = TARGET_SPEED
        elif key == "s" or key == "down":
            v = -TARGET_SPEED
        elif key == "a" or key == "left":
            delta = MAX_STEER
        elif key == "d" or key == "right":
            delta = -MAX_STEER
        elif key == " ":
            v = 0.0
            delta = 0.0
        else:
            # 松开按键后, 转向回正, 速度缓慢减速
            delta *= 0.8
            v *= 0.9

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = delta
        pub.publish(msg)

        rospy.loginfo_throttle(0.5, "v=%.2f m/s, delta=%.1f deg" %
                               (v, math.degrees(delta)))
        rate.sleep()

    # 退出时发一次零速
    pub.publish(Twist())
    print("teleop exited.")


if __name__ == "__main__":
    main()
