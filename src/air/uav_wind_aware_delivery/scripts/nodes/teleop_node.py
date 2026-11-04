#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘手动控制：W/S 前后、A/D 左右、R/F 升降、Q 退出。"""
import os
import select
import sys
import termios
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _bootstrap  # noqa: E402,F401

import rospy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402


def get_key(timeout=0.05):
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return ""


def main():
    rospy.init_node("uav_teleop")
    pub = rospy.Publisher("/uav/setpoint", PoseStamped, queue_size=1)
    x, y, z = 0.0, 0.0, 2.5
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print("W/S forward/back, A/D left/right, R/F up/down, Q quit")
    try:
        while not rospy.is_shutdown():
            k = get_key()
            if k in ("q", "Q", "\x03"):
                break
            step = 0.06
            if k in ("w", "W"):
                x += step
            elif k in ("s", "S"):
                x -= step
            elif k in ("a", "A"):
                y += step
            elif k in ("d", "D"):
                y -= step
            elif k in ("r", "R"):
                z = min(4.0, z + 0.05)
            elif k in ("f", "F"):
                z = max(1.2, z - 0.05)
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
            msg.pose.orientation.w = 1.0
            pub.publish(msg)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
