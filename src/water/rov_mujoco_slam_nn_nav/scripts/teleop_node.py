#!/usr/bin/env python3
"""无人机键盘遥控节点：W/S 前后，A/D 左右，Q/E 偏航，R/F 升降，空格悬停，X 退出。"""

from __future__ import annotations

import os
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Float64

KEYS = {"w": (0.6, 0.0, 0.0), "s": (-0.5, 0.0, 0.0),
        "a": (0.0, 0.6, 0.0), "d": (0.0, -0.6, 0.0),
        "q": (0.0, 0.0, 0.8), "e": (0.0, 0.0, -0.8)}


class Teleop:
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.alt = rospy.Publisher("/rov/altitude", Float64, queue_size=1)
        self.reset = rospy.Publisher("/rov/reset", Empty, queue_size=1)
        self.settings = termios.tcgetattr(sys.stdin)
        self.z = 1.2

    def spin(self):
        rate = rospy.Rate(30)
        tw = Twist()
        print("W/S 前后，A/D 左右，Q/E 偏航，R/F 升降，空格悬停，X 退出")
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.02)[0]:
                    k = sys.stdin.read(1).lower()
                    if k == "x":
                        break
                    if k == " ":
                        tw = Twist()
                    elif k in KEYS:
                        tw.linear.x, tw.linear.y, tw.angular.z = KEYS[k]
                    elif k == "r":
                        self.z = min(2.2, self.z + 0.2)
                    elif k == "f":
                        self.z = max(0.6, self.z - 0.2)
                    elif k == "0":
                        self.reset.publish(Empty())
                    self.pub.publish(tw)
                    self.alt.publish(Float64(data=self.z))
                rate.sleep()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.pub.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("rov_teleop")
    Teleop().spin()

