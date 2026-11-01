#!/usr/bin/python3
"""Terminal keyboard teleop (W/A/S/D) for the GVNN simulation."""

from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String


SCENES = ["warehouse", "obstacle_field", "crossing"]


class Teleop:
    def __init__(self) -> None:
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.pub_reset = rospy.Publisher("/gvnn/reset", Empty, queue_size=2)
        self.pub_scene = rospy.Publisher("/gvnn/scene", String, queue_size=2)
        self.v = 0.0
        self.w = 0.0
        self.fd = sys.stdin.fileno()
        self.old = None

    def restore(self) -> None:
        if self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            self.old = None

    def handle(self, ch: str) -> bool:
        if ch in ("q", "Q", "\x03"):
            return False
        if ch in ("w", "W"):
            self.v = min(0.9, self.v + 0.15)
        elif ch in ("s", "S"):
            self.v = max(-0.4, self.v - 0.15)
        elif ch in ("a", "A"):
            self.w = min(1.4, self.w + 0.25)
        elif ch in ("d", "D"):
            self.w = max(-1.4, self.w - 0.25)
        elif ch == " ":
            self.v = self.w = 0.0
        elif ch in ("r", "R"):
            self.v = self.w = 0.0
            self.pub_reset.publish(Empty())
        elif ch in ("1", "2", "3"):
            self.v = self.w = 0.0
            self.pub_scene.publish(String(data=SCENES[int(ch) - 1]))
        else:
            return True
        self.pub.publish(self._msg())
        print("cmd: v=%.2f m/s  w=%.2f rad/s" % (self.v, self.w), flush=True)
        return True

    def _msg(self) -> Twist:
        msg = Twist()
        msg.linear.x = float(self.v)
        msg.angular.z = float(self.w)
        return msg

    def run(self) -> None:
        if sys.stdin.isatty():
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        print(__doc__)
        print("W/S: 前进/后退   A/D: 左/右转   空格: 停车")
        print("R: 重置   1/2/3: 切换场景   Q: 退出")
        rate = rospy.Rate(20.0)
        try:
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0)[0]:
                    ch = sys.stdin.read(1)
                    if not self.handle(ch):
                        break
                self.pub.publish(self._msg())
                rate.sleep()
        finally:
            self.v = self.w = 0.0
            self.pub.publish(self._msg())
            self.restore()


if __name__ == "__main__":
    rospy.init_node("gvnn_teleop_keyboard")
    node = Teleop()
    rospy.on_shutdown(node.restore)
    node.run()
