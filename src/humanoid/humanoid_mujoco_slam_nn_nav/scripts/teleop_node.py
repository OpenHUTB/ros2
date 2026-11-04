#!/usr/bin/env python3
"""键盘遥控节点：W/S 前后，A/D 转向，空格急停，R 重置，Q 退出。

用 termios + select 读单个按键（非阻塞、无需回车），松开按键 0.3 s 后自动停车。
"""

from __future__ import annotations

import os
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

KEYMAP = {
    "w": (0.22, 0.0), "s": (-0.10, 0.0),
    "a": (0.10, 0.40), "d": (0.10, -0.40),
    "q": (0.0, 0.6), "e": (0.0, -0.6),
}


class Teleop:
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.reset_pub = rospy.Publisher("/humanoid/reset", Empty, queue_size=1)
        self.settings = termios.tcgetattr(sys.stdin)
        self.last_key = None
        self.last_time = rospy.Time.now()

    def _read_key(self, timeout=0.05):
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if not rlist:
            return None
        return sys.stdin.read(1)

    def spin(self):
        rate = rospy.Rate(30)
        tw = Twist()
        rospy.loginfo("键盘遥控已启动：W/S 前后，A/D 转向，空格急停，R 重置，Q 退出")
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not rospy.is_shutdown():
                key = self._read_key()
                now = rospy.Time.now()
                if key:
                    key = key.lower()
                    if key == "q":
                        break
                    if key == "r":
                        self.reset_pub.publish(Empty())
                        rospy.loginfo("reset")
                        key = None
                    elif key == " ":
                        tw.linear.x = tw.angular.z = 0.0
                        self.pub.publish(tw)
                        self.last_key = None
                        key = None
                    elif key in KEYMAP:
                        v, w = KEYMAP[key]
                        tw.linear.x, tw.angular.z = v, w
                        self.pub.publish(tw)
                        self.last_key = key
                        self.last_time = now
                # 松键自动停车
                if self.last_key and (now - self.last_time).to_sec() > 0.3:
                    tw.linear.x = tw.angular.z = 0.0
                    self.pub.publish(tw)
                    self.last_key = None
                rate.sleep()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            tw.linear.x = tw.angular.z = 0.0
            self.pub.publish(tw)


if __name__ == "__main__":
    rospy.init_node("humanoid_teleop")
    os.system("")  # 兼容部分终端
    Teleop().spin()
