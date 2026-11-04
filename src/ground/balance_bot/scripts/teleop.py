#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘遥控（ROS 1）：W/S 前进后退，A/D 转向，空格停，Q 退出。

用 termios 非阻塞读键，不依赖 PyQt/pygame —— 虚拟机无 GPU、SSH 进来都能用。
注意：必须在**真实终端**里运行（需要 TTY）。
"""
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


class KeyTeleop(object):

    def __init__(self):
        self.v_step = float(rospy.get_param("~v_step", 0.15))
        self.w_step = float(rospy.get_param("~w_step", 0.40))
        self.v = 0.0
        self.w = 0.0
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def run(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        print("\n  W/S 前进后退   A/D 转向   空格 停   Q 退出\n")
        try:
            tty.setcbreak(fd)
            rate = rospy.Rate(20)
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    c = ch.lower()
                    if c == "q" or ch == "\x03":
                        break
                    elif c == "w":
                        self.v += self.v_step
                    elif c == "s":
                        self.v -= self.v_step
                    elif c == "a":
                        self.w += self.w_step
                    elif c == "d":
                        self.w -= self.w_step
                    elif ch == " ":
                        self.v = self.w = 0.0
                    self.v = max(-0.6, min(0.6, self.v))
                    self.w = max(-1.2, min(1.2, self.w))
                    sys.stderr.write("\r  v=%+.2f m/s  w=%+.2f rad/s   " % (self.v, self.w))
                    sys.stderr.flush()
                tw = Twist()
                tw.linear.x = self.v
                tw.angular.z = self.w
                self.pub.publish(tw)
                rate.sleep()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            self.pub.publish(Twist())
            print("\n  [退出] 已停车")


def main():
    rospy.init_node("balance_teleop")
    KeyTeleop().run()


if __name__ == "__main__":
    main()
