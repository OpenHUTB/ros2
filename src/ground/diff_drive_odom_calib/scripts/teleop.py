#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘遥控（ROS 1）：W/S 前后，A/D 转向，空格停，Q 退出。

刻意用 termios 非阻塞读键而不是 PyQt/pygame：
    虚拟机无 GPU 也能用，SSH 进来也能用，只依赖标准库。

注意：必须在**真实终端**里运行（需要 TTY）；通过 roslaunch 启动时
要加 `launch-prefix="xterm -e"` 或直接在终端里 `rosrun`。
"""

import os
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


class KeyTeleop(object):

    def __init__(self):
        self.v_step = float(rospy.get_param("~v_step", 0.30))
        self.w_step = float(rospy.get_param("~w_step", 0.80))
        self.v = 0.0
        self.w = 0.0
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # ------------------------------------------------------------------
    def _read_key(self, fd):
        import select
        r, _w, _x = select.select([sys.stdin], [], [], 0.1)
        if not r:
            return None
        return sys.stdin.read(1)

    def run(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        print("")
        print("  键盘遥控  W/S 前进后退   A/D 左右转   空格 停车   Q 退出")
        print("  每次按键给一个速度增量，松开不会自动停（按空格清零）")
        print("")
        try:
            tty.setcbreak(fd)
            rate = rospy.Rate(20)
            while not rospy.is_shutdown():
                ch = self._read_key(fd)
                if ch:
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
                    self.v = max(-0.8, min(0.8, self.v))
                    self.w = max(-1.6, min(1.6, self.w))
                    # 只打印到 stderr，避免污染 roslaunch 的 stdout 日志
                    sys.stderr.write("\r  v=%+.2f m/s  w=%+.2f rad/s   " % (self.v, self.w))
                    sys.stderr.flush()
                tw = Twist()
                tw.linear.x = self.v
                tw.angular.z = self.w
                self.pub.publish(tw)
                rate.sleep()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            tw = Twist()
            self.pub.publish(tw)
            print("\n  [退出] 已停车")


def main():
    rospy.init_node("key_teleop")
    KeyTeleop().run()


if __name__ == "__main__":
    main()
