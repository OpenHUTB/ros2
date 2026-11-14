#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
键盘遥控节点（任务 1）

把终端按键转换成 /uav/cmd_vel 上的速度指令。

操作说明：
    w / s    前进 / 后退
    a / d    左移 / 右移
    q / e    左转 / 右转
    r / f    上升 / 下降
    空格     急停（速度清零）
    Ctrl+C   退出

注意：需要用真正的终端运行（rosrun 或 roslaunch 里直接启动），
      通过管道重定向标准输入时无法读取按键。
"""

import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

BANNER = """
========================================
 无人机键盘遥控 (AirSim + ROS)
----------------------------------------
 w/s  前进/后退     a/d  左移/右移
 q/e  左转/右转     r/f  上升/下降
 空格 急停          Ctrl+C 退出
========================================
按任意键开始...
"""


class KeyboardTeleop(object):
    def __init__(self):
        self.speed = float(rospy.get_param('~speed', 3.0))            # m/s
        self.turn_rate = float(rospy.get_param('~turn_rate', 1.0))    # rad/s
        self.rate_hz = float(rospy.get_param('~rate', 20.0))

        self.pub = rospy.Publisher('/uav/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()

        # 当前按下的键，避免长按时的重复触发
        self.keys = set()

    def get_key(self, timeout=0.05):
        fd = sys.stdin.fileno()
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''

    def update_cmd(self):
        cmd = Twist()
        if 'w' in self.keys:
            cmd.linear.x = self.speed
        if 's' in self.keys:
            cmd.linear.x = -self.speed
        if 'a' in self.keys:
            cmd.linear.y = -self.speed
        if 'd' in self.keys:
            cmd.linear.y = self.speed
        if 'r' in self.keys:
            cmd.linear.z = self.speed
        if 'f' in self.keys:
            cmd.linear.z = -self.speed
        if 'q' in self.keys:
            cmd.angular.z = self.turn_rate
        if 'e' in self.keys:
            cmd.angular.z = -self.turn_rate
        self.cmd = cmd

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(BANNER)
        self.get_key(timeout=0.5)

        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                if key:
                    key = key.lower()
                    if key == ' ':
                        self.keys.clear()
                        self.cmd = Twist()
                        print('\n[急停]')
                    elif key in 'wsadqerf':
                        # 松开判断靠“只保留最后一次按键”，简化处理为单键操作
                        self.keys = {key}
                    elif key == '\x03':      # Ctrl+C
                        break
                self.update_cmd()
                self.pub.publish(self.cmd)
                rate.sleep()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self.pub.publish(Twist())
            print('\n已停止发布速度指令')


def main():
    rospy.init_node('keyboard_teleop')
    KeyboardTeleop().run()


if __name__ == '__main__':
    main()
