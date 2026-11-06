#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务 1：键盘遥控节点

按键：
  W / S      前进 / 后退
  A / D      左转 / 右转（偏航角速度）
  Q / E      上升 / 下降
  1 / 2 / 3  速度档位（1.0 / 2.0 / 4.0 m/s）
  空格       悬停（速度归零）
  T / L      起飞 / 降落（调用桥接节点的服务）
  ESC        退出

节点发布 /drone/cmd_vel，并订阅 /drone/odom 显示巡航距离与当前速度。
"""

import math
import os
import select
import sys
import termios

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

SPEED_LEVELS = (1.0, 2.0, 4.0)


class KeyboardTeleop(object):
    def __init__(self):
        self.speed_idx = 1
        self.total_dist = 0.0
        self.last_xy = None
        self.current_speed = 0.0

        self.pub = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/drone/odom', Odometry, self._on_odom)
        self.takeoff = rospy.ServiceProxy('/drone/takeoff', Trigger)
        self.land = rospy.ServiceProxy('/drone/land', Trigger)

        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        if self.last_xy is not None:
            self.total_dist += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)
        v = msg.twist.twist.linear
        self.current_speed = math.sqrt(v.x * v.x + v.y * v.y)

    def _set_raw(self):
        new = termios.tcgetattr(self.fd)
        new[3] &= ~(termios.ICANON | termios.ECHO)
        new[6][termios.VMIN] = 0
        new[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSADRAIN, new)

    def _restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)

    def _read_key(self):
        r, _, _ = select.select([sys.stdin], [], [], 0.1)
        if not r:
            return None
        return sys.stdin.read(1)

    def _publish(self, vx, vy, vz, wz):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = wz
        self.pub.publish(cmd)

    def _banner(self):
        print('=' * 52)
        print(' 无人机键盘遥控  |  档位 %d (%.1f m/s)' % (self.speed_idx + 1,
                                                          SPEED_LEVELS[self.speed_idx]))
        print(' W/S 前后   A/D 转向   Q/E 升降   空格 悬停')
        print(' 1/2/3 速度   T 起飞   L 降落   ESC 退出')
        print('=' * 52)

    def run(self):
        rospy.sleep(1.5)
        self._banner()
        v = SPEED_LEVELS[self.speed_idx]
        yaw_rate = 0.8

        try:
            self._set_raw()
            while not rospy.is_shutdown():
                key = self._read_key()
                if key is None:
                    continue
                if key in ('w', 'W'):
                    self._publish(v, 0, 0, 0)
                elif key in ('s', 'S'):
                    self._publish(-v, 0, 0, 0)
                elif key in ('a', 'A'):
                    self._publish(0, 0, 0, yaw_rate)
                elif key in ('d', 'D'):
                    self._publish(0, 0, 0, -yaw_rate)
                elif key in ('q', 'Q'):
                    self._publish(0, 0, v, 0)
                elif key in ('e', 'E'):
                    self._publish(0, 0, -v, 0)
                elif key == ' ':
                    self._publish(0, 0, 0, 0)
                elif key in ('1', '2', '3'):
                    self.speed_idx = int(key) - 1
                    v = SPEED_LEVELS[self.speed_idx]
                    print('\r速度档位 -> %s (%.1f m/s)          ' %
                          (key, v), end='', flush=True)
                elif key in ('t', 'T'):
                    try:
                        resp = self.takeoff()
                        print('\r起飞: %s          ' % resp.message, end='', flush=True)
                    except Exception as exc:
                        print('\r起飞失败: %s          ' % exc, end='', flush=True)
                elif key in ('l', 'L'):
                    try:
                        resp = self.land()
                        print('\r降落: %s          ' % resp.message, end='', flush=True)
                    except Exception as exc:
                        print('\r降落失败: %s          ' % exc, end='', flush=True)
                elif key == '\x1b':
                    break
                else:
                    continue
                print('\r档位 %.1f m/s | 速度 %.2f m/s | 累计 %.1f m      ' %
                      (v, self.current_speed, self.total_dist), end='', flush=True)
        finally:
            self._restore()
            self._publish(0, 0, 0, 0)
            print('\n遥控结束')


def main():
    rospy.init_node('teleop_keyboard')
    KeyboardTeleop().run()


if __name__ == '__main__':
    main()
