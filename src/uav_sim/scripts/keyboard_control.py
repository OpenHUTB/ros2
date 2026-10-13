#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
键盘控制 hector 无人机节点（任务①）。

功能：
  1. 启动后自动调用 /engage 服务解锁电机；
  2. 读取键盘输入，发布 geometry_msgs/Twist 到 /cmd_vel 控制无人机运动。

按键说明：
  w/s  前进/后退      a/d  左移/右移
  r/f  上升/下降      q/e  左转/右转
  空格 悬停           Ctrl+C 退出并断开电机
"""
import rospy
import sys
import select
import termios
import tty

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

# 按键 -> (前后, 左右, 升降, 转向) 的方向符号
KEYMAP = {
    'w': (1,  0,  0,  0),   # 前进
    's': (-1, 0,  0,  0),   # 后退
    'a': (0,  1,  0,  0),   # 左移
    'd': (0, -1,  0,  0),   # 右移
    'r': (0,  0,  1,  0),   # 上升
    'f': (0,  0, -1,  0),   # 下降
    'q': (0,  0,  0,  1),   # 左转（逆时针）
    'e': (0,  0,  0, -1),   # 右转（顺时针）
}


class KeyboardControl(object):
    def __init__(self):
        rospy.init_node('keyboard_control')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.max_xy = rospy.get_param('~max_linear_xy', 1.0)   # 水平速度 m/s
        self.max_z = rospy.get_param('~max_linear_z', 0.5)     # 升降速度 m/s
        self.max_yaw = rospy.get_param('~max_angular_z', 1.0)  # 偏航角速度 rad/s
        self.rate = rospy.Rate(10.0)

        # 起飞前必须先 engage 解锁电机
        rospy.loginfo("等待 /engage 服务 ...")
        rospy.wait_for_service('/engage', timeout=15)
        rospy.ServiceProxy('/engage', Empty)()
        rospy.loginfo("已解锁(engage)，可以用键盘飞行了！")

    def _get_key(self, timeout=0.1):
        """非阻塞读取单个按键，返回字符（无按键时返回空串）。"""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            key = sys.stdin.read(1) if rlist else ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def run(self):
        rospy.loginfo("键盘控制：")
        rospy.loginfo("  w/s 前后 | a/d 左右 | r/f 升降 | q/e 转向 | 空格 悬停")
        rospy.loginfo("  按 Ctrl+C 退出")

        # 自动起飞：解锁后上升 3 秒到安全悬停高度（约 1.5m），避免趴在地面空转
        rospy.loginfo("自动起飞中（上升 3 秒）...")
        takeoff = Twist()
        takeoff.linear.z = 0.5
        for _ in range(30):
            self.pub.publish(takeoff)
            self.rate.sleep()
        rospy.loginfo("到达悬停高度，开始键盘控制！")

        twist = Twist()
        try:
            while not rospy.is_shutdown():
                key = self._get_key()
                if key == '\x03':        # Ctrl+C
                    break
                if key == ' ':           # 悬停：速度清零
                    twist = Twist()
                elif key in KEYMAP:
                    kx, ky, kz, kyaw = KEYMAP[key]
                    twist.linear.x = kx * self.max_xy
                    twist.linear.y = ky * self.max_xy
                    twist.linear.z = kz * self.max_z
                    twist.angular.z = kyaw * self.max_yaw
                self.pub.publish(twist)
                self.rate.sleep()
        finally:
            self.pub.publish(Twist())
            try:
                rospy.ServiceProxy('/shutdown', Empty)()
            except Exception:
                pass
            rospy.loginfo("已停止并断开电机")


if __name__ == '__main__':
    try:
        KeyboardControl().run()
    except rospy.ROSInterruptException:
        pass
