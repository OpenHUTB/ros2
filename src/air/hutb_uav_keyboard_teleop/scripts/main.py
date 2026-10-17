#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hutb_uav_keyboard_teleop: 无人机键盘运动控制节点 (不依赖 uinput/手柄设备)
=========================================================
直接发布 rotors_joy_interface 所需的 /joy 消息:

  axes[0] = roll     (A/D 左右滚转)
  axes[1] = pitch    (Up/Down 俯仰)
  axes[2] = thrust   (W/S 油门, -1..1)
  buttons[3] = yaw_left    (Q)
  buttons[4] = yaw_right   (E)

控制链路: 本节点(/joy) -> rotors_joy_interface
          -> /firefly/command/roll_pitch_yawrate_thrust
          -> roll_pitch_yawrate_thrust_controller -> 电机

用法: roslaunch hutb_uav_keyboard_teleop main.launch
"""
import sys
import termios
import tty
import select
import threading

import rospy
from sensor_msgs.msg import Joy

RATE = 20          # 发布频率 Hz
STEP = 0.06        # 杆量步进
THRUST_STEP = 0.04


class KeyReader:
    """非阻塞读取单个按键 (termios raw 模式)"""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.non_tty = False
        try:
            self.old = termios.tcgetattr(self.fd)
        except termios.error:
            # roslaunch 后台运行时 stdin 非 tty, 键盘输入不可用
            self.non_tty = True

    def __enter__(self):
        if self.non_tty:
            return self
        tty.setraw(self.fd)
        return self

    def __exit__(self, *args):
        if not self.non_tty:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def poll(self, timeout=0.01):
        if select.select([sys.stdin], [], [], timeout)[0]:
            ch = sys.stdin.read(1)
            return ch
        return None

    def read_seq(self, timeout=0.02):
        """读取转义序列（方向键）"""
        seq = self.poll(timeout)
        if seq == '[':
            seq2 = self.poll(timeout)
            if seq2:
                return '\x1b[' + seq2
        return seq


class DroneTeleop:
    def __init__(self):
        self.pub = rospy.Publisher('/joy', Joy, queue_size=1)
        self.roll = 0.0
        self.pitch = 0.0
        self.thrust = -1.0          # 初始油门 0
        self.yaw_left = False
        self.yaw_right = False
        self._lock = threading.Lock()
        rospy.loginfo('无人机键盘控制启动. 按键: A/D=滚转, Up/Down=俯仰, W/S=油门, Q/E=偏航, C=停止')

    def on_key(self, ch):
        with self._lock:
            if ch == 'a':
                self.roll = max(-1.0, self.roll - STEP)
            elif ch == 'd':
                self.roll = min(1.0, self.roll + STEP)
            elif ch == 'w':
                self.thrust = min(1.0, self.thrust + THRUST_STEP)
            elif ch == 's':
                self.thrust = max(-1.0, self.thrust - THRUST_STEP)
            elif ch == 'q':
                self.yaw_left = True
            elif ch == 'e':
                self.yaw_right = True
            elif ch == 'c':
                self.roll = self.pitch = 0.0
                self.thrust = -1.0
            elif ch == '\x1b[A':
                self.pitch = min(1.0, self.pitch + STEP)
            elif ch == '\x1b[B':
                self.pitch = max(-1.0, self.pitch - STEP)

    def publish_loop(self):
        r = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            with self._lock:
                msg = Joy()
                msg.header.stamp = rospy.Time.now()
                msg.axes = [self.roll, self.pitch, self.thrust, 0.0, 0.0, 0.0, 0.0]
                msg.buttons = [0] * 11
                if self.yaw_left:
                    msg.buttons[3] = 1
                if self.yaw_right:
                    msg.buttons[4] = 1
            self.pub.publish(msg)
            r.sleep()

    def run(self):
        t = threading.Thread(target=self.publish_loop, daemon=True)
        t.start()
        with KeyReader() as kr:
            if kr.non_tty:
                rospy.logwarn('无终端键盘输入(建议前台 rosrun 运行), 仅发布零指令')
            while not rospy.is_shutdown():
                ch = kr.poll(0.05)
                if ch is None:
                    continue
                if ch == '\x1b':
                    seq = kr.read_seq()
                    if seq:
                        self.on_key(seq)
                else:
                    self.on_key(ch)


def main():
    rospy.init_node('drone_keyboard_teleop')
    DroneTeleop().run()


if __name__ == '__main__':
    main()
