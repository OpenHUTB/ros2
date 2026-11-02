#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘遥操作节点：终端按键（切换式）→ 发布肌肉激活指令。

按键映射见 msk_hand_keyboard.KEY_HELP。按一次开、再按一次关；
空格全部放松，ESC 退出。激活经 KeyboardPolicy 一阶平滑后以 50Hz 发布。

话题：
  发布  /msk_hand/muscle_cmd (std_msgs/Float32MultiArray) 11 维肌肉激活
"""
import os
import sys
import termios
import tty
import select

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import rospy
from std_msgs.msg import Float32MultiArray

from msk_hand_keyboard import KeyboardPolicy, KEY_HELP


class KeyboardNode:
    def __init__(self):
        self.pub = rospy.Publisher("/msk_hand/muscle_cmd", Float32MultiArray,
                                   queue_size=1)
        self.policy = KeyboardPolicy()
        self.rate = rospy.Rate(50)
        # 终端切换为原始模式（退出时恢复）
        self._old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        rospy.loginfo("\n" + KEY_HELP)
        rospy.on_shutdown(self.restore_tty)

    def restore_tty(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_attr)

    def poll_key(self, timeout=0.0):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else None

    def spin(self):
        dt = 0.02
        while not rospy.is_shutdown():
            k = self.poll_key(0)
            if k is not None:
                if k == "\x1b":          # ESC 退出
                    break
                m = self.policy.toggle(k)
                if m:
                    rospy.loginfo("按键 %r -> %s | held=%s",
                                  k, m, sorted(self.policy.held))
            cmd = self.policy.update(dt)
            msg = Float32MultiArray()
            msg.data = [float(v) for v in cmd]
            self.pub.publish(msg)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("msk_hand_keyboard_node")
    KeyboardNode().spin()
