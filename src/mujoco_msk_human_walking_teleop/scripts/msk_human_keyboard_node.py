#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘遥操作节点：终端按键 -> 发布肌肉激活与动作指令。

在 xterm 终端中运行（launch 中以 launch-prefix="xterm -e" 启动）。
持续键按住生效（30 Hz 心跳重发手动激活），动作键按一次发布一次。
ESC 退出。

话题：
  发布 /msk_human/manual_cmd (std_msgs/Float32MultiArray, 22 维)
  发布 /msk_human/action_cmd (std_msgs/String)
"""
import os
import select
import sys
import termios
import tty

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import rospy
from std_msgs.msg import Float32MultiArray, String

from msk_human_keyboard import KEY_HELP, KeyboardPolicy


class KeyboardNode:
    def __init__(self):
        self.pub_manual = rospy.Publisher(
            "/msk_human/manual_cmd", Float32MultiArray, queue_size=1)
        self.pub_action = rospy.Publisher(
            "/msk_human/action_cmd", String, queue_size=1)
        self.policy = KeyboardPolicy()
        self.rate = rospy.Rate(30)
        self._old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        rospy.on_shutdown(self.restore_tty)
        rospy.loginfo("\n" + KEY_HELP)

    def restore_tty(self):
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_attr)
        except Exception:
            pass

    def poll_key(self, timeout=0.0):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else None

    def spin(self):
        while not rospy.is_shutdown():
            k = self.poll_key(0)
            if k is not None:
                if k == "\x1b":
                    rospy.signal_shutdown("ESC")
                    return
                if k.lower() == "h":
                    rospy.loginfo("\n" + KEY_HELP)
                else:
                    action = self.policy.press(k.lower())
                    if action:
                        self.pub_action.publish(String(action))
            msg = Float32MultiArray()
            msg.data = [float(v) for v in self.policy.manual_vector()]
            self.pub_manual.publish(msg)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("msk_human_keyboard_node")
    KeyboardNode().spin()
