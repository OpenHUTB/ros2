#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""监视节点：/brake_status 的真实订阅者（三节点架构之三）

作用：证明状态话题存在真实消费者；同时可作为演示时的状态字幕。
状态变化时打印一行，避免刷屏。
"""
import rospy
from std_msgs.msg import Bool

_last = None


def _on_status(msg):
    global _last
    if msg.data != _last:
        print("[monitor] 制动状态 -> %s" % ("BRAKE" if msg.data else "RELEASE"))
        _last = msg.data


if __name__ == '__main__':
    rospy.init_node('brake_monitor', anonymous=True)
    rospy.Subscriber('/brake_status', Bool, _on_status, queue_size=10)
    print("监视节点已启动，等待 /brake_status ...")
    rospy.spin()
