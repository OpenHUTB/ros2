#!/usr/bin/env python3
"""端到端节点：深度图 -> CNN -> /cmd_vel。

模型：models/e2e.npz（24×32 深度图输入，输出 v 与 ω）。
可订阅 preprocess 参数决定是否做归一化（训练时是 depth/8）。
"""

from __future__ import annotations

import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from humanoid_mujoco_slam_nn_nav.neural import NeuralPolicy  # noqa: E402


class E2ENode:
    def __init__(self):
        self.policy = NeuralPolicy()
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.v = 0.0
        self.w = 0.0
        self.alpha = float(rospy.get_param("~filter", 0.35))
        rospy.Subscriber("/camera/depth/image_raw", Image, self._on_image, queue_size=1)
        rospy.loginfo("端到端 CNN 已加载，等待 /camera/depth/image_raw ...")

    def _on_image(self, msg: Image):
        h, w = msg.height, msg.width
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
        v, wc = self.policy.end_to_end(depth)
        self.v += (v - self.v) * self.alpha
        self.w += (wc - self.w) * self.alpha
        if self.v < 0.06 and abs(self.w) > 0.25:
            self.v = 0.08
        tw = Twist()
        tw.linear.x = float(self.v)
        tw.angular.z = float(self.w)
        self.pub.publish(tw)


if __name__ == "__main__":
    rospy.init_node("humanoid_e2e")
    E2ENode()
    rospy.spin()
