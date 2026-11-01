#!/usr/bin/env python3
"""端到端节点：深度图 -> CNN -> /cmd_vel（机体系 vx, vy）+ 偏航角速度。"""

from __future__ import annotations

import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from aerial_mujoco_slam_nn_nav.neural import NeuralPolicy  # noqa: E402


class E2ENode:
    def __init__(self):
        self.policy = NeuralPolicy()
        self.alpha = float(rospy.get_param("~filter", 0.4))
        self.altitude = float(rospy.get_param("~altitude", 1.2))
        self.vx = self.vy = self.w = 0.0
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_alt = rospy.Publisher("/drone/altitude", Float64, queue_size=1)
        rospy.Subscriber("/camera/depth/image_raw", Image, self._on_image, queue_size=1)
        rospy.loginfo("端到端 CNN 已加载，等待 /camera/depth/image_raw")

    def _on_image(self, msg):
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        vx, vy, w = self.policy.end_to_end(depth)
        self.vx += (vx - self.vx) * self.alpha
        self.vy += (vy - self.vy) * self.alpha
        self.w += (w - self.w) * self.alpha
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = self.vx, self.vy, self.w
        self.pub.publish(tw)
        self.pub_alt.publish(Float64(data=self.altitude))


if __name__ == "__main__":
    rospy.init_node("drone_e2e")
    E2ENode()
    rospy.spin()
