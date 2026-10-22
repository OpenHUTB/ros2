#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""神经网络逆模型节点：目标指尖位置 → 肌肉激活指令。

收到目标点后用 MLP 逆模型解算 11 维激活并发布（free 场景到达任务）。

话题：
  订阅  /msk_hand/target      (geometry_msgs/Point) 食指指尖目标位置(m)
  发布  /msk_hand/muscle_cmd  (std_msgs/Float32MultiArray) 11 维肌肉激活
参数：
  ~weights (str, 默认 assets/nn_weights.npz)
"""
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

from msk_hand_nn import MLP
from msk_hand_core import PKG_DIR

ASSET_DIR = os.path.join(PKG_DIR, "assets")


class NNNode:
    def __init__(self):
        path = rospy.get_param("~weights",
                               os.path.join(ASSET_DIR, "nn_weights.npz"))
        self.net, self.x_mean, self.x_std = MLP.load(path)
        self.pub = rospy.Publisher("/msk_hand/muscle_cmd", Float32MultiArray,
                                   queue_size=1)
        rospy.Subscriber("/msk_hand/target", Point, self.on_target,
                         queue_size=1)
        rospy.loginfo("msk_hand_nn_node 就绪, 权重: %s", path)

    def on_target(self, msg):
        x = np.array([[msg.x, msg.y, msg.z]])
        a, _ = self.net.forward((x - self.x_mean) / self.x_std)
        msg_out = Float32MultiArray()
        msg_out.data = [float(v) for v in np.clip(a[0], 0.0, 1.0)]
        self.pub.publish(msg_out)
        rospy.loginfo("目标(%.3f,%.3f,%.3f) -> 激活 %s",
                      msg.x, msg.y, msg.z, np.round(msg_out.data, 2))


if __name__ == "__main__":
    rospy.init_node("msk_hand_nn_node")
    NNNode()
    rospy.spin()
