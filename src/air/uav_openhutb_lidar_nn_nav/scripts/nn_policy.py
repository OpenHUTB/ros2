#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务4：用训练好的网络实时飞行（输入只有激光扫描，不使用任何真值）。

权重由 nn_train.py 生成（scan_policy.npz）。
"""

import os

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class NnPolicy(object):
    def __init__(self):
        weights = rospy.get_param("~weights", "")
        if not weights:
            weights = os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), "models", "scan_policy.npz")
        weights = os.path.expanduser(weights)
        if not os.path.exists(weights):
            rospy.logfatal("找不到权重文件 %s，请先运行 --mode train", weights)
            raise SystemExit(1)

        z = np.load(weights)
        self.W1, self.b1 = z["W1"], z["b1"]
        self.W2, self.b2 = z["W2"], z["b2"]
        self.xm, self.xs = z["xm"], z["xs"]
        self.ym, self.ys = z["ym"], z["ys"]
        self.n = self.W1.shape[0]
        self.range_max = float(rospy.get_param("~range_max", 40.0))

        self.count = 0
        self.pub = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/uav/lidar/scan", LaserScan, self.on_scan, queue_size=1)
        rospy.loginfo("神经网络策略已加载：%s（输入 %d 维）", weights, self.n)

    def on_scan(self, msg):
        r = np.array(msg.ranges, dtype=np.float32)
        r[~np.isfinite(r)] = self.range_max
        r = np.clip(r, 0.0, self.range_max)
        idx = np.linspace(0, len(r) - 1, self.n).astype(int)
        feat = (r[idx] - self.xm) / self.xs

        h = np.maximum(feat @ self.W1 + self.b1, 0.0)
        out = (h @ self.W2 + self.b2) * self.ys + self.ym

        cmd = Twist()
        cmd.linear.x = float(np.clip(out[0], 0.0, 3.0))
        cmd.angular.z = float(np.clip(out[1], -1.0, 1.0))
        self.pub.publish(cmd)

        self.count += 1
        if self.count % 100 == 0:
            rospy.loginfo("推理 %d 帧  v=%.2f w=%.2f", self.count, cmd.linear.x, cmd.angular.z)


def main():
    rospy.init_node("uav_nn_policy")
    NnPolicy()
    rospy.spin()


if __name__ == "__main__":
    main()
