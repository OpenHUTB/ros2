#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端 RL 部署节点：加载训练好的 PPO 权重，相机图像 -> 速度指令。"""
import os
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from stable_baselines3 import PPO


class DeployNode:
    def __init__(self):
        model_path = rospy.get_param("~model_path",
            os.path.expanduser("~/catkin_ws/src/air/uav_e2e_rl/models/ppo_e2e.zip"))
        rospy.loginfo(f"loading model: {model_path}")
        self.model = PPO.load(model_path)
        self._img = np.zeros((1, 48, 64), dtype=np.float32)
        self._cmd_pub = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/camera/image_raw", Image, self._img_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self._step)

    def _img_cb(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        gray = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
        small = cv2.resize(gray, (64, 48)).astype(np.float32) / 255.0
        self._img = small[np.newaxis, :, :]

    def _step(self, _):
        action, _ = self.model.predict(self._img, deterministic=True)
        vx, wz = float(action[0]), float(action[1])
        tw = Twist()
        tw.linear.x = vx
        tw.angular.z = wz
        self._cmd_pub.publish(tw)


def main():
    rospy.init_node("e2e_rl_deploy")
    DeployNode()
    rospy.spin()


if __name__ == "__main__":
    main()
