#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
perception_node.py —— 部署节点：前视图像 -> CNN 推理 -> 发布 /uav/cmd_vel
"""
import os
import rospy
import torch
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from dataset import preprocess_image
from model import build_model


class NeuralPerceptionNode:
    def __init__(self):
        rospy.init_node("uav_neural_perception")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.model_path = rospy.get_param(
            "~model_path",
            os.path.join(os.path.dirname(__file__), "models", "best_drone_model.pth"))
        self.max_vx = float(rospy.get_param("~max_vx", 2.0))
        self.max_vy = float(rospy.get_param("~max_vy", 0.5))
        self.max_yaw = float(rospy.get_param("~max_yaw", 0.4))

        self.model = build_model().to(self.device)
        if not os.path.exists(self.model_path):
            rospy.logwarn("找不到权重文件 %s，将使用随机权重（请先跑 train.py）", self.model_path)
        else:
            ckpt = torch.load(self.model_path, map_location=self.device)
            state_dict = ckpt.get("state_dict", ckpt) if isinstance(ckpt, dict) else ckpt
            self.model.load_state_dict(state_dict)
            rospy.loginfo("模型权重已加载：%s", self.model_path)
        self.model.eval()

        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/camera/image_raw", Image, self.on_image, queue_size=1)
        rospy.loginfo("神经网络感知节点已启动，等待 /camera/image_raw ...")

    def on_image(self, msg):
        img_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        tensor = preprocess_image(img_bgr).unsqueeze(0).to(self.device)

        with torch.no_grad():
            action = self.model(tensor)[0].cpu().numpy()

        vx, vy, yaw = float(action[0]), float(action[1]), float(action[2])
        vx = np.clip(vx, -self.max_vx, self.max_vx)
        vy = np.clip(vy, -self.max_vy, self.max_vy)
        yaw = np.clip(yaw, -self.max_yaw, self.max_yaw)
        vz = 0.0

        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = vx, vy, vz
        twist.angular.z = yaw
        self.cmd_pub.publish(twist)
        rospy.loginfo_throttle(1.0, "下发: vx=%.2f vy=%.2f yaw=%.2f", vx, vy, yaw)


if __name__ == "__main__":
    try:
        NeuralPerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
