#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 极简自主节点：启动即加载权重并订阅图像，收到图像 -> 推理 -> 直接下发速度。
# 无状态机、无看门狗、无交互阻塞；vz 锁死 0（定高），vy/yaw 软限幅。

import rospy
import torch
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from dataset import preprocess_image
from model import build_model


class AutonomousDrone:
    def __init__(self):
        rospy.init_node('drone_autonomous_node')
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = build_model().to(self.device)
        ckpt = torch.load('models/best_drone_model.pth', map_location=self.device)
        state_dict = ckpt.get('state_dict', ckpt) if isinstance(ckpt, dict) else ckpt
        self.model.load_state_dict(state_dict)
        self.model.eval()
        rospy.loginfo('模型权重已加载（设备 %s），节点启动即推流控机', self.device)
        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/drone/front_camera/image_raw', Image,
                         self.on_image, queue_size=1)

    def on_image(self, msg):
        """图像 -> 预处理 -> 前向推理 -> 约束 -> 直接发布 Twist"""
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        tensor = preprocess_image(img).unsqueeze(0).to(self.device)
        with torch.no_grad():
            action = self.model(tensor)[0].cpu().numpy()
        vx, vy, yaw = float(action[0]), float(action[1]), float(action[3])
        vz = 0.0                                   # 巡航高度锁死
        vy = max(-0.2, min(0.2, vy))               # 侧移软限幅 ±0.2 m/s
        yaw = max(-0.1, min(0.1, yaw))             # 偏航软限幅 ±0.1 rad/s
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = vx, vy, vz
        twist.angular.z = yaw
        self.cmd_pub.publish(twist)
        rospy.loginfo_throttle(1.0, '下发速度: vx=%.2f vy=%.2f vz=%.2f yaw=%.2f',
                               vx, vy, vz, yaw)


if __name__ == '__main__':
    AutonomousDrone()
    rospy.spin()
