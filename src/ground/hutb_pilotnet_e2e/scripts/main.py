#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hutb_pilotnet_e2e: 端到端模型实时推理节点
==============================
输入: 前视相机图像 (160x96x3)
输出: /cmd_vel (线速度 + 角速度), 即"输入图像、输出控制指令"
模型: PilotNet CNN (models/pilotnet.pt)

用法:
  roslaunch hutb_pilotnet_e2e main.launch mode:=inference
"""
import os
import sys

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import torch

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PKG_DIR, 'models'))
from pilotnet import PilotNet

IMG_W, IMG_H = 160, 96
STEER_SCALE = 1.0 / 1.5
SPEED_SCALE = 1.0 / 0.22


class End2EndNode:
    def __init__(self):
        weights = rospy.get_param('~weights',
                                  os.path.join(PKG_DIR, 'models', 'pilotnet.pt'))
        self.bridge = CvBridge()
        self.model = PilotNet()
        state = torch.load(weights, map_location='cpu')
        self.model.load_state_dict(state)
        self.model.eval()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber(rospy.get_param('~image_topic', '/camera/rgb/image_raw'),
                         Image, self.cb, queue_size=1)
        self.sim_time = rospy.Time.now()
        rospy.loginfo('端到端模型加载完成: %s' % weights)

    def cb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        img = cv2.resize(bgr, (IMG_W, IMG_H))[:, :, ::-1]      # BGR->RGB
        x = (img.astype(np.float32) / 127.5 - 1.0).transpose(2, 0, 1)
        x = torch.from_numpy(x).unsqueeze(0)
        with torch.no_grad():
            out = self.model(x)[0].numpy()
        steer = float(out[0]) / STEER_SCALE
        speed = float(out[1]) / SPEED_SCALE
        # 限幅
        steer = max(-1.5, min(1.5, steer))
        speed = max(0.0, min(0.22, speed))
        t = Twist()
        t.linear.x = speed
        t.angular.z = steer
        self.pub.publish(t)

        # 周期打印
        now = rospy.Time.now()
        if (now - self.sim_time).to_sec() > 1.0:
            rospy.loginfo('端到端输出: v=%.2f m/s, w=%.2f rad/s' % (speed, steer))
            self.sim_time = now


def main():
    rospy.init_node('end2end_inference')
    End2EndNode()
    rospy.spin()


if __name__ == '__main__':
    main()
