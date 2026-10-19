#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hutb_pilotnet_e2e: 端到端模型训练数据采集节点
==================================
订阅 TurtleBot3 Waffle Pi 单目相机图像 (/camera/rgb/image_raw)
同步记录键盘驾驶产生的控制指令 (/cmd_vel), 保存为
    data/training/images.npy       (N, 160, 96, 3)  uint8
    data/training/steerings.npy    (N,)
    data/training/speeds.npy       (N,)
用法:
  roslaunch hutb_pilotnet_e2e main.launch mode:=collect
  然后在运行 teleop 键盘节点的终端里驾驶机器人漫游
"""
import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

IMG_W, IMG_H = 160, 96
SAMPLE_RATE = 5.0          # 每秒采样帧数


class DataCollector:
    def __init__(self):
        self.save_dir = rospy.get_param('~save_dir',
                                        os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                                    'data', 'training'))
        self.bridge = CvBridge()
        self.last_img = None
        self.last_cmd = np.zeros(2)
        self.images, self.steers, self.speeds = [], [], []
        self.cnt = 0
        self.last_save = 0
        rospy.Subscriber(rospy.get_param('~image_topic', '/camera/rgb/image_raw'),
                         Image, self.img_cb, queue_size=1)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / SAMPLE_RATE), self.sample)

    def img_cb(self, msg):
        try:
            self.last_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def cmd_cb(self, msg):
        self.last_cmd = np.array([msg.linear.x, msg.angular.z])

    def sample(self, _):
        if self.last_img is None:
            return
        img = cv2.resize(self.last_img, (IMG_W, IMG_H))
        # 保留下半部(路面区域)以模拟 PilotNet 预处理, 这里保留全图用于可视化
        self.images.append(img)
        self.steers.append(self.last_cmd[1])
        self.speeds.append(self.last_cmd[0])
        self.cnt += 1
        # 每 100 帧自动落盘一次, 防止进程被强杀时数据丢失
        if self.cnt - self.last_save >= 100:
            self.last_save = self.cnt
            self.save()
        if self.cnt % 50 == 0:
            rospy.loginfo('已采集 %d 帧' % self.cnt)

    def save(self):
        if not self.images:
            rospy.logwarn('无数据, 未保存')
            return
        os.makedirs(self.save_dir, exist_ok=True)
        np.save(os.path.join(self.save_dir, 'images.npy'), np.array(self.images, dtype=np.uint8))
        np.save(os.path.join(self.save_dir, 'steerings.npy'), np.array(self.steers, dtype=np.float32))
        np.save(os.path.join(self.save_dir, 'speeds.npy'), np.array(self.speeds, dtype=np.float32))
        rospy.loginfo('数据已保存到 %s (共 %d 帧)' % (self.save_dir, len(self.images)))


def main():
    rospy.init_node('data_collector')
    node = DataCollector()
    rospy.on_shutdown(node.save)
    rospy.loginfo('数据采集启动: 请在 teleop 终端驾驶机器人漫游 (Ctrl+C 结束采集)')
    rospy.spin()


if __name__ == '__main__':
    main()
