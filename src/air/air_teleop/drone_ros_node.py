#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 极简桥接节点：连接 AirSim -> 自动起飞 -> 定高 3 m -> 双向透传
# cmd_vel 收到即直通下发；前视图像按 10 Hz 定时发布。无线程、无看门狗、无状态机。

import argparse
import math
import threading

import airsim
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class DroneBridge:
    def __init__(self):
        rospy.init_node('drone_ros_node')
        parser = argparse.ArgumentParser()
        parser.add_argument('--host', type=str, default='192.168.91.1')  # 宿主机 AirSim IP
        args, _ = parser.parse_known_args()
        self.client = airsim.MultirotorClient(ip=args.host)
        self.rpc_lock = threading.Lock()   # RPC 套接字互斥锁：取图/下发速度串行
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.client.takeoffAsync().join()           # 自动起飞悬停
        self.client.moveToZAsync(-3.0, 1.5).join()  # NED 定高 3 m，避开地面护栏
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/drone/front_camera/image_raw',
                                         Image, queue_size=1)
        self.cmd_sub = rospy.Subscriber('/drone/cmd_vel', Twist,
                                        self.on_cmd, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.publish_image)   # 10 Hz 推图
        rospy.loginfo('已连接 AirSim：起飞并定高 3 m，开始双向透传')

    def on_cmd(self, msg):
        """Twist 直通 AirSim：0.2 s 时长速度指令，偏航按角速率（度/秒）"""
        with self.rpc_lock:
            self.client.moveByVelocityAsync(
                msg.linear.x, msg.linear.y, msg.linear.z, duration=0.2,
                yaw_mode=airsim.YawMode(is_rate=True,
                                        yaw_or_rate=math.degrees(msg.angular.z)))

    def publish_image(self, _event):
        """抓前视 Scene 图 -> 翻正 -> rgb8 发布（与训练数据同一条管道）"""
        for name in ('front_center', '0'):
            with self.rpc_lock:
                responses = self.client.simGetImages(
                    [airsim.ImageRequest(name, airsim.ImageType.Scene,
                                         False, False)])
            if responses and responses[0].width > 0:
                img = np.frombuffer(responses[0].image_data_uint8,
                                    np.uint8).reshape(
                    responses[0].height, responses[0].width, 3)
                self.image_pub.publish(
                    self.bridge.cv2_to_imgmsg(np.flipud(img), 'rgb8'))
                return


if __name__ == '__main__':
    DroneBridge()
    rospy.spin()
