#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
前置相机节点（任务 4 的感知层）

从 AirSim 的 front_center 相机取图，发布：
  /drone/image_raw   sensor_msgs/Image  bgr8
  /drone/depth       sensor_msgs/Image  32FC1（米）

端到端 DQN 节点订阅 /drone/image_raw；depth 用于调试与建图对比实验。
"""

import os
import sys

import numpy as np
import rospy
from sensor_msgs.msg import Image

_client_dir = os.environ.get('AIRSIM_PYTHON_CLIENT')
if _client_dir and _client_dir not in sys.path:
    sys.path.insert(0, _client_dir)

import airsim  # noqa: E402


def bgr_frame(response):
    """AirSim 不同版本可能返回 BGR 或 BGRA。"""
    img = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
    pixels = response.height * response.width
    if img.size == pixels * 4:
        return img.reshape(response.height, response.width, 4)[:, :, :3].copy()
    if img.size == pixels * 3:
        return img.reshape(response.height, response.width, 3)
    raise ValueError('相机图像尺寸不匹配: 收到 %d 字节，期望 %d 或 %d' %
                     (img.size, pixels * 3, pixels * 4))


class CameraNode(object):
    def __init__(self):
        self.ip = rospy.get_param('~airsim_ip', '172.25.176.1')
        self.vehicle = rospy.get_param('~vehicle_name', 'Drone_1')
        self.camera = rospy.get_param('~camera_name', 'front_center')
        self.rate_hz = float(rospy.get_param('~rate', 10.0))

        self.client = airsim.MultirotorClient(ip=self.ip, timeout_value=10)
        self.client.confirmConnection()
        vehicles = self.client.listVehicles()
        if self.vehicle not in vehicles:
            raise RuntimeError(
                'AirSim 中没有机体 %s（当前车辆: %s）。请用 -settings 指向本模块 '
                'config/settings.json 启动 Blocks。' % (self.vehicle, vehicles))

        self.rgb_pub = rospy.Publisher('/drone/image_raw', Image, queue_size=2)
        self.depth_pub = rospy.Publisher('/drone/depth', Image, queue_size=2)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                responses = self.client.simGetImages([
                    airsim.ImageRequest(self.camera, airsim.ImageType.Scene, False, False),
                    airsim.ImageRequest(self.camera, airsim.ImageType.DepthPlanar, True, False),
                ], vehicle_name=self.vehicle)

                rgb = responses[0]
                if rgb and rgb.image_data_uint8:
                    try:
                        frame = bgr_frame(rgb)
                    except ValueError as exc:
                        rospy.logerr_throttle(5.0, '%s', exc)
                    else:
                        msg = Image()
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = 'camera_link'
                        msg.height = rgb.height
                        msg.width = rgb.width
                        msg.encoding = 'bgr8'
                        msg.step = rgb.width * 3
                        msg.data = frame.tobytes()
                        self.rgb_pub.publish(msg)

                depth = responses[1]
                if depth and depth.image_data_float:
                    d = np.array(depth.image_data_float, dtype=np.float32)
                    if d.size == depth.height * depth.width:
                        msg = Image()
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = 'camera_link'
                        msg.height = depth.height
                        msg.width = depth.width
                        msg.encoding = '32FC1'
                        msg.step = depth.width * 4
                        msg.data = d.tobytes()
                        self.depth_pub.publish(msg)
            except Exception as exc:  # pragma: no cover
                rospy.logwarn_throttle(5.0, '相机取图失败: %s', exc)
            rate.sleep()


def main():
    rospy.init_node('camera_node')
    CameraNode().run()


if __name__ == '__main__':
    main()
