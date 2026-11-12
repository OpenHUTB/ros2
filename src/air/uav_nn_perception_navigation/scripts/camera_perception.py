#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机感知节点（任务 4 的感知部分）

从 AirSim 读取机载相机图像，发布标准 ROS 图像话题：
    /uav/image_raw   sensor_msgs/Image   rgb8
    /uav/depth       sensor_msgs/Image  32FC1（米）

不依赖 cv_bridge，直接构造消息，减少环境依赖。
"""

import os
import sys

import numpy as np
import rospy
from sensor_msgs.msg import Image

_client_path = os.environ.get('AIRSIM_PYTHON_CLIENT')
if _client_path and _client_path not in sys.path:
    sys.path.insert(0, _client_path)

import airsim  # noqa: E402


def to_image_msg(arr, encoding, stamp):
    """把 numpy 数组打包成 sensor_msgs/Image。"""
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = 'base_link'
    msg.height, msg.width = arr.shape[0], arr.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = arr.shape[1] * arr.shape[2] * arr.dtype.itemsize if arr.ndim == 3 \
        else arr.shape[1] * arr.dtype.itemsize
    msg.data = arr.tobytes()
    return msg


class CameraPerceptionNode(object):
    def __init__(self):
        self.ip = rospy.get_param('~airsim_ip', '172.25.176.1')
        self.vehicle = rospy.get_param('~vehicle_name', 'Drone_1')
        self.camera = rospy.get_param('~camera_name', 'front_center')
        self.rate_hz = float(rospy.get_param('~rate', 10.0))
        self.publish_depth = bool(rospy.get_param('~publish_depth', True))

        rospy.loginfo('连接 AirSim %s', self.ip)
        self.client = airsim.MultirotorClient(ip=self.ip, timeout_value=10)
        self.client.confirmConnection()

        self.rgb_pub = rospy.Publisher('/uav/image_raw', Image, queue_size=5)
        self.depth_pub = rospy.Publisher('/uav/depth', Image, queue_size=5)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                stamp = rospy.Time.now()
                reqs = [airsim.ImageRequest(self.camera, airsim.ImageType.Scene,
                                            False, False)]
                if self.publish_depth:
                    reqs.append(airsim.ImageRequest(
                        self.camera, airsim.ImageType.DepthPlanar, True, False))

                responses = self.client.simGetImages(reqs, vehicle_name=self.vehicle)

                rgb = responses[0]
                img = np.frombuffer(rgb.image_data_uint8, dtype=np.uint8)
                img = img.reshape(rgb.height, rgb.width, 3)
                self.rgb_pub.publish(to_image_msg(img, 'rgb8', stamp))

                if self.publish_depth and len(responses) > 1:
                    depth = responses[1]
                    d = np.array(depth.image_data_float, dtype=np.float32)
                    d = d.reshape(depth.height, depth.width)
                    self.depth_pub.publish(to_image_msg(d, '32FC1', stamp))

            except Exception as exc:  # pragma: no cover
                rospy.logwarn_throttle(5.0, '读取相机失败: %s', exc)

            rate.sleep()


def main():
    rospy.init_node('camera_perception')
    CameraPerceptionNode().run()


if __name__ == '__main__':
    main()
