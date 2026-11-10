#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
激光雷达扫描节点（任务 2/3 的感知层）

从 AirSim 读取 lidar_1 的点云，做两件事：
  1. 累积 1 秒点云后生成 LaserScan（/drone/lidar/scan），供 hector_slam 建图
  2. 把最新一帧压成 24 扇区距离，发到 /drone/lidar/sectors 供调试与可视化

AirSim 的旋转式雷达单帧只覆盖一小段水平角，所以建图扫描必须用累积点云；
控制用的扇区特征则保持单帧，避免引入时间延迟。
"""

import collections
import math
import os
import sys

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from lidar_features import (  # noqa: E402
    FOV_DEG, MAX_RANGE, extract_scan, scan_angle_max, sector_distances,
)

_client_dir = os.environ.get('AIRSIM_PYTHON_CLIENT')
if _client_dir and _client_dir not in sys.path:
    sys.path.insert(0, _client_dir)

import airsim  # noqa: E402


class LidarScanNode(object):
    def __init__(self):
        self.ip = rospy.get_param('~airsim_ip', '172.25.176.1')
        self.vehicle = rospy.get_param('~vehicle_name', 'Drone_1')
        self.lidar = rospy.get_param('~lidar_name', 'lidar_1')
        self.rate_hz = float(rospy.get_param('~rate', 10.0))
        self.bins = int(rospy.get_param('~scan_bins', 180))
        self.frame = rospy.get_param('~scan_frame', 'laser')
        self.accum_sec = float(rospy.get_param('~accumulate_sec', 1.0))

        rospy.loginfo('连接 AirSim %s', self.ip)
        self.client = airsim.MultirotorClient(ip=self.ip, timeout_value=10)
        self.client.confirmConnection()

        self.buffer = collections.deque()
        self.scan_pub = rospy.Publisher('/drone/lidar/scan', LaserScan, queue_size=5)
        self.sector_pub = rospy.Publisher('/drone/lidar/sectors',
                                          Float32MultiArray, queue_size=5)

    def _read_points(self):
        data = self.client.getLidarData(lidar_name=self.lidar,
                                        vehicle_name=self.vehicle)
        arr = np.array(data.point_cloud, dtype=np.float32)
        return arr.reshape(-1, 3) if arr.size else np.zeros((0, 3), np.float32)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                points = self._read_points()
                now = rospy.Time.now().to_sec()

                if points.shape[0]:
                    self.buffer.append((now, points))
                while self.buffer and now - self.buffer[0][0] > self.accum_sec:
                    self.buffer.popleft()
                accum = (np.vstack([p for _, p in self.buffer])
                         if self.buffer else points)

                ranges = extract_scan(accum, bins=self.bins, max_range=MAX_RANGE)
                scan = LaserScan()
                scan.header.stamp = rospy.Time.now()
                scan.header.frame_id = self.frame
                scan.angle_min = math.radians(-FOV_DEG / 2.0)
                scan.angle_increment = math.radians(FOV_DEG / self.bins)
                scan.angle_max = scan_angle_max(scan.angle_min,
                                                scan.angle_increment,
                                                self.bins)
                scan.range_min = 0.15
                scan.range_max = MAX_RANGE
                scan.ranges = ranges.tolist()
                self.scan_pub.publish(scan)

                msg = Float32MultiArray()
                msg.data = sector_distances(points).tolist()
                self.sector_pub.publish(msg)
            except Exception as exc:  # pragma: no cover
                rospy.logwarn_throttle(5.0, '雷达读取失败: %s', exc)
            rate.sleep()


def main():
    rospy.init_node('lidar_scan')
    LidarScanNode().run()


if __name__ == '__main__':
    main()
