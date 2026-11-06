#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
激光雷达感知节点（任务 2 的感知部分）

从 AirSim 读取 lidar_1 的点云，做两件事：
  1. 转成标准 ROS 的 sensor_msgs/LaserScan，方便在 RViz 里看
  2. 压成 12 个扇区的最近距离，发到 /uav/lidar/features 给控制节点

点云坐标系为传感器系（x 前、y 右、z 下），视场角 ±30°。
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

from uav_features import (  # noqa: E402
    FOV_DEG, MAX_RANGE, NUM_SECTORS, extract_scan, extract_sector_features,
)

_client_path = os.environ.get('AIRSIM_PYTHON_CLIENT')
if _client_path and _client_path not in sys.path:
    sys.path.insert(0, _client_path)

import airsim  # noqa: E402


class LidarPerceptionNode(object):
    def __init__(self):
        self.ip = rospy.get_param('~airsim_ip', '172.25.176.1')
        self.vehicle = rospy.get_param('~vehicle_name', 'Drone_1')
        self.lidar_name = rospy.get_param('~lidar_name', 'lidar_1')
        self.rate_hz = float(rospy.get_param('~rate', 10.0))
        self.scan_bins = int(rospy.get_param('~scan_bins', 180))
        self.scan_fov = float(rospy.get_param('~scan_fov', 360.0))
        # 扫描帧必须和 gmapping 的 base_frame 区分开，否则会卡在等 TF 上
        self.scan_frame = rospy.get_param('~scan_frame', 'laser')
        # 建图扫描的来源：lidar（稀疏但全向）或 depth（稠密但只有前向 90 度）
        self.scan_source = rospy.get_param('~scan_source', 'lidar')
        self.camera = rospy.get_param('~camera_name', 'front_center')
        self.depth_fov = float(rospy.get_param('~camera_fov', 90.0))

        rospy.loginfo('连接 AirSim %s', self.ip)
        self.client = airsim.MultirotorClient(ip=self.ip, timeout_value=10)
        self.client.confirmConnection()

        # 旋转式雷达每帧只吐出一小段扫描，累积一段时间才能拼出完整一圈
        self.accum_sec = float(rospy.get_param('~accumulate_sec', 1.0))
        self.point_buffer = collections.deque()

        self.scan_pub = rospy.Publisher('/uav/lidar/scan', LaserScan, queue_size=5)
        self.feat_pub = rospy.Publisher('/uav/lidar/features', Float32MultiArray,
                                        queue_size=5)

    def depth_to_points(self, max_dist=40.0):
        """把深度图反投影成 3D 点（传感器系：x 前、y 右、z 下）。"""
        resp = self.client.simGetImages(
            [airsim.ImageRequest(self.camera, airsim.ImageType.DepthPlanar, True, False)],
            vehicle_name=self.vehicle)[0]
        if resp is None or not resp.image_data_float:
            return np.zeros((0, 3), dtype=np.float32)

        depth = np.array(resp.image_data_float, dtype=np.float32)
        depth = depth.reshape(resp.height, resp.width)
        h, w = depth.shape

        fx = (w / 2.0) / math.tan(math.radians(self.depth_fov) / 2.0)
        fy = fx
        cx, cy = w / 2.0, h / 2.0

        us, vs = np.meshgrid(np.arange(w), np.arange(h))
        z_cam = depth                              # 相机系前向深度
        x_cam = (us - cx) * z_cam / fx             # 右
        y_cam = (vs - cy) * z_cam / fy             # 下

        valid = (z_cam > 0.2) & (z_cam < max_dist)
        # 相机系 (x右, y下, z前) -> 传感器系 (x前, y右, z下)
        pts = np.stack([z_cam[valid], x_cam[valid], y_cam[valid]], axis=1)
        # 只保留大致同高度的一层，做二维建图
        pts = pts[np.abs(pts[:, 2]) < 2.0]
        return pts.astype(np.float32)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                lidar = self.client.getLidarData(lidar_name=self.lidar_name,
                                                 vehicle_name=self.vehicle)
                raw = np.array(lidar.point_cloud, dtype=np.float32)
                points = raw.reshape(-1, 3) if raw.size else np.zeros((0, 3), np.float32)

                # 控制用的特征只用最新一帧，保持和训练时一致
                feats = extract_sector_features(points)

                # 建图用的扫描用累积点云，拼出完整一圈
                now = rospy.Time.now().to_sec()
                if len(points):
                    self.point_buffer.append((now, points))
                while self.point_buffer and now - self.point_buffer[0][0] > self.accum_sec:
                    self.point_buffer.popleft()
                if self.point_buffer:
                    accum = np.vstack([p for _, p in self.point_buffer])
                else:
                    accum = points

                if self.scan_source == 'depth':
                    # 深度图点云稠密，直接用它拼扫描（前向 FOV 与相机一致）
                    dpts = self.depth_to_points()
                    if len(dpts):
                        ranges = extract_scan(dpts, bins=self.scan_bins,
                                              fov_deg=self.depth_fov,
                                              max_range=MAX_RANGE)
                    else:
                        ranges = np.full(self.scan_bins, MAX_RANGE, dtype=np.float32)
                else:
                    ranges = extract_scan(accum, bins=self.scan_bins,
                                          fov_deg=self.scan_fov, max_range=MAX_RANGE)
                scan = LaserScan()
                scan.header.stamp = rospy.Time.now()
                scan.header.frame_id = self.scan_frame
                fov = self.depth_fov if self.scan_source == 'depth' else self.scan_fov
                scan.angle_min = math.radians(-fov / 2.0)
                scan.angle_max = math.radians(fov / 2.0)
                scan.angle_increment = math.radians(fov / self.scan_bins)
                scan.range_min = 0.1
                scan.range_max = MAX_RANGE
                scan.ranges = ranges.tolist()
                self.scan_pub.publish(scan)

                msg = Float32MultiArray()
                msg.data = feats.tolist()
                self.feat_pub.publish(msg)

            except Exception as exc:  # pragma: no cover - 仿真侧异常
                rospy.logwarn_throttle(5.0, '读取激光雷达失败: %s', exc)

            rate.sleep()


def main():
    rospy.init_node('lidar_perception')
    LidarPerceptionNode().run()


if __name__ == '__main__':
    main()
