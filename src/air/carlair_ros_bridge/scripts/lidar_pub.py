#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""激光雷达发布节点：CarlaAir/AirSim LiDAR -> /lidar/points (sensor_msgs/PointCloud2).

发布：
    /lidar/points   sensor_msgs/PointCloud2   XYZ(float32) 世界系（ENU）点云

坐标系
------
AirSim 的 `LidarData.point_cloud` 所在坐标系由 settings.json 中该雷达的
`DataFrame` 决定：
    * `VehicleInertialFrame`（默认）：载具惯性系（NED 世界系，米），
      与 `/uav/odom` 同源，桥接层只做 NED -> ENU 轴变换；
    * `SensorLocalFrame`：雷达局部系，需用 `LidarData.pose` 先变换到惯性系。
本包的 `config/settings.json` 采用 **VehicleInertialFrame**，这样点云、里程计、
目标点全部落在同一个 `world`(ENU) 坐标系里，后续建图（octomap）与路径规划
不需要额外的外参标定。

PointCloud2 二进制布局
----------------------
每个点 3 个 float32，依次为 x、y、z，故
    point_step = 3 * 4 = 12 字节
    row_step   = point_step * width
    height     = 1（无序点云，按一维数组存储）
    data       = concatenate(x, y, z) 的小端 float32 字节流
这与 `sensor_msgs.point_cloud2.create_cloud_xyz32()` 的输出完全一致，
但这里手写以便在无 ROS 环境下单元测试字节布局。

参数（默认值见 config/bridge.yaml）：
    sim/host, sim/airsim_port, sim/vehicle_name
    rate/lidar_hz, sensor/lidar_name, sensor/lidar_frame
    lidar/min_range, lidar/max_range, lidar/voxel_leaf
    frame/world, topic/lidar
"""
from __future__ import annotations

import os
import sys

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_client import SimClient  # noqa: E402

# sensor_msgs/PointField 的 datatype 取值（ROS 消息定义中的枚举）
PF_FLOAT32 = 7
FLOAT32_SIZE = 4


# --------------------------------------------------------------------- 纯函数
def filter_by_range(points: np.ndarray, min_range: float = 0.0,
                    max_range: float = 0.0) -> np.ndarray:
    """按到原点的距离做环形滤波（0 表示不启用该侧）."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] == 0 or (min_range <= 0.0 and max_range <= 0.0):
        return pts
    r = np.linalg.norm(pts, axis=1)
    keep = np.ones(pts.shape[0], dtype=bool)
    if min_range > 0.0:
        keep &= r >= min_range
    if max_range > 0.0:
        keep &= r <= max_range
    return pts[keep]


def voxel_downsample(points: np.ndarray, leaf: float = 0.0) -> np.ndarray:
    """体素栅格降采样：每个 leaf^3 立方体内只保留一个点（取均值）.

    用整数坐标 (floor(p / leaf)) 作为体素索引，再用 np.unique 分组，
    复杂度 O(N log N)，无需第三方点云库，便于在虚拟机内运行。
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if leaf <= 0.0 or pts.shape[0] == 0:
        return pts
    keys = np.floor(pts / float(leaf)).astype(np.int64)
    _uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.ravel()
    counts = np.bincount(inverse, minlength=int(inverse.max()) + 1).astype(np.float64)
    sums = np.zeros((counts.size, 3), dtype=np.float64)
    np.add.at(sums, inverse, pts)
    return sums / counts[:, None]


def pack_xyz32(points: np.ndarray) -> bytes:
    """N x 3 点 -> 小端 float32 连续字节流（point_step = 12）."""
    arr = np.ascontiguousarray(np.asarray(points, dtype="<f4").reshape(-1, 3))
    return arr.tobytes()


def fields_xyz():
    """PointCloud2 字段描述：x@0, y@4, z@8，均为 FLOAT32."""
    return [PointField(name="x", offset=0, datatype=PF_FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PF_FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PF_FLOAT32, count=1)]


def make_pointcloud2(points: np.ndarray, frame_id: str, stamp,
                     seq: int = 0) -> PointCloud2:
    """把 ENU 点云打包成 sensor_msgs/PointCloud2（无 TFs 外参，坐标系即 world）."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    n = int(pts.shape[0])

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.header.seq = seq
    msg.height = 1
    msg.width = n
    msg.fields = fields_xyz()
    msg.is_bigendian = False
    msg.point_step = 3 * FLOAT32_SIZE
    msg.row_step = msg.point_step * n
    msg.data = pack_xyz32(pts)
    msg.is_dense = bool(n == 0 or np.isfinite(pts).all())
    return msg


# --------------------------------------------------------------------- 节点
class LidarPublisher(object):
    def __init__(self):
        host = rospy.get_param("sim/host", "127.0.0.1")
        port = int(rospy.get_param("sim/airsim_port", 41451))
        vehicle = rospy.get_param("sim/vehicle_name", "")
        rate_hz = float(rospy.get_param("rate/lidar_hz", 10.0))

        self.lidar_name = rospy.get_param("sensor/lidar_name", "lidar1")
        self.lidar_frame = rospy.get_param("sensor/lidar_frame", "vehicle_inertial")
        self.min_range = float(rospy.get_param("lidar/min_range", 0.0))
        self.max_range = float(rospy.get_param("lidar/max_range", 0.0))
        self.voxel_leaf = float(rospy.get_param("lidar/voxel_leaf", 0.0))

        topic = rospy.get_param("topic/lidar", "/lidar/points")
        self.world_frame = rospy.get_param("frame/world", "world")

        self.sim = SimClient(host=host, port=port, vehicle_name=vehicle)
        self.sim.connect()
        rospy.loginfo("lidar_pub: 已连接仿真器 %s:%d，雷达 %s (DataFrame=%s)",
                      host, port, self.lidar_name, self.lidar_frame)

        self.pub = rospy.Publisher(topic, PointCloud2, queue_size=2)
        self.rate = rospy.Rate(rate_hz)
        self.seq = 0
        self.fail_count = 0
        self.total_points = 0

    def spin(self):
        while not rospy.is_shutdown():
            try:
                self.step_once()
                self.fail_count = 0
            except Exception as exc:  # noqa: BLE001 - 仿真断连时不能让节点崩溃
                self.fail_count += 1
                if self.fail_count in (1, 10, 100):
                    rospy.logwarn("lidar_pub: 读取点云失败(第 %d 次): %s",
                                  self.fail_count, exc)
            self.rate.sleep()

    def process(self, points: np.ndarray) -> np.ndarray:
        """滤波 + 降采样（顺序：先环形滤波再去重，减少体素分组的点数）."""
        pts = filter_by_range(points, self.min_range, self.max_range)
        return voxel_downsample(pts, self.voxel_leaf)

    def step_once(self):
        """取一帧并发布（抽出为独立方法，便于无 ROS 的本地测试）."""
        pts = self.sim.get_lidar_points_enu(self.lidar_name, frame=self.lidar_frame)
        if pts is None:
            return
        pts = self.process(pts)
        self.pub.publish(make_pointcloud2(pts, self.world_frame,
                                          rospy.Time.now(), self.seq))
        self.seq += 1
        self.total_points = int(pts.shape[0])


def main():
    rospy.init_node("uav_lidar_pub", anonymous=False)
    node = LidarPublisher()
    node.spin()


if __name__ == "__main__":
    main()
