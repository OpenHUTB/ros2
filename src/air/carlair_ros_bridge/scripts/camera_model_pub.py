#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机模型节点：CameraInfo、相机 TF 与深度点云.

订阅：
    /camera/image_raw   sensor_msgs/Image  bgr8
    /camera/depth       sensor_msgs/Image  32FC1（米）

发布：
    /camera/camera_info        sensor_msgs/CameraInfo
    /camera/depth/camera_info  sensor_msgs/CameraInfo
    /camera/depth/points       sensor_msgs/PointCloud2（相机光学系）

TF：
    base_link -> front_rgb / front_depth（静态，安装偏移可配）

参数（默认值见 config/bridge.yaml）：
    camera/fov_degrees, camera/rgb_frame, camera/depth_frame
    camera/mount_x, mount_y, mount_z
    topic/image, topic/depth, topic/camera_info,
    topic/depth_info, topic/depth_points
    frame/body
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_client import matrix_to_quat_xyzw  # noqa: E402


def make_camera_info(width, height, fov_degrees, frame_id, stamp, seq=0):
    """按针孔模型构造 CameraInfo（方形像素，主点在图像中心）."""
    width = int(width)
    height = int(height)
    fx = width / (2.0 * math.tan(math.radians(fov_degrees) / 2.0))
    cx = width / 2.0
    cy = height / 2.0

    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.header.seq = seq
    msg.width = width
    msg.height = height
    msg.distortion_model = "plumb_bob"
    msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.K = [fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0]
    msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.P = [fx, 0.0, cx, 0.0, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def depth_to_points(depth, fov_degrees, max_depth=100.0):
    """深度图（米）-> 相机光学系点云 (N, 3).

    针孔反投影：
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        Z = depth
    其中 depth 是 DepthPlanar 的沿光轴深度，可直接作为 Z。
    """
    depth = np.asarray(depth, dtype=np.float32)
    height, width = depth.shape
    fx = width / (2.0 * math.tan(math.radians(fov_degrees) / 2.0))
    cx = width / 2.0
    cy = height / 2.0

    valid = np.isfinite(depth) & (depth > 0.0) & (depth < max_depth)
    v, u = np.nonzero(valid)
    z = depth[v, u]
    x = (u.astype(np.float32) - cx) * z / fx
    y = (v.astype(np.float32) - cy) * z / fx
    return np.column_stack((x, y, z)).astype(np.float32)


def make_pointcloud2(points, frame_id, stamp, seq=0):
    """XYZ float32 -> PointCloud2（point_step=12，小端）."""
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.header.seq = seq
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
    return msg


class CameraModelPublisher(object):
    def __init__(self):
        self.base_frame = rospy.get_param("frame/body", "base_link")
        self.rgb_frame = rospy.get_param("camera/rgb_frame", "front_rgb")
        self.depth_frame = rospy.get_param("camera/depth_frame", "front_depth")
        self.fov = float(rospy.get_param("camera/fov_degrees", 90.0))
        self.mount = (
            float(rospy.get_param("camera/mount_x", 0.3)),
            float(rospy.get_param("camera/mount_y", 0.0)),
            float(rospy.get_param("camera/mount_z", 0.1)),
        )

        self.topic_rgb = rospy.get_param("topic/image", "/camera/image_raw")
        self.topic_depth = rospy.get_param("topic/depth", "/camera/depth")
        self.topic_info = rospy.get_param("topic/camera_info", "/camera/camera_info")
        self.topic_depth_info = rospy.get_param("topic/depth_info", "/camera/depth/camera_info")
        self.topic_points = rospy.get_param("topic/depth_points", "/camera/depth/points")

        self.pub_info = rospy.Publisher(self.topic_info, CameraInfo, queue_size=2)
        self.pub_depth_info = rospy.Publisher(self.topic_depth_info, CameraInfo, queue_size=2)
        self.pub_points = rospy.Publisher(self.topic_points, PointCloud2, queue_size=2)

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.seq = 0
        self.send_static_tf()

        self.sub_rgb = rospy.Subscriber(self.topic_rgb, Image, self.rgb_cb, queue_size=1)
        self.sub_depth = rospy.Subscriber(self.topic_depth, Image, self.depth_cb, queue_size=1)
        rospy.loginfo("camera_model_pub: fov=%.1f, frames=%s/%s",
                      self.fov, self.rgb_frame, self.depth_frame)

    def send_static_tf(self):
        # ROS 机体 FLU -> 相机光学系（z 前、x 右、y 下）
        R = np.array([[0.0, -1.0, 0.0],
                      [0.0, 0.0, -1.0],
                      [1.0, 0.0, 0.0]], dtype=np.float64)
        qx, qy, qz, qw = matrix_to_quat_xyzw(R)
        stamp = rospy.Time.now()
        transforms = []
        for child in (self.rgb_frame, self.depth_frame):
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.base_frame
            tf.child_frame_id = child
            tf.transform.translation.x = self.mount[0]
            tf.transform.translation.y = self.mount[1]
            tf.transform.translation.z = self.mount[2]
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            transforms.append(tf)
        self.tf_broadcaster.sendTransform(transforms)

    def rgb_cb(self, msg):
        stamp = rospy.Time.now()
        self.pub_info.publish(
            make_camera_info(msg.width, msg.height, self.fov, self.rgb_frame,
                             stamp, self.seq))

    def depth_cb(self, msg):
        stamp = rospy.Time.now()
        self.pub_depth_info.publish(
            make_camera_info(msg.width, msg.height, self.fov, self.depth_frame,
                             stamp, self.seq))
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        points = depth_to_points(depth, self.fov)
        self.pub_points.publish(
            make_pointcloud2(points, self.depth_frame, stamp, self.seq))
        self.seq += 1


def main():
    rospy.init_node("uav_camera_model_pub", anonymous=False)
    CameraModelPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
