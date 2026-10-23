#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""numpy ↔ ROS 消息的转换（字节序、step、field 布局、坐标系等易错点集中于此）。"""
from __future__ import annotations

import array

import numpy as np

import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
import tf2_msgs.msg


def _as_byte_array(buf):
    """bytes → ``array.array('B')``；rclpy 的 ``uint8[]`` setter 收 ``bytes`` 会逐元素转换（230 KB 实测 84 ms，array('B') 仅 0.08 ms）。"""
    a = array.array("B")
    a.frombytes(buf)
    return a


def make_header(stamp, frame_id):
    h = std_msgs.msg.Header()
    h.stamp = stamp
    h.frame_id = frame_id
    return h


def to_odometry(msg, stamp, world_frame, body_frame):
    """把仿真返回的 odom 字典转成 nav_msgs/Odometry（ENU）。"""
    o = msg["odom"]
    m = nav_msgs.msg.Odometry()
    m.header = make_header(stamp, world_frame)
    m.child_frame_id = body_frame

    p = o["pos"]
    q = o["quat"]                       # (w, x, y, z)
    m.pose.pose.position.x = float(p[0])
    m.pose.pose.position.y = float(p[1])
    m.pose.pose.position.z = float(p[2])
    m.pose.pose.orientation.w = float(q[0])
    m.pose.pose.orientation.x = float(q[1])
    m.pose.pose.orientation.y = float(q[2])
    m.pose.pose.orientation.z = float(q[3])

    v = o["vel"]
    m.twist.twist.linear.x = float(v[0])
    m.twist.twist.linear.y = float(v[1])
    m.twist.twist.linear.z = float(v[2])
    w = o["omega"]
    m.twist.twist.angular.x = float(w[0])
    m.twist.twist.angular.y = float(w[1])
    m.twist.twist.angular.z = float(w[2])
    return m


def to_tf(msg, stamp, world_frame, body_frame):
    """world → body 的 TF。"""
    o = msg["odom"]
    p, q = o["pos"], o["quat"]
    t = geometry_msgs.msg.TransformStamped()
    t.header = make_header(stamp, world_frame)
    t.child_frame_id = body_frame
    t.transform.translation.x = float(p[0])
    t.transform.translation.y = float(p[1])
    t.transform.translation.z = float(p[2])
    t.transform.rotation.w = float(q[0])
    t.transform.rotation.x = float(q[1])
    t.transform.rotation.y = float(q[2])
    t.transform.rotation.z = float(q[3])
    tm = tf2_msgs.msg.TFMessage()
    tm.transforms = [t]
    return tm


def to_image(arr, stamp, frame_id, encoding):
    """numpy 数组 → sensor_msgs/Image；``step`` 取实际字节宽度（深度图 32FC1 为 width*4）。"""
    a = np.ascontiguousarray(arr)
    m = sensor_msgs.msg.Image()
    m.header = make_header(stamp, frame_id)
    m.height = int(a.shape[0])
    m.width = int(a.shape[1])
    m.encoding = encoding
    m.is_bigendian = False
    m.step = int(a.strides[0])
    m.data = _as_byte_array(a.tobytes())      # ← 走 array('B')，见 _as_byte_array
    return m


def to_pointcloud2(points, stamp, frame_id):
    """(N,3) float32 点云 → sensor_msgs/PointCloud2（XYZ 三段 float32）。"""
    pts = np.ascontiguousarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("点云必须是 (N,3)")
    n = int(pts.shape[0])

    m = sensor_msgs.msg.PointCloud2()
    m.header = make_header(stamp, frame_id)
    m.height = 1
    m.width = n
    m.is_bigendian = False
    m.is_dense = True
    m.point_step = 12                      # 3 × float32
    m.row_step = 12 * n

    fields = []
    for i, name in enumerate(("x", "y", "z")):
        f = sensor_msgs.msg.PointField()
        f.name = name
        f.offset = 4 * i
        f.datatype = sensor_msgs.msg.PointField.FLOAT32
        f.count = 1
        fields.append(f)
    m.fields = fields
    m.data = _as_byte_array(pts.tobytes())    # ← 同上
    return m


def make_twist(vx, vy, vz, wz):
    t = geometry_msgs.msg.Twist()
    t.linear.x = float(vx)
    t.linear.y = float(vy)
    t.linear.z = float(vz)
    t.angular.z = float(wz)
    return t
