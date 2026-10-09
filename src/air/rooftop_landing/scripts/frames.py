#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""坐标与点云工具（纯函数，不依赖 ROS，可离线回归测试）。

carlair_ros_bridge 发布的世界系点云与里程计为 ENU 约定：
    x 指向东, y 指向北, z 指向上（单位 m）

> ⚠️ 实测中 `/uav/odom` 与 `/uav/goal` 的高度分量在不同仿真器配置下可能是
> 「上为正」（ENU）或「下为正」（NED）。本模块不做假设，由节点参数
> `frame/z_up` 决定，见 `config/rooftop_landing.yaml`。

本文件刻意不 `import rospy`，因此可以在没有 ROS、没有仿真器的机器上直接运行测试。
"""
from __future__ import annotations

import math
import struct

import numpy as np

# PointCloud2 字段 datatype -> (struct 格式字符, 字节数)
_PF_TO_STRUCT = {1: ("b", 1), 2: ("B", 1), 3: ("h", 2), 4: ("H", 2),
                 5: ("i", 4), 6: ("I", 4), 7: ("f", 4), 8: ("d", 8)}


def pointcloud2_to_xyz(fields, point_step, data, width=0, height=0,
                       is_bigendian=False):
    """把 sensor_msgs/PointCloud2 解析成 ``(N, 3) float32`` 数组。

    参数直接取消息属性，所以本函数不需要 ROS::

        pointcloud2_to_xyz(msg.fields, msg.point_step, msg.data,
                           msg.width, msg.height, bool(msg.is_bigendian))

    缺少 x/y/z 任一字段时返回 ``(0, 3)`` 空数组（不抛异常）。
    """
    names = {}
    for f in fields or []:
        name = getattr(f, "name", None)
        if name:
            names[name] = f
    if not all(k in names for k in ("x", "y", "z")):
        return np.zeros((0, 3), dtype=np.float32)
    if not data or not point_step:
        return np.zeros((0, 3), dtype=np.float32)

    n = len(data) // point_step
    if width and height:
        n = min(n, width * height)
    if n <= 0:
        return np.zeros((0, 3), dtype=np.float32)

    raw = bytes(data)[: n * point_step]
    arr = np.frombuffer(raw, dtype=np.uint8).reshape(n, point_step)

    prefix = ">" if is_bigendian else "<"
    out = np.empty((n, 3), dtype=np.float32)
    for i, axis in enumerate("xyz"):
        f = names[axis]
        spec = _PF_TO_STRUCT.get(int(f.datatype))
        if spec is None:
            return np.zeros((0, 3), dtype=np.float32)
        fmt, size = spec
        off = int(f.offset)
        chunk = np.ascontiguousarray(arr[:, off:off + size]).tobytes()
        try:
            values = struct.unpack(prefix + fmt * n, chunk)
        except struct.error:
            return np.zeros((0, 3), dtype=np.float32)
        out[:, i] = values
    return out


def filter_finite_xyz(points):
    """去掉 NaN/Inf 的点；输入输出都是 ``(N, 3)``。"""
    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError("points 必须是 (N,3) 数组")
    return pts[np.isfinite(pts).all(axis=1)][:, :3]


def enu_to_ned(p):
    """ENU (x东, y北, z上) -> NED (x北, y东, z下)。"""
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    return np.array([y, x, -z], dtype=np.float64)


def ned_to_enu(p):
    """NED -> ENU。"""
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    return np.array([y, x, -z], dtype=np.float64)


def yaw_from_quaternion(q):
    """从四元数 ``(x, y, z, w)`` 取偏航角（rad）。"""
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def altitude(z_raw, z_up=True):
    """把里程计/目标点的高度分量统一成「离地高度」（向上为正）。

    ``z_up=True``  表示原始值已经是 ENU 的上为正；
    ``z_up=False`` 表示原始值是 NED 的下为正，取负数。
    """
    z = float(z_raw)
    return z if z_up else -z


def to_altitude_frame(z_up_value, z_up=True):
    """把「上为正的高度」转换回该仿真器约定的原始分量，用于下发目标点。"""
    z = float(z_up_value)
    return z if z_up else -z
