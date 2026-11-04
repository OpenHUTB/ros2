#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""坐标、点云与桥接状态工具（纯函数，不依赖 ROS，可离线回归测试）。

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


# --------------------------------------------------------------------------- #
# 桥接层状态字符串
# --------------------------------------------------------------------------- #
def status_token(text):
    """从桥接层状态字符串里取出**状态词**（第一个空白分隔的词），大写。

    ``carlair_ros_bridge`` 的 ``/uav/status`` 并不总是只发一个词：
    ``cmd_vel_sub.py`` 到达目标点时发的是::

        "GOAL_DONE %.2f %.2f %.2f" % target      # 例：GOAL_DONE -1.43 0.14 -20.08

    即状态词后面还带三个坐标。所以判断状态必须取第一个词，
    用 ``text == "GOAL_DONE"`` 这种全串比较在真机上永远不成立
    （实测：目标点下发后一直等不到，直到 ``approach/goal_timeout`` 超时进 ABORT）。

    空串、全空白、``None`` 都返回 ``""``，调用方不必再做保护。
    """
    if not text:
        return ""
    parts = str(text).strip().split()
    return parts[0].upper() if parts else ""


def status_values(text):
    """取出状态字符串里状态词**后面的数字**（没有则返回空元组）。

    ``carlair_ros_bridge`` 到达目标点时发的是::

        "GOAL_DONE %.2f %.2f %.2f" % target

    这三个数就是桥接**实际完成的那个目标点**。把它和本节点下发的目标点比对，
    就能分辨这条 ``GOAL_DONE`` 到底是不是自己的 —— 实测踩过的坑：先手动
    ``rostopic pub /uav/goal`` 让飞机爬升，再把本节点拉起来，本节点会把那次
    爬升的 ``GOAL_DONE`` 当成「我到了」，于是**在错误的位置原地下降**，
    直到 ``descend/timeout`` 超时才 ABORT。
    """
    out = []
    for part in str(text or "").strip().split()[1:]:
        try:
            out.append(float(part))
        except ValueError:
            pass
    return tuple(out)


def goal_matches(status_values_xyz, sent_goal, tol=0.5):
    """带坐标的 ``GOAL_DONE`` 是否对应本节点下发的目标点。

    * 状态里**没有**坐标（旧版桥接只发一个裸词）-> 无从区分来源，返回 True，
      保持兼容，行为与只看状态词一致；
    * 有坐标 -> 三个分量都要落在 ``tol`` 米以内才算自己的。
    """
    if not status_values_xyz or sent_goal is None:
        return True
    if len(status_values_xyz) < 3 or len(sent_goal) < 3:
        return True
    return all(abs(float(a) - float(b)) <= abs(tol)
               for a, b in zip(status_values_xyz[:3], sent_goal[:3]))
