#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PointCloud2 解码工具。

桥接层发布的 XYZ 字段为小端 float32，坐标已经转换到世界 ENU。
本模块只依赖 numpy，便于在虚拟机中运行和离线测试。
"""

from __future__ import annotations

import numpy as np


_FLOAT32 = 7
_FLOAT64 = 8


def pointcloud2_to_xyz(msg):
    """把 sensor_msgs/PointCloud2 转换为 (N, 3) float64 数组."""
    if msg is None:
        return np.empty((0, 3), dtype=np.float64)

    width = int(getattr(msg, "width", 0))
    height = int(getattr(msg, "height", 0))
    point_step = int(getattr(msg, "point_step", 0))
    row_step = int(getattr(msg, "row_step", 0))
    if width <= 0 or height <= 0 or point_step <= 0:
        return np.empty((0, 3), dtype=np.float64)
    if row_step <= 0:
        row_step = width * point_step

    fields = {field.name: field for field in getattr(msg, "fields", [])}
    missing = {"x", "y", "z"} - set(fields)
    if missing:
        raise ValueError("PointCloud2 缺少字段: %s" % ", ".join(sorted(missing)))

    datatypes = {int(fields[name].datatype) for name in ("x", "y", "z")}
    if datatypes == {_FLOAT32}:
        dtype = np.dtype("<f4")
    elif datatypes == {_FLOAT64}:
        dtype = np.dtype("<f8")
    else:
        raise ValueError("仅支持 float32/float64 XYZ 点云，收到 %r" % datatypes)

    itemsize = dtype.itemsize
    if row_step % itemsize or point_step % itemsize:
        raise ValueError("row_step/point_step 与字段宽度不对齐")

    words_per_row = row_step // itemsize
    flat = np.frombuffer(msg.data, dtype=dtype)
    required = height * words_per_row
    if flat.size < required:
        raise ValueError(
            "PointCloud2 数据长度不足: got=%d expected>=%d"
            % (flat.size, required)
        )

    rows = flat[:required].reshape(height, words_per_row)
    point_words = point_step // itemsize
    if words_per_row < width * point_words:
        raise ValueError("PointCloud2 row_step 小于有效点数据长度")
    rows = rows[:, : width * point_words].reshape(height, width, point_words)
    columns = []
    for name in ("x", "y", "z"):
        offset = int(fields[name].offset)
        if offset % itemsize:
            raise ValueError("%s 字段偏移未按数据类型对齐" % name)
        columns.append(rows[:, :, offset // itemsize])

    xyz = np.stack(columns, axis=-1)
    xyz = xyz.reshape(-1, 3).astype(np.float64, copy=False)
    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite]
