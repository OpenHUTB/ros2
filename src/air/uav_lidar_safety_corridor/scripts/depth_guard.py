#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""前视深度图近距离保护。"""

from __future__ import annotations

import numpy as np


def depth_image_to_array(msg):
    """解析 32FC1 sensor_msgs/Image，返回 (height, width) float32 深度."""
    if msg is None:
        return np.empty((0, 0), dtype=np.float32)
    if msg.encoding != "32FC1":
        raise ValueError("深度图编码必须是 32FC1，收到 %r" % msg.encoding)

    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    if height <= 0 or width <= 0 or step < width * 4:
        return np.empty((0, 0), dtype=np.float32)

    row_words = step // 4
    flat = np.frombuffer(msg.data, dtype="<f4")
    if flat.size < height * row_words:
        raise ValueError("深度图数据长度不足")
    return flat[: height * row_words].reshape(height, row_words)[:, :width]


def central_min_depth(
    depth,
    x_min=0.2,
    x_max=0.8,
    y_min=0.25,
    y_max=0.75,
):
    """返回中心 ROI 内最近的有效深度（米）."""
    depth = np.asarray(depth, dtype=np.float64)
    if depth.ndim != 2 or depth.size == 0:
        return float("inf")
    height, width = depth.shape
    x0 = max(0, min(width, int(round(width * float(x_min)))))
    x1 = max(x0, min(width, int(round(width * float(x_max)))))
    y0 = max(0, min(height, int(round(height * float(y_min)))))
    y1 = max(y0, min(height, int(round(height * float(y_max)))))
    roi = depth[y0:y1, x0:x1]
    valid = roi[np.isfinite(roi) & (roi > 0.0)]
    if valid.size == 0:
        return float("inf")
    return float(np.min(valid))


def depth_speed_scale(depth_m, stop_distance, slow_distance):
    """把前向深度映射到 [0,1] 水平速度倍率."""
    depth_m = float(depth_m)
    stop_distance = float(stop_distance)
    slow_distance = max(float(slow_distance), stop_distance + 1e-6)
    if not np.isfinite(depth_m) or depth_m >= slow_distance:
        return 1.0
    if depth_m <= stop_distance:
        return 0.0
    return float(
        np.clip(
            (depth_m - stop_distance) / (slow_distance - stop_distance),
            0.0,
            1.0,
        )
    )
