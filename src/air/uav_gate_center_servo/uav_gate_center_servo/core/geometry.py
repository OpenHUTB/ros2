#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""坐标与投影：四元数、相机偏移、门中心标签。"""
from __future__ import annotations

from typing import Tuple

import numpy as np

#: front_cam 在机体系的安装位置（quadrotor.xml）
CAM_OFFSET_BODY = np.array([0.075, 0.0, 0.012], dtype=np.float64)
FOVY_DEG = 70.0

__all__ = ["CAM_OFFSET_BODY", "FOVY_DEG", "quat_to_mat", "yaw_of",
           "camera_label", "body_target", "project_label", "heading_to_body"]


def quat_to_mat(q) -> np.ndarray:
    """四元数 (w,x,y,z) → 机体到世界的 3×3 旋转矩阵。"""
    w, x, y, z = (float(v) for v in np.asarray(q, dtype=float).reshape(4))
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def yaw_of(q) -> float:
    """四元数 → 偏航角（rad）。"""
    w, x, y, z = (float(v) for v in np.asarray(q, dtype=float).reshape(4))
    return float(np.arctan2(2.0 * (w * z + x * y),
                            1.0 - 2.0 * (y * y + z * z)))


def camera_label(gate_center: np.ndarray, pos: np.ndarray, quat) -> np.ndarray:
    """门中心相对相机的标签 [forward, left, up]，米。

    固定航向 0 时与世界 ENU 同向；相机安装偏移在这里扣除。
    """
    R = quat_to_mat(quat)
    p_cam = np.asarray(pos, float) + R @ CAM_OFFSET_BODY
    return (np.asarray(gate_center, float) - p_cam).astype(np.float32)


def body_target(label_cam: np.ndarray, quat) -> np.ndarray:
    """相机系标签 → 机体质心目标：label + R·c_b。"""
    R = quat_to_mat(quat)
    return (np.asarray(label_cam, float) + R @ CAM_OFFSET_BODY).astype(np.float32)


def project_label(label: np.ndarray, hw: Tuple[int, int] = (48, 64),
                  fovy_deg: float = FOVY_DEG) -> Tuple[float, float]:
    """[forward,left,up] → 像素 (u,v)；图像右 = -y，图像上 = +z。"""
    forward, left, up = (float(v) for v in np.asarray(label, float).reshape(3))
    h, w = int(hw[0]), int(hw[1])
    if forward <= 1e-6:
        return float("nan"), float("nan")
    fx = fy = (h / 2.0) / np.tan(np.deg2rad(fovy_deg) / 2.0)
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    return float(cx - fx * left / forward), float(cy - fy * up / forward)


def heading_to_body(v_heading: np.ndarray, quat) -> np.ndarray:
    """航向系（世界 ENU）速度 → 机体系；底座会再左乘 R 复原。"""
    R = quat_to_mat(quat)
    return (R.T @ np.asarray(v_heading, float)).astype(np.float32)
