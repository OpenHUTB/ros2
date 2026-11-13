#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""下视相机：把地面标线投影到机载图像，并提取横向偏移与航向误差。

相机坐标系：x 向前、y 向左；图像列对应横向，行对应前向。
本模块只做几何投影与图像域特征提取，不依赖 ROS，可在离线仿真中直接运行。
"""

from __future__ import annotations

import math

import numpy as np

from . import track

IMAGE_W = 32
IMAGE_H = 24
FWD_MIN = -0.15
FWD_MAX = 0.85
LAT_MIN = -1.30
LAT_MAX = 1.30


class DownviewCamera:
    """简化下视相机，输出标线特征与二值图像网格。"""

    def __init__(self, image_w: int = IMAGE_W, image_h: int = IMAGE_H):
        self.w = image_w
        self.h = image_h

    def observe(self, state) -> dict:
        """根据机体位姿提取标线特征。

        参数:
            state: 含 ``x, y, yaw`` 的状态对象或字典。

        返回:
            字典：``image`` 二值网格、``offset`` 横向偏移 m、
            ``heading`` 航向误差 rad、``lost`` 是否丢失标线。
        """
        x = float(state.x if hasattr(state, "x") else state["x"])
        y = float(state.y if hasattr(state, "y") else state["y"])
        yaw = float(state.yaw if hasattr(state, "yaw") else state["yaw"])

        image = np.zeros((self.h, self.w), dtype=np.int8)
        samples: list[tuple[float, float]] = []

        xs = np.linspace(max(track.X_MIN, x - 0.5),
                         min(track.X_MAX, x + 1.0), 24)
        cy, sy = math.cos(yaw), math.sin(yaw)
        for rx in xs:
            ry = track.reference_y(rx)
            dx, dy = rx - x, ry - y
            fwd = cy * dx + sy * dy
            left = -sy * dx + cy * dy
            if FWD_MIN <= fwd <= FWD_MAX and LAT_MIN <= left <= LAT_MAX:
                samples.append((left, fwd))
                col = int(round((left - LAT_MIN) / (LAT_MAX - LAT_MIN) * (self.w - 1)))
                row = int(round((FWD_MAX - fwd) / (FWD_MAX - FWD_MIN) * (self.h - 1)))
                col = min(max(col, 0), self.w - 1)
                row = min(max(row, 0), self.h - 1)
                image[row, col] = 1

        lost = len(samples) < 2
        if lost:
            return {"image": image, "offset": 0.0, "heading": 0.0, "lost": True,
                    "samples": samples}

        lefts = np.array([s[0] for s in samples])
        fwds = np.array([s[1] for s in samples])
        offset = float(lefts.mean())

        # 用样本点拟合 left = a·fwd + b，斜率 a 给出标线在机体系的朝向
        if fwds.max() - fwds.min() > 1e-6:
            a, _b = np.polyfit(fwds, lefts, 1)
            heading = float(math.atan(float(a)))
        else:
            heading = 0.0

        return {"image": image, "offset": offset, "heading": heading,
                "lost": False, "samples": samples}

    @staticmethod
    def normalize(offset: float, heading: float, speed: float) -> np.ndarray:
        """把感知特征归一化到神经网络输入区间。"""
        return np.array([
            float(np.clip(offset / 0.5, -1.0, 1.0)),
            float(np.clip(heading / 0.6, -1.0, 1.0)),
            float(np.clip(speed / 1.2, -1.0, 1.0)),
        ], dtype=float)
