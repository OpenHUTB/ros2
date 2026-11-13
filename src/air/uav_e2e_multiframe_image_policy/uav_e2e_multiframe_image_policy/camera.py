#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""下视相机：渲染局部俯视灰度图，维护连续 T 帧历史缓存。"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

IMAGE_W = 16
IMAGE_H = 12
PATCH_W = 2.4
PATCH_H = 1.8
FRAME_STACK = 3


def render_patch(x: float, y: float, yaw: float) -> np.ndarray:
    """以机体为中心渲染局部俯视灰度图，返回 H×W、取值 [0,1]。"""
    image = np.full((IMAGE_H, IMAGE_W), 0.15, dtype=float)
    cw = PATCH_W / IMAGE_W
    ch = PATCH_H / IMAGE_H
    for r in range(IMAGE_H):
        for c in range(IMAGE_W):
            lx = (c - IMAGE_W / 2.0) * cw
            ly = (IMAGE_H / 2.0 - r) * ch
            wx = x + lx
            wy = y + ly
            from . import scene
            dy = abs(wy - scene.centerline_y(wx))
            if dy < 0.28:
                image[r, c] = 0.95
            elif dy < 0.60:
                image[r, c] = 0.45
    return image


class FrameHistory:
    """保留最近 FRAME_STACK 帧，不足时重复首帧填充。"""

    def __init__(self, frame_stack: int = FRAME_STACK) -> None:
        self.frame_stack = frame_stack
        self.frames: deque[np.ndarray] = deque(maxlen=frame_stack)

    def update(self, frame: np.ndarray) -> None:
        self.frames.append(frame.copy())

    def reset(self, frame: np.ndarray) -> None:
        self.frames.clear()
        self.frames.append(frame.copy())

    def stack(self) -> np.ndarray:
        if not self.frames:
            raise RuntimeError("帧缓存为空，请先 update 一帧")
        padded = list(self.frames)
        while len(padded) < self.frame_stack:
            padded.insert(0, padded[0])
        return np.stack(padded, axis=0)
