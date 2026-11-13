#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端多帧图像策略网络。

输入连续 T 帧下视灰度图堆叠，固定卷积/池化线索提取后经两层 MLP 直接输出
前飞速度与偏航指令；纯 numpy 前向，不依赖深度学习框架。
"""

from __future__ import annotations

import numpy as np

HIDDEN_DIM = 10


def _conv3x3(image: np.ndarray, kernel: np.ndarray) -> np.ndarray:
    h, w = image.shape
    out = np.zeros((h - 2, w - 2), dtype=float)
    for r in range(1, h - 1):
        for c in range(1, w - 1):
            region = image[r - 1:r + 2, c - 1:c + 2]
            out[r - 1, c - 1] = float(np.sum(region * kernel))
    return out


class E2EImagePolicy:
    """多帧图像端到端策略。"""

    def __init__(self) -> None:
        self.kernels = self._fixed_kernels()
        # 隐藏层近似恒等映射，保留两层结构；输出层按跟随走廊的反馈结构预置
        self.w_hidden = np.zeros((HIDDEN_DIM, 4))
        for i in range(4):
            self.w_hidden[i, i] = 1.0
        self.b_hidden = np.zeros(HIDDEN_DIM)
        self.w_out = np.zeros((2, HIDDEN_DIM))
        self.w_out[0, 1] = 0.15          # 中心可见度调节前飞
        self.w_out[1, 0] = -0.26         # 行质心偏差 -> 反向偏航修正
        self.w_out[1, 2] = 0.15          # 横向梯度 -> 航向微调
        self.b_out = np.array([0.55, 0.0])

    @staticmethod
    def _fixed_kernels() -> list[np.ndarray]:
        center_row = np.array([[0, 0, 0], [1, 1, 1], [0, 0, 0]], dtype=float)
        sobel_x = np.array([[-1, 0, 1], [-1, 0, 1], [-1, 0, 1]], dtype=float)
        sobel_y = np.array([[-1, -1, -1], [0, 0, 0], [1, 1, 1]], dtype=float)
        blur = np.ones((3, 3), dtype=float) / 9.0
        return [center_row, sobel_x, sobel_y, blur]

    def _features(self, frames: np.ndarray) -> np.ndarray:
        # 三帧取平均，叠加时间平滑
        image = frames.mean(axis=0)
        mask = image > 0.5
        ys, xs = np.nonzero(mask)
        row_centroid = (float(ys.mean()) - (image.shape[0] - 1) / 2.0) if len(ys) else 0.0
        center_band = float(image[4:8, 6:10].mean())
        sobel = [_conv3x3(image, k) for k in self.kernels]
        grad_x = float(sobel[1].mean())
        return np.array([row_centroid, center_band, grad_x,
                         float(sobel[3].mean())])

    def forward(self, frames: np.ndarray) -> dict:
        f = self._features(frames)
        hidden = np.tanh(self.w_hidden @ f + self.b_hidden)
        raw = self.w_out @ hidden + self.b_out
        forward = float(np.clip(raw[0], 0.0, 1.0))
        yaw_cmd = float(np.clip(raw[1], -1.2, 1.2))
        return {"forward": forward, "yaw_cmd": yaw_cmd, "features": f}
