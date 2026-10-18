"""
predictor.py —— 推理封装
========================

加载训练好的 MLP 权重，输入一帧前视灰度图，输出 (类别, 概率)。
"""

from __future__ import annotations

import numpy as np
from .nn import FallPredictorMLP

LABEL_NAMES = ["normal", "forward", "backward", "left", "right"]


class FallPredictor:
    """跌倒方向推理器。"""

    def __init__(self, weight_path: str, input_dim: int = 256,
                 hidden1: int = 128, hidden2: int = 64, output_dim: int = 5):
        self.model = FallPredictorMLP(input_dim, hidden1, hidden2, output_dim)
        self.model.load(weight_path)

    def predict_from_image(self, gray_img: np.ndarray) -> tuple[str, int, float]:
        """输入 (H,W) 灰度图（[0,1]），返回 (标签名, 类别, 置信度)。"""
        # 16x16 下采样
        h, w = gray_img.shape
        size = 16
        bh, bw = h // size, w // size
        cropped = gray_img[:bh * size, :bw * size]
        pooled = cropped.reshape(size, bh, size, bw).mean(axis=(1, 3))
        x = pooled.flatten().astype(np.float64)[None, :]
        cls, prob = self.model.predict(x)
        c = int(cls[0])
        return LABEL_NAMES[c], c, float(prob[0])
