"""predictor.py —— 推理封装（3帧→15维关节角）"""
from __future__ import annotations
import numpy as np
from .nn import TemporalMLP


class TemporalPredictor:
    def __init__(self, weights_path):
        self.model = TemporalMLP()
        self.model.load(weights_path)
        self._buffer = []

    def push_frame(self, gray_img):
        h, w = gray_img.shape
        size = 16
        bh, bw = h//size, w//size
        crop = gray_img[:bh*size, :bw*size]
        feat = crop.reshape(size, bh, size, bw).mean(axis=(1,3)).flatten().astype(np.float64)
        self._buffer.append(feat)
        if len(self._buffer) > 3:
            self._buffer = self._buffer[-3:]

    def predict(self):
        if len(self._buffer) < 3:
            return np.zeros(15)
        x = np.concatenate(self._buffer[-3:]).reshape(1, -1)
        return self.model.predict(x)[0]
