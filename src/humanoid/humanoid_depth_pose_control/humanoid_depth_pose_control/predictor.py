"""predictor.py —— 推理封装"""
from __future__ import annotations
import numpy as np
from .nn import DepthPoseMLP


class DepthPosePredictor:
    def __init__(self, weights_path):
        self.model = DepthPoseMLP()
        self.model.load(weights_path)

    def predict_from_depth(self, depth_img):
        """depth_img: (64,64) float [0,1] → (pitch, roll, yaw) 弧度"""
        h, w = depth_img.shape
        size = 16
        bh, bw = h//size, w//size
        crop = depth_img[:bh*size, :bw*size]
        feat = crop.reshape(size, bh, size, bw).mean(axis=(1,3)).flatten().astype(np.float64)
        pred = self.model.predict(feat.reshape(1, -1))[0]
        return float(pred[0]), float(pred[1]), float(pred[2])
