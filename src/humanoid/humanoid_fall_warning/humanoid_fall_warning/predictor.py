"""predictor.py —— 多模态跌倒预警推理"""
from __future__ import annotations
import numpy as np
from .nn import FallWarningMLP


class FallWarningPredictor:
    def __init__(self, weights_path):
        self.model = FallWarningMLP()
        self.model.load(weights_path)

    def predict(self, img_feat, imu_pitch, imu_roll):
        """img_feat: 256维图像特征, imu_pitch/roll: 标量 → (正常概率, 跌倒概率)"""
        x = np.concatenate([img_feat, [imu_pitch, imu_roll]]).reshape(1, -1)
        probs = self.model.predict(x)[0]
        return float(probs[0]), float(probs[1])
