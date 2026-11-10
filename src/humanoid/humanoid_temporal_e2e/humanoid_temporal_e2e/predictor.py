"""predictor.py —— 推理封装（连续 3 帧灰度图 → 15 维关节目标角）

相对原实现：
- 池化逻辑复用 ``dataset.feat_from_gray``，训练 / demo / ROS 节点三处不再各写一份；
- 帧不满 3 帧时回退到训练集均值姿态（原来是返回全 0，会让机器人在前两帧
  收到“全零关节目标”，直接把腿拉到极限）；
- 标准化参数由模型自带（``load`` 已恢复 x_mean/x_std），推理端无需再算一遍。
"""
from __future__ import annotations

import numpy as np
from .dataset import feat_from_gray, N_FRAMES
from .nn import TemporalMLP


class TemporalPredictor:
    def __init__(self, weights_path):
        self.model = TemporalMLP().load(weights_path)
        self._buffer = []
        self.fallback = self.model.y_mean.copy()

    def reset(self):
        self._buffer = []

    def push_frame(self, gray_img):
        feat = feat_from_gray(gray_img)
        self._buffer.append(feat)
        if len(self._buffer) > N_FRAMES:
            self._buffer = self._buffer[-N_FRAMES:]
        return self._buffer[-1]

    @property
    def ready(self):
        return len(self._buffer) >= N_FRAMES

    def predict(self):
        if not self.ready:
            return self.fallback.copy()
        x = np.concatenate(self._buffer[-N_FRAMES:]).reshape(1, -1)
        return self.model.predict(x)[0]
