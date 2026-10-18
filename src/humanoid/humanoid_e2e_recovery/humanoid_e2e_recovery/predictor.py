"""predictor.py —— 端到端推理：图像 → 15维关节目标角。"""
from __future__ import annotations
import numpy as np
from .e2e_nn import E2EMLP


class E2EPredictor:
    def __init__(self, weight_path, in_dim=256, h1=128, h2=64, out_dim=15):
        self.model = E2EMLP(in_dim, h1, h2, out_dim)
        self.model.load(weight_path)

    def predict_from_image(self, gray_img):
        h, w = gray_img.shape
        size = 16
        bh, bw = h//size, w//size
        c = gray_img[:bh*size, :bw*size]
        vec = c.reshape(size, bh, size, bw).mean(axis=(1,3)).flatten().astype(np.float64)[None, :]
        return self.model.predict(vec)[0]
