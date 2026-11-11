"""predictor.py —— 端到端推理：图像 → 15 维力矩增量（N·m）。

训练时输入做过逐维标准化、输出做过中心化+缩放，推理必须应用**同一套**
参数，否则网络看到的分布与训练时不同（这正是上一版权重被判定为"死网络"
的原因之一：权重文件与数据集版本错配，且输入从未零中心化）。
"""
from __future__ import annotations
import numpy as np
from .e2e_nn import E2EMLP

IMG_SIZE = 16


class E2EPredictor:
    def __init__(self, weight_path, in_dim=256, h1=128, h2=64, out_dim=15):
        self.model = E2EMLP(in_dim, h1, h2, out_dim)
        self.model.load(weight_path)
        d = np.load(weight_path)
        # X 标准化参数：缺失时退化为恒等变换（兼容早期权重），并给出告警
        self.x_mean = (d["X_mean"].astype(np.float64) if "X_mean" in d
                       else np.zeros(in_dim))
        self.x_std = (d["X_std"].astype(np.float64) if "X_std" in d
                      else np.ones(in_dim))
        # Y 中心化 + 缩放参数
        self.y_mean = (d["Y_mean"].astype(np.float64) if "Y_mean" in d
                       else np.zeros(out_dim))
        self.y_scale = (d["Y_scale"].astype(np.float64) if "Y_scale" in d
                        else np.ones(out_dim))
        # 恒定维度若被存成极小值（旧版缺陷），这里兜底为 1.0，防止输出爆炸
        self.y_scale = np.where(np.abs(self.y_scale) > 1e-6,
                                self.y_scale, 1.0)

    def _img_to_vec(self, gray_img):
        h, w = gray_img.shape
        bh, bw = h // IMG_SIZE, w // IMG_SIZE
        c = gray_img[:bh * IMG_SIZE, :bw * IMG_SIZE]
        return c.reshape(IMG_SIZE, bh, IMG_SIZE, bw).mean(axis=(1, 3)) \
                .flatten().astype(np.float64)

    def predict_from_image(self, gray_img):
        return self.predict_vecs(self._img_to_vec(gray_img)[None, :])[0]

    def predict_vecs(self, vecs):
        """批量前向（原始像素向量入，物理量纲力矩出）。"""
        vecs = np.atleast_2d(np.asarray(vecs, dtype=np.float64))
        xn = (vecs - self.x_mean) / self.x_std
        return self.model.predict(xn) * self.y_scale + self.y_mean

    def hidden_stats(self, vecs):
        """第一/二层 ReLU 健康度——用于判别"死网络"（输出与输入解耦）。"""
        vecs = np.atleast_2d(np.asarray(vecs, dtype=np.float64))
        xn = (vecs - self.x_mean) / self.x_std
        m = self.model
        z1 = xn @ m.W1 + m.b1
        a1 = np.maximum(0, z1)
        z2 = a1 @ m.W2 + m.b2
        a2 = np.maximum(0, z2)
        return {
            "relu1_activation_rate": float((a1 > 0).mean()),
            "relu1_alive_neurons": int((a1 > 0).any(axis=0).sum()),
            "relu1_total_neurons": int(a1.shape[1]),
            "relu2_activation_rate": float((a2 > 0).mean()),
            "relu2_alive_neurons": int((a2 > 0).any(axis=0).sum()),
            "relu2_total_neurons": int(a2.shape[1]),
        }
