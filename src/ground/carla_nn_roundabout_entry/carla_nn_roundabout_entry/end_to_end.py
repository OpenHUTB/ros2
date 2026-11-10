"""相机图像 → 方向/油门/制动 的端到端 MLP 控制器（纯 NumPy，无框架依赖）。"""

from __future__ import annotations

import numpy as np


class ImageToControl:
    """16x16 灰度前视图像 -> (steering, throttle, brake) 的两层 MLP。

    输入图像形状为 (N, 16, 16) 或 (N, 256)，像素归一化到 [0,1]。
    输出形状 (N, 3)：steering ∈ [-1,1]，throttle/brake ∈ [0,1]。
    """

    def __init__(self, pixels: int = 16 * 16, hidden: int = 24, seed: int = 11) -> None:
        rng = np.random.default_rng(seed)
        self.w1 = rng.normal(0, 0.08, (pixels, hidden))
        self.b1 = np.zeros(hidden)
        self.w2 = rng.normal(0, 0.08, (hidden, 3))
        self.b2 = np.zeros(3)

    def _features(self, images: np.ndarray) -> np.ndarray:
        x = np.asarray(images, dtype=np.float64).reshape(len(images), -1) / 255.0
        return x

    def predict(self, images: np.ndarray) -> np.ndarray:
        x = self._features(images)
        hidden = np.tanh(x @ self.w1 + self.b1)
        raw = np.tanh(hidden @ self.w2 + self.b2)
        steering = raw[:, 0]
        throttle = (raw[:, 1] + 1.0) / 2.0
        brake = (raw[:, 2] + 1.0) / 2.0
        return np.column_stack((steering, throttle, brake))

    def train(self, images: np.ndarray, controls: np.ndarray,
              epochs: int = 200, learning_rate: float = 0.02) -> list[float]:
        """全连接回归训练，返回每轮损失列表（MSE）。"""
        x = self._features(images)
        y = np.asarray(controls, dtype=np.float64).copy()
        if x.shape[1] != self.w1.shape[0] or y.shape != (len(x), 3):
            raise ValueError("expected N grayscale images and an Nx3 control matrix")
        # 网络输出空间：throttle/brake 先映射回 [-1,1] 再做 tanh 回归
        y[:, 1:] = y[:, 1:] * 2.0 - 1.0
        losses: list[float] = []
        for _ in range(epochs):
            hidden = np.tanh(x @ self.w1 + self.b1)
            output = np.tanh(hidden @ self.w2 + self.b2)
            diff = output - y
            losses.append(float(np.mean(diff * diff)))
            grad = 2.0 * diff * (1.0 - output * output) / len(x)
            gw2, gb2 = hidden.T @ grad, grad.sum(axis=0)
            gh = (grad @ self.w2.T) * (1.0 - hidden * hidden)
            self.w2 -= learning_rate * gw2
            self.b2 -= learning_rate * gb2
            self.w1 -= learning_rate * (x.T @ gh)
            self.b1 -= learning_rate * gh.sum(axis=0)
        return losses

    # ---------- 极简持久化 ----------
    def save(self, path: str) -> None:
        np.savez(path, w1=self.w1, b1=self.b1, w2=self.w2, b2=self.b2)

    @classmethod
    def load(cls, path: str) -> "ImageToControl":
        data = np.load(path)
        model = cls(pixels=data["w1"].shape[0], hidden=data["w1"].shape[1])
        model.w1, model.b1 = data["w1"], data["b1"]
        model.w2, model.b2 = data["w2"], data["b2"]
        return model
