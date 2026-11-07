"""将灰度前视图像映射为方向、油门和制动指令的 MLP。"""

from __future__ import annotations

import numpy as np


class ImageToControl:
    def __init__(self, pixels: int = 16 * 16, hidden: int = 24, seed: int = 11) -> None:
        rng = np.random.default_rng(seed)
        self.w1 = rng.normal(0, 0.08, (pixels, hidden))
        self.b1 = np.zeros(hidden)
        self.w2 = rng.normal(0, 0.08, (hidden, 3))
        self.b2 = np.zeros(3)

    def predict(self, images: np.ndarray) -> np.ndarray:
        x = np.asarray(images, dtype=np.float64).reshape(len(images), -1) / 255.0
        hidden = np.tanh(x @ self.w1 + self.b1)
        raw = np.tanh(hidden @ self.w2 + self.b2)
        steering = raw[:, 0]
        throttle = (raw[:, 1] + 1.0) / 2.0
        brake = (raw[:, 2] + 1.0) / 2.0
        return np.column_stack((steering, throttle, brake))

    def train(self, images: np.ndarray, controls: np.ndarray, epochs: int = 200, learning_rate: float = 0.02) -> None:
        x = np.asarray(images, dtype=np.float64).reshape(len(images), -1) / 255.0
        y = np.asarray(controls, dtype=np.float64).copy()
        if x.shape[1] != self.w1.shape[0] or y.shape != (len(x), 3):
            raise ValueError("expected N grayscale images and an Nx3 control matrix")
        y[:, 1:] = y[:, 1:] * 2.0 - 1.0
        for _ in range(epochs):
            hidden = np.tanh(x @ self.w1 + self.b1)
            output = np.tanh(hidden @ self.w2 + self.b2)
            grad = 2.0 * (output - y) * (1.0 - output * output) / len(x)
            gw2, gb2 = hidden.T @ grad, grad.sum(axis=0)
            gh = (grad @ self.w2.T) * (1.0 - hidden * hidden)
            self.w2 -= learning_rate * gw2
            self.b2 -= learning_rate * gb2
            self.w1 -= learning_rate * (x.T @ gh)
            self.b1 -= learning_rate * gh.sum(axis=0)
