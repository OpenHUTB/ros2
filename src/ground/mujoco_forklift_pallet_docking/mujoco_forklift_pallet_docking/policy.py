"""Small NumPy MLP that maps camera pixels to throttle and brake."""

from __future__ import annotations

import numpy as np


class ImagePolicy:
    def __init__(self, seed: int = 17, hidden: int = 32) -> None:
        rng = np.random.default_rng(seed)
        self.w1 = rng.normal(0, 0.04, (16 * 16, hidden))
        self.b1 = np.zeros(hidden)
        self.w2 = rng.normal(0, 0.035, (hidden, 2))
        self.b2 = np.zeros(2)

    def forward(self, images: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        pixels = np.asarray(images, dtype=np.float64).reshape(-1, 16, 4, 16, 4)
        x = pixels.mean(axis=(2, 4)).reshape(-1, 16 * 16) / 64.0
        h = np.tanh(x @ self.w1 + self.b1)
        y = 1.0 / (1.0 + np.exp(-(h @ self.w2 + self.b2)))
        return h, y

    def predict(self, images: np.ndarray) -> np.ndarray:
        return self.forward(images)[1]

    def train(self, images: np.ndarray, actions: np.ndarray, epochs: int = 240, lr: float = 0.08) -> float:
        pixels = np.asarray(images, dtype=np.float64).reshape(-1, 16, 4, 16, 4)
        x = pixels.mean(axis=(2, 4)).reshape(-1, 16 * 16) / 64.0
        target = np.asarray(actions, dtype=np.float64).reshape(-1, 2)
        for _ in range(epochs):
            h, y = self.forward(images)
            weights = np.ones_like(target)
            weights[:, 1] += 5.0 * target[:, 1]
            dy = (y - target) * y * (1.0 - y) * weights / weights.sum(axis=0)
            dw2 = h.T @ dy
            db2 = dy.sum(axis=0)
            dh = (dy @ self.w2.T) * (1.0 - h * h)
            dw1 = x.T @ dh
            db1 = dh.sum(axis=0)
            self.w1 -= lr * dw1
            self.b1 -= lr * db1
            self.w2 -= lr * dw2
            self.b2 -= lr * db2
        return float(np.mean((self.predict(images) - target) ** 2))

    def save(self, path: str) -> None:
        np.savez(path, w1=self.w1, b1=self.b1, w2=self.w2, b2=self.b2)

    @classmethod
    def load(cls, path: str) -> "ImagePolicy":
        model = cls()
        with np.load(path) as data:
            model.w1, model.b1 = data["w1"], data["b1"]
            model.w2, model.b2 = data["w2"], data["b2"]
        return model


def teacher_action(distance: float, speed: float) -> np.ndarray:
    """Supervisory braking target from stopping distance and current speed."""
    required = speed * speed / 8.0 - 0.40
    brake = float(distance <= required + 1e-12 * max(1.0, abs(required)))
    throttle = float(brake == 0.0 and speed < 2.8)
    return np.array([throttle, brake], dtype=np.float64)
