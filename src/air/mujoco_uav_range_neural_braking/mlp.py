"""轻量 MLP 回归器。 Small NumPy-only MLP regressor."""
from pathlib import Path
import json

import numpy as np


class MLPRegressor:
    """两层 ReLU 网络，使用 JSON 保存权重。 Two-layer ReLU network with JSON weights."""

    def __init__(self, input_size=2, hidden_size=16, seed=2026):
        rng = np.random.default_rng(seed)
        self.w1 = rng.normal(0, np.sqrt(2 / input_size), (input_size, hidden_size))
        self.b1 = np.zeros(hidden_size)
        self.w2 = rng.normal(0, np.sqrt(2 / hidden_size), hidden_size)
        self.b2 = 0.0
        self.mean = np.zeros(input_size)
        self.scale = np.ones(input_size)
        self.target_mean = 0.0
        self.target_scale = 1.0

    def _forward(self, features):
        hidden = np.maximum(features @ self.w1 + self.b1, 0)
        return hidden, hidden @ self.w2 + self.b2

    def fit(self, features, target, epochs=350, learning_rate=0.003):
        """在有限二维样本上拟合。 Fit only finite, shape-consistent samples."""
        features = np.asarray(features, dtype=float)
        target = np.asarray(target, dtype=float).reshape(-1)
        if features.ndim != 2 or features.shape[1] != self.w1.shape[0]:
            raise ValueError("输入特征形状不匹配 / Feature matrix shape does not match the model")
        if len(features) == 0 or len(features) != len(target):
            raise ValueError("样本与标签须非空且数量相同 / Samples and targets must be non-empty and aligned")
        if not np.isfinite(features).all() or not np.isfinite(target).all():
            raise ValueError("训练样本须为有限数值 / Training data must contain only finite values")
        if epochs <= 0 or learning_rate <= 0 or not np.isfinite(learning_rate):
            raise ValueError("训练轮次和学习率须为正数 / Epochs and learning rate must be positive")
        self.mean = features.mean(axis=0)
        self.scale = np.maximum(features.std(axis=0), 1e-6)
        self.target_mean = float(target.mean())
        self.target_scale = float(max(target.std(), 1e-6))
        x = (features - self.mean) / self.scale
        y = (target - self.target_mean) / self.target_scale
        for _ in range(epochs):
            hidden, prediction = self._forward(x)
            error = prediction - y
            grad_prediction = 2 * error / len(x)
            grad_w2 = hidden.T @ grad_prediction
            grad_b2 = grad_prediction.sum()
            grad_hidden = grad_prediction[:, None] * self.w2
            grad_hidden[hidden <= 0] = 0
            grad_w1 = x.T @ grad_hidden
            grad_b1 = grad_hidden.sum(axis=0)
            self.w1 -= learning_rate * grad_w1
            self.b1 -= learning_rate * grad_b1
            self.w2 -= learning_rate * grad_w2
            self.b2 -= learning_rate * grad_b2
        return self

    def predict(self, features):
        values = np.asarray(features, dtype=float)
        single = values.ndim == 1
        values = np.atleast_2d(values)
        if values.ndim != 2 or values.shape[1] != self.w1.shape[0]:
            raise ValueError("输入特征形状不匹配 / Feature matrix shape does not match the model")
        if not np.isfinite(values).all():
            raise ValueError("输入特征须为有限数值 / Features must contain only finite values")
        normalized = (values - self.mean) / self.scale
        _, prediction = self._forward(normalized)
        result = prediction * self.target_scale + self.target_mean
        return float(result[0]) if single else result

    def save(self, path):
        payload = {"input_features": ["sensor_range_m", "closing_speed_m_s"],
                   "hidden_size": len(self.b1), "w1": self.w1.tolist(), "b1": self.b1.tolist(),
                   "w2": self.w2.tolist(), "b2": float(self.b2), "mean": self.mean.tolist(),
                   "scale": self.scale.tolist(), "target_mean": self.target_mean,
                   "target_scale": self.target_scale}
        Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path):
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        model = cls(input_size=len(payload["mean"]), hidden_size=payload["hidden_size"])
        for name in ("w1", "b1", "w2", "mean", "scale"):
            setattr(model, name, np.asarray(payload[name], dtype=float))
        model.b2 = float(payload["b2"])
        model.target_mean = float(payload["target_mean"])
        model.target_scale = float(payload["target_scale"])
        return model
