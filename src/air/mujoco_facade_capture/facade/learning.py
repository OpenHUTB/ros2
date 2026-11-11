"""幕墙扫描速度控制的小型 NumPy 神经网络。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from .scene import ray_features


FEATURE_COUNT = 15
HIDDEN_UNITS = 32


def control_features(error: np.ndarray, ranges: np.ndarray, velocity: np.ndarray,
                     distance_target: float) -> np.ndarray:
    """将斜向测距、航点偏差及当前速度整理为有限输入。"""
    projected = ray_features(ranges)
    valid = projected[np.isfinite(projected)]
    wall_distance = float(np.median(valid)) if valid.size else distance_target
    corrected = np.asarray(error, dtype=float).copy()
    corrected[1] = np.clip(wall_distance - distance_target, -3., 3.)
    vector = np.concatenate((np.nan_to_num(ranges, nan=12., posinf=12.), corrected,
                             np.asarray(velocity, dtype=float)))
    if vector.shape != (FEATURE_COUNT,) or not np.isfinite(vector).all():
        raise ValueError("控制特征必须包含 15 个有限数值")
    return vector


class DenseController:
    """一层 ReLU 隐藏层，输入标准化，输出为三个归一化速度指令。"""

    def __init__(self, mean: np.ndarray, scale: np.ndarray, w1: np.ndarray,
                 b1: np.ndarray, w2: np.ndarray, b2: np.ndarray):
        self.mean = np.asarray(mean, dtype=np.float64)
        self.scale = np.asarray(scale, dtype=np.float64)
        self.w1, self.b1 = np.asarray(w1, dtype=np.float64), np.asarray(b1, dtype=np.float64)
        self.w2, self.b2 = np.asarray(w2, dtype=np.float64), np.asarray(b2, dtype=np.float64)
        if (self.mean.shape != (FEATURE_COUNT,) or self.scale.shape != (FEATURE_COUNT,)
                or self.w1.shape != (FEATURE_COUNT, HIDDEN_UNITS)
                or self.b1.shape != (HIDDEN_UNITS,) or self.w2.shape != (HIDDEN_UNITS, 3)
                or self.b2.shape != (3,) or not self._finite()):
            raise ValueError("模型权重结构不符合幕墙控制网络定义")
        if np.any(self.scale <= 0):
            raise ValueError("模型输入标准差必须为正数")

    def _finite(self) -> bool:
        return all(np.isfinite(array).all() for array in
                   (self.mean, self.scale, self.w1, self.b1, self.w2, self.b2))

    def predict(self, features: np.ndarray) -> np.ndarray:
        x = (np.asarray(features, dtype=np.float64) - self.mean) / self.scale
        hidden = np.maximum(x @ self.w1 + self.b1, 0.)
        return np.clip(hidden @ self.w2 + self.b2, -1., 1.)

    def save(self, path: Path) -> None:
        payload = {"version": 1, "input_features": FEATURE_COUNT,
                   "hidden_units": HIDDEN_UNITS, "mean": self.mean.tolist(),
                   "scale": self.scale.tolist(), "w1": self.w1.tolist(),
                   "b1": self.b1.tolist(), "w2": self.w2.tolist(), "b2": self.b2.tolist()}
        Path(path).write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")

    @classmethod
    def load(cls, path: Path) -> "DenseController":
        payload: dict[str, Any] = json.loads(Path(path).read_text(encoding="utf-8"))
        if payload.get("version") != 1 or payload.get("input_features") != FEATURE_COUNT:
            raise ValueError("不支持的控制模型版本或输入维数")
        return cls(*(payload[key] for key in ("mean", "scale", "w1", "b1", "w2", "b2")))


def train_controller(features: np.ndarray, targets: np.ndarray, epochs: int = 45,
                     seed: int = 27) -> DenseController:
    """使用固定随机种子和 Adam 最小化专家速度指令的均方误差。"""
    x = np.asarray(features, dtype=np.float64)
    y = np.asarray(targets, dtype=np.float64)
    if x.ndim != 2 or x.shape[1] != FEATURE_COUNT or y.shape != (len(x), 3):
        raise ValueError("训练数据形状必须是 N×15 输入和 N×3 标签")
    if len(x) < 64 or not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError("训练数据至少需要 64 个完整样本")
    if epochs < 1:
        raise ValueError("训练轮数必须大于零")
    rng = np.random.default_rng(seed)
    mean = x.mean(axis=0)
    scale = x.std(axis=0)
    scale[scale < 1e-6] = 1.
    x = (x - mean) / scale
    w1 = rng.normal(0., np.sqrt(2. / FEATURE_COUNT), (FEATURE_COUNT, HIDDEN_UNITS))
    b1 = np.zeros(HIDDEN_UNITS)
    w2 = rng.normal(0., np.sqrt(2. / HIDDEN_UNITS), (HIDDEN_UNITS, 3))
    b2 = np.zeros(3)
    params = [w1, b1, w2, b2]
    moments = [np.zeros_like(p) for p in params]
    variances = [np.zeros_like(p) for p in params]
    step = 0
    for _ in range(epochs):
        order = rng.permutation(len(x))
        for start in range(0, len(x), 128):
            indices = order[start:start + 128]
            xb, yb = x[indices], y[indices]
            z1 = xb @ w1 + b1
            h1 = np.maximum(z1, 0.)
            z2 = h1 @ w2 + b2
            prediction = np.clip(z2, -1., 1.)
            grad2 = (2. / len(indices)) * (prediction - yb)
            grad2 *= np.abs(z2) < 1.
            grad1 = (grad2 @ w2.T) * (z1 > 0.)
            gradients = [xb.T @ grad1, grad1.sum(axis=0), h1.T @ grad2, grad2.sum(axis=0)]
            step += 1
            for i, gradient in enumerate(gradients):
                moments[i] = .9 * moments[i] + .1 * gradient
                variances[i] = .999 * variances[i] + .001 * gradient * gradient
                mean_hat = moments[i] / (1. - .9 ** step)
                variance_hat = variances[i] / (1. - .999 ** step)
                params[i] -= .003 * mean_hat / (np.sqrt(variance_hat) + 1e-8)
    return DenseController(mean, scale, *params)
