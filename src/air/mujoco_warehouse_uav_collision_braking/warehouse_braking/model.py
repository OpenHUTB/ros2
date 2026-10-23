"""轻量多层感知机。 / Lightweight multilayer perceptron."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


class RiskSpeedMLP:
    """共享隐藏层，同时预测风险类别和参考速度。 / Joint risk and speed predictor."""

    def __init__(self, input_size=20, hidden=(24, 16), seed=42):
        self.sizes = (input_size, *hidden, 4)
        rng = np.random.default_rng(seed)
        self.weights = [rng.normal(0, np.sqrt(2 / (a + b)), (a, b))
                        for a, b in zip(self.sizes[:-1], self.sizes[1:])]
        self.biases = [np.zeros(b) for b in self.sizes[1:]]
        self.mean = np.zeros(input_size, dtype=np.float32)
        self.scale = np.ones(input_size, dtype=np.float32)

    def _forward(self, x):
        activations = [x]
        for weight, bias in zip(self.weights[:-1], self.biases[:-1]):
            activations.append(np.tanh(activations[-1] @ weight + bias))
        logits_speed = activations[-1] @ self.weights[-1] + self.biases[-1]
        return activations, logits_speed

    def predict(self, x):
        normalized = (np.asarray(x, dtype=np.float32) - self.mean) / self.scale
        _, output = self._forward(normalized)
        logits = output[:, :3]
        logits -= logits.max(axis=1, keepdims=True)
        probability = np.exp(logits)
        probability /= probability.sum(axis=1, keepdims=True)
        speed = 3.0 / (1.0 + np.exp(-output[:, 3]))
        return probability, speed

    def fit(self, x, target_speed, risk, epochs=80, learning_rate=0.01):
        x = np.asarray(x, dtype=np.float32)
        target_speed = np.asarray(target_speed, dtype=np.float32)
        risk = np.asarray(risk, dtype=np.int64)
        self.mean, self.scale = x.mean(0), x.std(0)
        self.scale = np.where(self.scale < 1e-6, 1.0, self.scale)
        normalized = (x - self.mean) / self.scale
        history = []
        for _ in range(epochs):
            activations, output = self._forward(normalized)
            logits = output[:, :3]
            logits -= logits.max(axis=1, keepdims=True)
            probability = np.exp(logits)
            probability /= probability.sum(axis=1, keepdims=True)
            speed = 3.0 / (1.0 + np.exp(-output[:, 3]))
            error = probability.copy()
            error[np.arange(len(risk)), risk] -= 1.0
            error /= len(x)
            speed_error = (speed - target_speed) * (speed * (3.0 - speed) / 3.0)
            gradient = np.column_stack([error, 0.1 * speed_error])
            gradients_w = []
            gradients_b = []
            for index in range(len(self.weights) - 1, -1, -1):
                gradients_w.append(activations[index].T @ gradient)
                gradients_b.append(gradient.sum(0))
                if index:
                    gradient = (gradient @ self.weights[index].T) * (1 - activations[index] ** 2)
            for index, (grad_w, grad_b) in enumerate(zip(gradients_w[::-1], gradients_b[::-1])):
                self.weights[index] -= learning_rate * grad_w
                self.biases[index] -= learning_rate * grad_b
            history.append(float(-np.log(probability[np.arange(len(risk)), risk] + 1e-8).mean()))
        return history

    def save(self, path: Path):
        payload = {"mean": self.mean, "scale": self.scale}
        payload.update({f"w{i}": value for i, value in enumerate(self.weights)})
        payload.update({f"b{i}": value for i, value in enumerate(self.biases)})
        np.savez(path, **payload)

    @classmethod
    def load(cls, path: Path):
        with np.load(path) as data:
            model = cls(input_size=len(data["mean"]), hidden=(len(data["w0"][0]), len(data["w1"][0])))
            model.mean, model.scale = data["mean"], data["scale"]
            model.weights = [data[f"w{i}"] for i in range(3)]
            model.biases = [data[f"b{i}"] for i in range(3)]
        return model


def save_metadata(path: Path, metadata: dict):
    """保存可读训练元数据。 / Save human-readable training metadata."""
    Path(path).write_text(json.dumps(metadata, indent=2), encoding="utf-8")
