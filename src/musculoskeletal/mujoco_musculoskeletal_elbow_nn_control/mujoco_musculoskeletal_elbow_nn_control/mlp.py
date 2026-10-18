"""Small dependency-light MLP for bounded elbow activation inference."""

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Tuple

import numpy as np

FEATURES = ("target_angle_rad", "angle_rad", "angular_velocity_rad_s", "flexor_force_n", "extensor_force_n")
TARGETS = ("flexor_activation", "extensor_activation")


@dataclass
class Metrics:
    mae: float
    rmse: float
    steady_state_error: float
    inference_ms: float


class ActivationMLP:
    """Two-layer tanh network with JSON serialization and bounded outputs."""

    def __init__(self, input_size=5, hidden_size=16, seed=25012):
        rng = np.random.default_rng(seed)
        self.w1 = rng.normal(0.0, np.sqrt(2.0 / input_size), (input_size, hidden_size))
        self.b1 = np.zeros(hidden_size)
        self.w2 = rng.normal(0.0, np.sqrt(2.0 / hidden_size), (hidden_size, 2))
        self.b2 = np.zeros(2)
        self.mean = np.zeros(input_size)
        self.scale = np.ones(input_size)

    def _forward(self, x):
        hidden = np.tanh(x @ self.w1 + self.b1)
        return 1.0 / (1.0 + np.exp(-np.clip(hidden @ self.w2 + self.b2, -30.0, 30.0)))

    def predict(self, features: Iterable[Iterable[float]]) -> np.ndarray:
        x = np.asarray(features, dtype=float)
        if x.ndim == 1:
            x = x[None, :]
        if x.shape[1] != len(FEATURES) or not np.isfinite(x).all():
            raise ValueError("features must be finite arrays with five columns")
        return np.clip(self._forward((x - self.mean) / self.scale), 0.0, 1.0)

    def fit(self, x, y, epochs=600, learning_rate=0.01, patience=60, validation=None):
        x = np.asarray(x, dtype=float); y = np.asarray(y, dtype=float)
        if x.ndim != 2 or x.shape[1] != 5 or y.shape != (len(x), 2):
            raise ValueError("training arrays have invalid shapes")
        self.mean = x.mean(axis=0); self.scale = x.std(axis=0); self.scale[self.scale < 1e-8] = 1.0
        xn = (x - self.mean) / self.scale
        vx, vy = validation if validation is not None else (xn, y)
        if validation is not None: vx = (np.asarray(vx) - self.mean) / self.scale
        best = None; best_loss = float("inf"); stale = 0
        for _ in range(epochs):
            h = np.tanh(xn @ self.w1 + self.b1); p = 1.0 / (1.0 + np.exp(-np.clip(h @ self.w2 + self.b2, -30, 30)))
            dz2 = (p - y) * p * (1.0 - p) * (2.0 / len(x))
            self.w2 -= learning_rate * h.T @ dz2; self.b2 -= learning_rate * dz2.sum(axis=0)
            dz1 = (dz2 @ self.w2.T) * (1.0 - h * h)
            self.w1 -= learning_rate * xn.T @ dz1; self.b1 -= learning_rate * dz1.sum(axis=0)
            loss = float(np.mean((self._forward(vx) - vy) ** 2))
            if loss < best_loss - 1e-10:
                best_loss = loss; best = self.to_dict(); stale = 0
            else: stale += 1
            if stale >= patience: break
        if best is not None: self.from_dict(best)
        return best_loss

    def to_dict(self):
        return {"features": list(FEATURES), "targets": list(TARGETS), "w1": self.w1.tolist(), "b1": self.b1.tolist(), "w2": self.w2.tolist(), "b2": self.b2.tolist(), "mean": self.mean.tolist(), "scale": self.scale.tolist()}

    def from_dict(self, data):
        self.w1 = np.asarray(data["w1"], dtype=float); self.b1 = np.asarray(data["b1"], dtype=float)
        self.w2 = np.asarray(data["w2"], dtype=float); self.b2 = np.asarray(data["b2"], dtype=float)
        self.mean = np.asarray(data["mean"], dtype=float); self.scale = np.asarray(data["scale"], dtype=float)
        return self

    def save(self, path: Path): Path(path).write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")

    @classmethod
    def load(cls, path: Path): return cls().from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def score(model, x, y, steady_tail=0.1) -> Metrics:
    start = time.perf_counter(); prediction = model.predict(x); elapsed = (time.perf_counter() - start) * 1000.0 / max(1, len(prediction))
    error = prediction - np.asarray(y); tail = max(1, int(len(error) * steady_tail))
    return Metrics(float(np.mean(np.abs(error))), float(np.sqrt(np.mean(error ** 2))), float(np.mean(np.abs(error[-tail:]))), float(elapsed))
