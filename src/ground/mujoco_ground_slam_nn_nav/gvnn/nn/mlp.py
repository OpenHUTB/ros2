"""Multi-layer perceptron with Adam training helpers."""

from __future__ import annotations

import json
import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .layers import Dense
from .losses import mse, softmax_cross_entropy
from .optim import Adam


class MLP:
    def __init__(self, sizes: Sequence[int], activation: str = "relu",
                 out_activation: str = "linear", seed: int = 0,
                 mean: Optional[np.ndarray] = None, std: Optional[np.ndarray] = None):
        if len(sizes) < 2:
            raise ValueError("sizes must contain input and output dimensions")
        self.sizes = [int(s) for s in sizes]
        self.activation = activation
        self.out_activation = out_activation
        rng = np.random.default_rng(seed)
        self.layers: List[Dense] = []
        for i in range(len(self.sizes) - 1):
            act = out_activation if i == len(self.sizes) - 2 else activation
            self.layers.append(Dense(self.sizes[i], self.sizes[i + 1], rng, act))
        self.mean = np.zeros(self.sizes[0]) if mean is None else np.asarray(mean, dtype=np.float64)
        self.std = np.ones(self.sizes[0]) if std is None else np.asarray(std, dtype=np.float64)

    # ------------------------------------------------------------------ math
    def forward(self, x: np.ndarray, cache: bool = True) -> np.ndarray:
        out = np.asarray(x, dtype=np.float64)
        for layer in self.layers:
            out = layer.forward(out, cache=cache)
        return out

    def backward(self, grad: np.ndarray) -> None:
        for layer in reversed(self.layers):
            grad = layer.backward(grad)

    def params(self) -> Dict[str, np.ndarray]:
        p: Dict[str, np.ndarray] = {}
        for i, layer in enumerate(self.layers):
            p[f"W{i}"] = layer.W
            p[f"b{i}"] = layer.b
        return p

    def grads(self) -> Dict[str, np.ndarray]:
        g: Dict[str, np.ndarray] = {}
        for i, layer in enumerate(self.layers):
            if "W" in layer.grads:
                g[f"W{i}"] = layer.grads["W"]
                g[f"b{i}"] = layer.grads["b"]
        return g

    def normalize(self, x: np.ndarray) -> np.ndarray:
        return (np.asarray(x, dtype=np.float64) - self.mean) / np.where(self.std < 1e-8, 1.0, self.std)

    def predict(self, x: np.ndarray, normalize: bool = True) -> np.ndarray:
        if normalize:
            x = self.normalize(x)
        return self.forward(x, cache=False)

    # ------------------------------------------------------------------ io
    def save(self, path: str) -> None:
        arrays = {f"W{i}": layer.W for i, layer in enumerate(self.layers)}
        arrays.update({f"b{i}": layer.b for i, layer in enumerate(self.layers)})
        arrays["mean"] = self.mean
        arrays["std"] = self.std
        np.savez_compressed(path, **arrays)
        meta = {
            "type": "mlp",
            "sizes": self.sizes,
            "activation": self.activation,
            "out_activation": self.out_activation,
        }
        with open(os.path.splitext(path)[0] + ".json", "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

    @classmethod
    def load(cls, path: str) -> "MLP":
        meta_path = os.path.splitext(path)[0] + ".json"
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)
        data = np.load(path)
        model = cls(meta["sizes"], meta.get("activation", "relu"),
                    meta.get("out_activation", "linear"))
        for i, layer in enumerate(model.layers):
            layer.W = np.asarray(data[f"W{i}"], dtype=np.float64)
            layer.b = np.asarray(data[f"b{i}"], dtype=np.float64)
        model.mean = np.asarray(data["mean"], dtype=np.float64)
        model.std = np.asarray(data["std"], dtype=np.float64)
        return model


def _clip_global_norm(grads: Dict[str, np.ndarray], max_norm: float = 5.0) -> None:
    total = math.sqrt(sum(float(np.sum(g * g)) for g in grads.values()))
    if total > max_norm and total > 0:
        scale = max_norm / total
        for k in grads:
            grads[k] *= scale


def train_model(model, X: np.ndarray, y: np.ndarray, epochs: int = 30,
                batch_size: int = 64, lr: float = 1e-3,
                loss: str = "mse", val_fraction: float = 0.1,
                seed: int = 0, verbose: bool = True,
                class_weights: Optional[np.ndarray] = None) -> Dict[str, list]:
    """Train an MLP/CNN with Adam.  ``loss='ce'`` expects integer labels."""
    X = np.asarray(X, dtype=np.float64)
    y = np.asarray(y)
    rng = np.random.default_rng(seed)
    n = len(X)
    idx = rng.permutation(n)
    n_val = max(1, int(n * val_fraction)) if val_fraction > 0 else 0
    val_idx = idx[:n_val]
    train_idx = idx[n_val:] if n_val else idx

    optimizer = Adam(model.params(), lr=lr)
    history: Dict[str, list] = {"train_loss": [], "val_loss": [], "val_metric": []}
    for epoch in range(epochs):
        rng.shuffle(train_idx)
        losses = []
        for start in range(0, len(train_idx), batch_size):
            batch = train_idx[start:start + batch_size]
            xb = model.normalize(X[batch]) if hasattr(model, "normalize") and model.sizes[0] == X.shape[1] else X[batch]
            pred = model.forward(xb)
            if loss == "ce":
                loss_val, grad = softmax_cross_entropy(y[batch].astype(int), pred)
                if class_weights is not None:
                    # Reweight the per-sample cross entropy (approximate).
                    w = class_weights[y[batch].astype(int)][:, None]
                    grad = grad * w
            else:
                loss_val, grad = mse(y[batch], pred)
            model.backward(grad)
            grads = model.grads()
            _clip_global_norm(grads)
            optimizer.step(grads)
            losses.append(loss_val)
        train_loss = float(np.mean(losses)) if losses else 0.0
        if n_val:
            xv = model.normalize(X[val_idx]) if hasattr(model, "normalize") and model.sizes[0] == X.shape[1] else X[val_idx]
            pv = model.forward(xv, cache=False)
            if loss == "ce":
                val_loss, _ = softmax_cross_entropy(y[val_idx].astype(int), pv)
                metric = float(np.mean(np.argmax(pv, axis=1) == y[val_idx]))
            else:
                val_loss, _ = mse(y[val_idx], pv)
                metric = float(np.sqrt(val_loss))
        else:
            val_loss, metric = train_loss, 0.0
        history["train_loss"].append(train_loss)
        history["val_loss"].append(float(val_loss))
        history["val_metric"].append(float(metric))
        if verbose and (epoch % max(1, epochs // 10) == 0 or epoch == epochs - 1):
            print(f"epoch {epoch + 1:3d}/{epochs}  train={train_loss:.6f}  val={val_loss:.6f}  metric={metric:.4f}")
    return history
