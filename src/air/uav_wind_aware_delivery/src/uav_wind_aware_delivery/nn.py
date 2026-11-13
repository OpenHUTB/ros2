# -*- coding: utf-8 -*-
"""纯 NumPy 落点预测 MLP（不依赖 PyTorch/TensorFlow）。

输入 6 维：释放高度、三轴初速度、风矢量 (wx,wy)；
输出 2 维：相对释放点的落点偏移 (dx,dy)。
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass

import numpy as np


@dataclass
class TrainReport:
    train_rmse: float
    val_rmse: float
    epochs: int
    samples: int

    def as_dict(self):
        return {
            "train_rmse_m": round(float(self.train_rmse), 5),
            "val_rmse_m": round(float(self.val_rmse), 5),
            "epochs": int(self.epochs),
            "samples": int(self.samples),
        }


class ImpactMLP:
    """两层 tanh MLP + Adam；输入输出都做标准化。"""

    def __init__(self, n_in: int = 6, n_out: int = 2, hidden=(64, 64), seed: int = 7):
        rng = np.random.default_rng(seed)
        sizes = [n_in, *hidden, n_out]
        self.W = []
        self.b = []
        for a, b in zip(sizes[:-1], sizes[1:]):
            # Xavier 初始化
            self.W.append(rng.normal(0.0, np.sqrt(2.0 / (a + b)), size=(a, b)))
            self.b.append(np.zeros(b))
        self.x_mean = np.zeros(n_in)
        self.x_std = np.ones(n_in)
        self.y_mean = np.zeros(n_out)
        self.y_std = np.ones(n_out)
        self.report: TrainReport | None = None

    # ------------------------------------------------------------- forward
    def _forward(self, X):
        h = X
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            h = h @ W + b
            if i < len(self.W) - 1:
                h = np.tanh(h)
        return h

    def predict(self, X: np.ndarray) -> np.ndarray:
        X = np.asarray(X, dtype=float)
        one = X.ndim == 1
        if one:
            X = X[None, :]
        Xn = (X - self.x_mean) / self.x_std
        y = self._forward(Xn) * self.y_std + self.y_mean
        return y[0] if one else y

    # --------------------------------------------------------------- train
    def fit(self, X, y, epochs: int = 260, lr: float = 0.006,
            batch: int = 256, val_ratio: float = 0.15, seed: int = 11) -> TrainReport:
        X = np.asarray(X, dtype=float)
        y = np.asarray(y, dtype=float)
        rng = np.random.default_rng(seed)
        n = len(X)
        idx = rng.permutation(n)
        n_val = max(1, int(n * val_ratio))
        val_idx, tr_idx = idx[:n_val], idx[n_val:]

        self.x_mean = X[tr_idx].mean(axis=0)
        self.x_std = X[tr_idx].std(axis=0) + 1e-8
        self.y_mean = y[tr_idx].mean(axis=0)
        self.y_std = y[tr_idx].std(axis=0) + 1e-8
        Xn = (X - self.x_mean) / self.x_std
        Yn = (y - self.y_mean) / self.y_std

        params = self.W + self.b
        m = [np.zeros_like(p) for p in params]
        v = [np.zeros_like(p) for p in params]
        beta1, beta2, eps = 0.9, 0.999, 1e-8
        step = 0
        best_val = float("inf")
        best = None
        n_tr = len(tr_idx)
        for epoch in range(1, epochs + 1):
            order = rng.permutation(n_tr)
            for start in range(0, n_tr, batch):
                take = order[start:start + batch]
                xb = Xn[tr_idx[take]]
                yb = Yn[tr_idx[take]]
                # forward 缓存
                acts = [xb]
                h = xb
                for i, (W, b) in enumerate(zip(self.W, self.b)):
                    h = h @ W + b
                    if i < len(self.W) - 1:
                        h = np.tanh(h)
                    acts.append(h)
                # backward
                gW = [None] * len(self.W)
                gb = [None] * len(self.b)
                delta = (acts[-1] - yb) * (2.0 / len(xb))
                for i in range(len(self.W) - 1, -1, -1):
                    gW[i] = acts[i].T @ delta
                    gb[i] = delta.sum(axis=0)
                    if i > 0:
                        delta = (delta @ self.W[i].T) * (1.0 - acts[i] ** 2)
                step += 1
                for i in range(len(self.W)):
                    for arr, grad, mm, vv in ((self.W[i], gW[i], m[i], v[i]),
                                              (self.b[i], gb[i],
                                               m[len(self.W) + i], v[len(self.W) + i])):
                        mm *= beta1
                        mm += (1 - beta1) * grad
                        vv *= beta2
                        vv += (1 - beta2) * grad * grad
                        mh = mm / (1 - beta1 ** step)
                        vh = vv / (1 - beta2 ** step)
                        arr -= lr * mh / (np.sqrt(vh) + eps)

            val_pred = self.predict(X[val_idx])
            val_rmse = float(np.sqrt(np.mean((val_pred - y[val_idx]) ** 2)))
            if val_rmse < best_val:
                best_val = val_rmse
                best = ([w.copy() for w in self.W], [b.copy() for b in self.b])
        if best is not None:
            self.W, self.b = best

        tr_pred = self.predict(X[tr_idx])
        tr_rmse = float(np.sqrt(np.mean((tr_pred - y[tr_idx]) ** 2)))
        self.report = TrainReport(tr_rmse, best_val, epochs, n)
        return self.report

    # ----------------------------------------------------------------- io
    def save(self, path: str, meta: dict | None = None):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        arrays = {"x_mean": self.x_mean, "x_std": self.x_std,
                  "y_mean": self.y_mean, "y_std": self.y_std}
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            arrays[f"W{i}"] = W
            arrays[f"b{i}"] = b
        np.savez(path, **arrays)
        if meta is not None:
            with open(os.path.splitext(path)[0] + ".json", "w", encoding="utf-8") as f:
                json.dump(meta, f, ensure_ascii=False, indent=2)

    @classmethod
    def load(cls, path: str) -> "ImpactMLP":
        z = np.load(path)
        n_layers = len([k for k in z.files if k.startswith("W")])
        n_in = z["W0"].shape[0]
        n_out = z[f"W{n_layers - 1}"].shape[1]
        model = cls(n_in=n_in, n_out=n_out,
                    hidden=tuple(z[f"W{i}"].shape[1] for i in range(n_layers - 1)))
        model.W = [z[f"W{i}"].copy() for i in range(n_layers)]
        model.b = [z[f"b{i}"].copy() for i in range(n_layers)]
        model.x_mean = z["x_mean"].copy()
        model.x_std = z["x_std"].copy()
        model.y_mean = z["y_mean"].copy()
        model.y_std = z["y_std"].copy()
        return model
