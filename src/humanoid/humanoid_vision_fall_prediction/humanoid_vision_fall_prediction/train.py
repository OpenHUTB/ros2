"""
train.py —— 训练跌倒预测 MLP
============================

加载 .npz 数据集，训练/验证划分，SGD+动量训练，输出训练曲线与准确率，
保存权重到 models_trained/fall_predictor.npz。
"""

from __future__ import annotations

import os
import json
import numpy as np

from .nn import FallPredictorMLP


def train_model(dataset_path: str, out_weight_path: str,
                input_dim: int = 256, hidden1: int = 128, hidden2: int = 64,
                output_dim: int = 5, epochs: int = 60, batch_size: int = 64,
                lr: float = 0.01, val_ratio: float = 0.2,
                seed: int = 0, verbose: bool = True) -> dict:
    """训练并返回训练日志。"""
    data = np.load(dataset_path)
    X = data["X"].astype(np.float64)
    y = data["y"].astype(np.int64)

    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(y))
    X, y = X[idx], y[idx]
    n_val = max(1, int(len(y) * val_ratio))
    X_val, y_val = X[:n_val], y[:n_val]
    X_train, y_train = X[n_val:], y[n_val:]

    model = FallPredictorMLP(input_dim, hidden1, hidden2, output_dim, seed=seed)

    log = {"train_loss": [], "val_acc": [], "train_acc": []}
    for epoch in range(epochs):
        # shuffle
        perm = rng.permutation(len(y_train))
        X_train, y_train = X_train[perm], y_train[perm]
        losses = []
        for i in range(0, len(y_train), batch_size):
            xb = X_train[i:i + batch_size]
            yb = y_train[i:i + batch_size]
            losses.append(model.train_step(xb, yb, lr=lr))

        # 评估
        train_pred, _ = model.predict(X_train)
        val_pred, _ = model.predict(X_val)
        train_acc = float((train_pred == y_train).mean())
        val_acc = float((val_pred == y_val).mean())
        log["train_loss"].append(float(np.mean(losses)))
        log["train_acc"].append(train_acc)
        log["val_acc"].append(val_acc)

        if verbose and (epoch % 5 == 0 or epoch == epochs - 1):
            print(f"  epoch {epoch:3d}/{epochs} | loss={np.mean(losses):.4f} | "
                  f"train_acc={train_acc:.3f} | val_acc={val_acc:.3f}")

    os.makedirs(os.path.dirname(out_weight_path) or ".", exist_ok=True)
    model.save(out_weight_path)
    log["final_val_acc"] = log["val_acc"][-1]
    log["final_train_acc"] = log["train_acc"][-1]
    log["weight_path"] = out_weight_path

    if verbose:
        print(f"[train] 权重已保存: {out_weight_path}")
        print(f"[train] 最终验证准确率: {log['final_val_acc']:.3f}")
    return log
