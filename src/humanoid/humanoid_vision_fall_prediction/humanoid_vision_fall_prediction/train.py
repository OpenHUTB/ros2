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
                output_dim: int = 5, epochs: int = 300, batch_size: int = 64,
                lr: float = 0.005, val_ratio: float = 0.2,
                lr_decay: float = 0.8, decay_every: int = 50,
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

    log = {"train_loss": [], "val_acc": [], "train_acc": [], "lr": []}
    best_val = -1.0
    best_weights = None
    for epoch in range(epochs):
        cur_lr = lr * (lr_decay ** (epoch // decay_every))
        # shuffle
        perm = rng.permutation(len(y_train))
        X_train, y_train = X_train[perm], y_train[perm]
        losses = []
        for i in range(0, len(y_train), batch_size):
            xb = X_train[i:i + batch_size]
            yb = y_train[i:i + batch_size]
            losses.append(model.train_step(xb, yb, lr=cur_lr))

        # 评估
        train_pred, _ = model.predict(X_train)
        val_pred, _ = model.predict(X_val)
        train_acc = float((train_pred == y_train).mean())
        val_acc = float((val_pred == y_val).mean())
        log["train_loss"].append(float(np.mean(losses)))
        log["train_acc"].append(train_acc)
        log["val_acc"].append(val_acc)
        log["lr"].append(cur_lr)

        # 记录验证集最优权重
        if val_acc > best_val:
            best_val = val_acc
            best_weights = {k: getattr(model, k).copy()
                            for k in ("W1", "b1", "W2", "b2", "W3", "b3")}

        if verbose and (epoch % 5 == 0 or epoch == epochs - 1):
            print(f"  epoch {epoch:3d}/{epochs} | loss={np.mean(losses):.4f} | "
                  f"train_acc={train_acc:.3f} | val_acc={val_acc:.3f}")

    # 保存验证集最优的一组权重。此前保存的是最后一轮权重，而验证准确率在
    # 小数据集上逐轮剧烈震荡（本例 best 1.00 / worst 0.20），最后一轮常常
    # 恰好落在低点，于是“训练日志看着不错、推理实际很差”。保存 best 后
    # 落盘权重的验证准确率与日志里的 best 一致。
    if best_weights is not None:
        for k, v in best_weights.items():
            setattr(model, k, v)
    os.makedirs(os.path.dirname(out_weight_path) or ".", exist_ok=True)
    model.save(out_weight_path)
    log["final_val_acc"] = log["val_acc"][-1]
    log["final_train_acc"] = log["train_acc"][-1]
    log["best_val_acc"] = float(best_val)
    log["last10_val_acc"] = float(np.mean(log["val_acc"][-10:]))
    log["weight_path"] = out_weight_path

    if verbose:
        print(f"[train] 权重已保存: {out_weight_path}")
        print(f"[train] 最后一轮验证准确率: {log['final_val_acc']:.3f} | "
              f"最优 {log['best_val_acc']:.3f} | 末 10 轮均值 {log['last10_val_acc']:.3f}")
    return log
