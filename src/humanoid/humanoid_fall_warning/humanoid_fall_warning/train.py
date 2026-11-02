"""train.py —— 训练多模态跌倒预警网络"""
from __future__ import annotations
import os
import numpy as np
from .nn import FallWarningMLP


def train(dataset_path, out_path, input_dim=258, hidden1=128, hidden2=64,
          output_dim=2, epochs=80, batch_size=64, lr=0.01, seed=0, verbose=True):
    data = np.load(dataset_path)
    X, y = data["X"].astype(np.float64), data["y"].astype(np.int64)
    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(y))
    X, y = X[idx], y[idx]
    n_val = max(1, int(len(y)*0.2))
    X_val, y_val = X[:n_val], y[:n_val]
    X_tr, y_tr = X[n_val:], y[n_val:]
    model = FallWarningMLP(input_dim, hidden1, hidden2, output_dim, seed=seed)
    for ep in range(epochs):
        perm = rng.permutation(len(y_tr))
        X_tr, y_tr = X_tr[perm], y_tr[perm]
        losses = []
        for i in range(0, len(y_tr), batch_size):
            losses.append(model.train_step(X_tr[i:i+batch_size], y_tr[i:i+batch_size], lr=lr))
        if verbose and ep % 20 == 0:
            val_pred = model.predict(X_val).argmax(axis=1)
            acc = float((val_pred == y_val).mean())
            print(f"  epoch {ep}/{epochs} | loss={np.mean(losses):.4f} | val_acc={acc:.3f}")
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    model.save(out_path)
    print(f"[train] saved {out_path}")
    return out_path
