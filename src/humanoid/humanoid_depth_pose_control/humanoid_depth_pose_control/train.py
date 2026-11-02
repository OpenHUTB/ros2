"""train.py —— 训练深度图姿态估计网络"""
from __future__ import annotations
import os
import numpy as np
from .nn import DepthPoseMLP


def train(dataset_path, out_path, input_dim=256, hidden1=128, hidden2=64,
          output_dim=3, epochs=80, batch_size=64, lr=0.005, seed=0, verbose=True):
    data = np.load(dataset_path)
    X, y = data["X"].astype(np.float64), data["y"].astype(np.float64)
    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(y))
    X, y = X[idx], y[idx]
    n_val = max(1, int(len(y)*0.2))
    X_val, y_val = X[:n_val], y[:n_val]
    X_tr, y_tr = X[n_val:], y[n_val:]
    model = DepthPoseMLP(input_dim, hidden1, hidden2, output_dim, seed=seed)
    log = {"val_mse": [], "train_mse": []}
    for ep in range(epochs):
        perm = rng.permutation(len(y_tr))
        X_tr, y_tr = X_tr[perm], y_tr[perm]
        losses = []
        for i in range(0, len(y_tr), batch_size):
            losses.append(model.train_step(X_tr[i:i+batch_size], y_tr[i:i+batch_size], lr=lr))
        val_pred = model.predict(X_val)
        val_mse = float(np.mean((val_pred - y_val)**2))
        log["train_mse"].append(float(np.mean(losses)))
        log["val_mse"].append(val_mse)
        if verbose and (ep % 10 == 0 or ep == epochs-1):
            print(f"  epoch {ep:3d}/{epochs} | train_mse={np.mean(losses):.6f} | val_mse={val_mse:.6f}")
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    model.save(out_path)
    log["final_val_mse"] = log["val_mse"][-1]
    log["weight_path"] = out_path
    if verbose:
        print(f"[train] saved {out_path}, final val_mse={log['final_val_mse']:.6f}")
    return log
