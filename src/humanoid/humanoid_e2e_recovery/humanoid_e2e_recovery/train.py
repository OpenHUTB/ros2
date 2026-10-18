"""train.py —— 训练端到端 MLP（图像→关节角回归）。"""
from __future__ import annotations
import os
import numpy as np
from .e2e_nn import E2EMLP


def train(dataset_path, out_weight_path, in_dim=256, h1=128, h2=64, out_dim=15,
          epochs=60, batch=64, lr=0.005, val_ratio=0.2, seed=0, verbose=True):
    d = np.load(dataset_path)
    X, Y = d["X"].astype(np.float64), d["Y"].astype(np.float64)
    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(Y))
    X, Y = X[idx], Y[idx]
    nv = max(1, int(len(Y)*val_ratio))
    Xv, Yv = X[:nv], Y[:nv]
    Xt, Yt = X[nv:], Y[nv:]
    model = E2EMLP(in_dim, h1, h2, out_dim, seed=seed)
    for ep in range(epochs):
        perm = rng.permutation(len(Yt))
        Xt, Yt = Xt[perm], Yt[perm]
        losses = []
        for i in range(0, len(Yt), batch):
            losses.append(model.train_step(Xt[i:i+batch], Yt[i:i+batch], lr=lr))
        pred_v = model.predict(Xv)
        val_mse = float(np.mean((pred_v - Yv)**2))
        if verbose and (ep % 10 == 0 or ep == epochs-1):
            print(f"  epoch {ep:3d}/{epochs} | train_loss={np.mean(losses):.5f} | val_mse={val_mse:.5f}")
    os.makedirs(os.path.dirname(out_weight_path) or ".", exist_ok=True)
    model.save(out_weight_path)
    if verbose:
        print(f"[train] 权重: {out_weight_path} | 最终 val_mse={val_mse:.5f}")
    return {"val_mse": val_mse, "weight_path": out_weight_path}
