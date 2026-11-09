"""训练 TTC 预测网络（逐样本 SGD + Adam）。"""
from __future__ import annotations

import os

import numpy as np

from .cnn import Adam, DepthPolicyNet


def train(X, Y, epochs=60, lr=5e-4, val_ratio=0.15, seed=0,
          out_path="models_trained/ttc.npz", verbose=True):
    if X.ndim == 4:
        X = X[:, 0]
    rng = np.random.RandomState(seed)
    n = len(X)
    perm = rng.permutation(n)
    n_val = max(1, int(n * val_ratio))
    val_idx, tr_idx = perm[:n_val], perm[n_val:]

    net = DepthPolicyNet(in_size=X.shape[1], seed=seed)
    opt = Adam(net, lr=lr)
    best = {k: v.copy() for k, v in net.params().items()}
    best_val = float("inf")

    for ep in range(epochs):
        order = tr_idx[rng.permutation(len(tr_idx))]
        acc = 0.0
        for j in order:
            y = net.forward(X[j]).copy()
            err = y - Y[j]
            acc += float(np.mean(err ** 2))
            dout = err * (1.0 - y ** 2) * 2.0
            net.backward(dout.astype(np.float32))
            opt.step()

        val = 0.0
        for j in val_idx:
            val += float(np.mean((net.forward(X[j]) - Y[j]) ** 2))
        val /= len(val_idx)
        if val < best_val:
            best_val = val
            best = {k: v.copy() for k, v in net.params().items()}
        if verbose and (ep % 5 == 0 or ep == epochs - 1):
            print("epoch %3d  训练 MSE %.5f  验证 MSE %.5f  最优 %.5f"
                  % (ep, acc / len(order), val, best_val), flush=True)

    net.load_state(best)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    net.save(out_path)

    preds = np.array([float(net.forward(X[j])[0]) for j in val_idx])
    tgt = Y[val_idx, 0]
    mse = float(np.mean((preds - tgt) ** 2))
    var = float(np.sum((tgt - tgt.mean()) ** 2))
    r2 = 1.0 - float(np.sum((preds - tgt) ** 2)) / var if var > 1e-9 else 0.0
    if verbose:
        print("验证 MSE %.5f  R2 %.3f  权重 -> %s" % (mse, r2, out_path))
    return net, {"val_mse": mse, "val_r2": r2}
