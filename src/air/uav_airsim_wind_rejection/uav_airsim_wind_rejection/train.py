"""训练抗风补偿网络。"""
from __future__ import annotations

import os

import numpy as np

from .mlp import MLP, Adam


def train(X, Y, epochs=120, batch=16, lr=2e-3, val_ratio=0.15, seed=0,
          hidden=(32,), l2=1e-3,
          out_path="models_trained/wind.npz", verbose=True):
    rng = np.random.RandomState(seed)
    n = len(X)
    perm = rng.permutation(n)
    n_val = max(1, int(n * val_ratio))
    val_idx, tr_idx = perm[:n_val], perm[n_val:]

    net = MLP(n_in=X.shape[1], hidden=hidden, n_out=Y.shape[1], seed=seed)
    opt = Adam(net, lr=lr)
    best = {k: v.copy() for k, v in net.params().items()}
    best_val = float("inf")

    for ep in range(epochs):
        order = tr_idx[rng.permutation(len(tr_idx))]
        acc_sum = 0.0
        n_batch = 0
        for start in range(0, len(order), batch):
            bidx = order[start:start + batch]
            accW = [np.zeros_like(w) for w in net.W]
            accb = [np.zeros_like(b) for b in net.b]
            for j in bidx:
                y = net.forward(X[j]).copy()
                err = y - Y[j]
                acc_sum += float(np.mean(err ** 2)) / len(bidx)
                dout = err * (1.0 - y ** 2) * 2.0 / len(bidx)
                gW, gb = net.backward(dout.astype(np.float32))
                for i in range(len(gW)):
                    accW[i] += gW[i] + l2 * net.W[i]
                    accb[i] += gb[i]
            opt.step(accW, accb)
            n_batch += 1

        val = 0.0
        for j in val_idx:
            val += float(np.mean((net.forward(X[j]) - Y[j]) ** 2))
        val /= len(val_idx)
        if val < best_val:
            best_val = val
            best = {k: v.copy() for k, v in net.params().items()}
        if verbose and (ep % 10 == 0 or ep == epochs - 1):
            print("epoch %3d  训练 MSE %.5f  验证 MSE %.5f  最优 %.5f"
                  % (ep, acc_sum / max(1, n_batch), val, best_val), flush=True)

    net.load_state(best)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    net.save(out_path)

    preds = np.array([net.forward(X[j]) for j in val_idx])
    mse = float(np.mean((preds - Y[val_idx]) ** 2))
    r2 = 1.0 - float(np.sum((preds - Y[val_idx]) ** 2)) / float(
        np.sum((Y[val_idx] - Y[val_idx].mean(axis=0)) ** 2))
    if verbose:
        print("验证 MSE %.5f  R2 %.3f  权重 -> %s" % (mse, r2, out_path))
    return net, {"val_mse": mse, "val_r2": r2}
