"""训练姿态估计 MLP：输入标准化 + 标签 z-score + 逐样本 Adam。"""
from __future__ import annotations

import os

import numpy as np

from .mlp import MLP, Adam


def train(X, Y, epochs=120, lr=1e-3, val_ratio=0.15, seed=0,
          out_path="models_trained/attitude.npz", verbose=True):
    rng = np.random.RandomState(seed)

    # 输入标准化：IMU 加速度量级 5~10 m/s^2，角速度不足 1 rad/s，
    # 尺度差一个量级，不标准化会让 tanh 隐层饱和。
    x_mean = X.mean(axis=0, keepdims=True)
    x_std = X.std(axis=0, keepdims=True) + 1e-6
    Xn = (X - x_mean) / x_std

    # 标签标准化：roll/pitch 集中在 0 附近，直接回归会退化成常数。
    y_mean = Y.mean(axis=0, keepdims=True)
    y_std = Y.std(axis=0, keepdims=True) + 1e-6
    Yn = (Y - y_mean) / y_std

    n = len(X)
    perm = rng.permutation(n)
    n_val = max(1, int(n * val_ratio))
    val_idx, tr_idx = perm[:n_val], perm[n_val:]

    net = MLP(n_in=X.shape[1], hidden=(32,), n_out=2, seed=seed)
    opt = Adam(net, lr=lr)
    best = {k: v.copy() for k, v in net.params().items()}
    best_val = float("inf")

    for ep in range(epochs):
        order = tr_idx[rng.permutation(len(tr_idx))]
        acc = 0.0
        for j in order:
            y = net.forward(Xn[j]).copy()
            err = y - Yn[j]
            acc += float(np.mean(err ** 2))
            dout = (2.0 * err).astype(np.float32)
            gw, gb = net.backward(dout)
            opt.step(gw, gb)

        val = 0.0
        for j in val_idx:
            val += float(np.mean((net.forward(Xn[j]) - Yn[j]) ** 2))
        val /= len(val_idx)
        if val < best_val:
            best_val = val
            best = {k: v.copy() for k, v in net.params().items()}
        if verbose and (ep % 20 == 0 or ep == epochs - 1):
            print("epoch %3d  训练 MSE %.5f  验证 MSE %.5f  最优 %.5f"
                  % (ep, acc / len(order), val, best_val), flush=True)

    net.load_state(best)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    params = net.params()
    params["x_mean"] = x_mean.astype(np.float32).reshape(-1)
    params["x_std"] = x_std.astype(np.float32).reshape(-1)
    params["y_mean"] = y_mean.astype(np.float32).reshape(-1)
    params["y_std"] = y_std.astype(np.float32).reshape(-1)
    np.savez(out_path, **params)

    preds = np.array([[float(v) for v in net.forward(Xn[j])] for j in val_idx]) * y_std + y_mean
    tgt = Y[val_idx]
    mse = float(np.mean((preds - tgt) ** 2))
    mae = float(np.mean(np.abs(preds - tgt)))
    var = float(np.sum((tgt - tgt.mean(axis=0)) ** 2))
    r2 = 1.0 - float(np.sum((preds - tgt) ** 2)) / var if var > 1e-9 else 0.0
    if verbose:
        print("验证 MSE %.6f  MAE %.4f rad  R2 %.3f  权重 -> %s" % (mse, mae, r2, out_path))
    return net, {"val_mse": mse, "val_mae_rad": mae, "val_r2": r2}
