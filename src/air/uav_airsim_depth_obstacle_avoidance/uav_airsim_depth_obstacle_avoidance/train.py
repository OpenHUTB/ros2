"""训练：把专家动作回归到深度图上。"""
from __future__ import annotations

import os

import numpy as np

from .cnn import Adam, DepthPolicyNet


def _zero_grad(acc, net):
    for key, layer in (("conv", net.conv), ("fc1", net.fc1), ("fc2", net.fc2)):
        acc[key + "_W"] = np.zeros_like(layer.W)
        acc[key + "_b"] = np.zeros_like(layer.b)


def _apply_mean_grad(acc, net, n):
    for key, layer in (("conv", net.conv), ("fc1", net.fc1), ("fc2", net.fc2)):
        layer.dW = acc[key + "_W"] / n
        layer.db = acc[key + "_b"] / n


def train(X, Y, epochs=60, batch=8, lr=1e-3, val_ratio=0.15, seed=0,
          out_path="models_trained/depth_avoid.npz", verbose=True):
    rng = np.random.RandomState(seed)
    n = len(X)
    perm = rng.permutation(n)
    n_val = max(1, int(n * val_ratio))
    val_idx, tr_idx = perm[:n_val], perm[n_val:]

    net = DepthPolicyNet(seed=seed)
    opt = Adam(net, lr=lr)
    best = net.params()
    best_val = float("inf")
    acc = {}

    for ep in range(epochs):
        order = tr_idx[rng.permutation(len(tr_idx))]
        total = 0.0
        for start in range(0, len(order), batch):
            bidx = order[start:start + batch]
            _zero_grad(acc, net)
            batch_loss = 0.0
            for j in bidx:
                xj, yj = X[j], Y[j]
                # 左右翻转增强：深度图和转向动作同时镜像，等于免费多一倍样本
                if rng.rand() < 0.5:
                    xj = xj[:, ::-1]
                    yj = np.array([-yj[0], yj[1]], dtype=np.float32)
                y = net.forward(xj).copy()
                err = y - yj
                # 动作大量集中在 0 附近，给大幅度动作加权，避免网络退化成「永远直飞」
                w = 1.0 + 3.0 * abs(float(yj[0]))
                batch_loss += w * float(np.mean(err ** 2))
                dp = err * (1.0 - y ** 2) * 2.0 * w / len(bidx)
                net.backward(dp.astype(np.float32))
                acc["conv_W"] += net.conv.dW
                acc["conv_b"] += net.conv.db
                acc["fc1_W"] += net.fc1.dW
                acc["fc1_b"] += net.fc1.db
                acc["fc2_W"] += net.fc2.dW
                acc["fc2_b"] += net.fc2.db
            _apply_mean_grad(acc, net, len(bidx))
            opt.step()
            total += batch_loss

        # 验证
        val_loss = 0.0
        for j in val_idx:
            y = net.forward(X[j])
            val_loss += float(np.mean((y - Y[j]) ** 2))
        val_loss /= len(val_idx)
        if val_loss < best_val:
            best_val = val_loss
            best = {k: v.copy() for k, v in net.params().items()}
        if verbose and (ep % 5 == 0 or ep == epochs - 1):
            print("epoch %3d  训练 MSE %.5f  验证 MSE %.5f  最优 %.5f"
                  % (ep, total / max(1, len(order) // batch), val_loss, best_val), flush=True)

    net.load_state(best)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    net.save(out_path)

    # 最终指标
    preds = np.array([net.forward(X[j]) for j in val_idx])
    mse = float(np.mean((preds - Y[val_idx]) ** 2))
    r2 = 1.0 - float(np.sum((preds - Y[val_idx]) ** 2)) / float(
        np.sum((Y[val_idx] - Y[val_idx].mean(axis=0)) ** 2))
    if verbose:
        print("验证集 MSE %.5f  R2 %.3f  权重已保存 -> %s" % (mse, r2, out_path))
    return net, {"val_mse": mse, "val_r2": r2}
