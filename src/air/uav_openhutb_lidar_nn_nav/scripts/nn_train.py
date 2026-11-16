#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务4：训练"激光扫描 → 速度指令"网络（行为克隆，纯 NumPy）。

用法
    python3 nn_train.py <数据目录> [训练轮数]
数据
    <数据目录>/scan_dataset.csv，由 dataset_recorder.py 生成
产物
    <数据目录>/scan_policy.npz   权重 + 归一化参数
"""

import csv
import os
import sys

import numpy as np


def load(path):
    rows = list(csv.reader(open(path)))
    head = rows[0]
    data = np.array([[float(v) for v in r] for r in rows[1:]], dtype=np.float32)
    return data[:, :-2], data[:, -2:]


def main():
    root = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/openhutb_nn_data")
    epochs = int(sys.argv[2]) if len(sys.argv) > 2 else 300
    hidden = int(sys.argv[3]) if len(sys.argv) > 3 else 96

    x, y = load(os.path.join(root, "scan_dataset.csv"))
    print("样本数:", x.shape)
    if len(x) < 50:
        print("样本太少，先运行 --mode record 采集数据")
        return 1

    # 按时间顺序切分，避免相邻帧泄漏
    cut = int(len(x) * 0.85)
    xtr, ytr, xva, yva = x[:cut], y[:cut], x[cut:], y[cut:]

    xm, xs = xtr.mean(0), xtr.std(0) + 1e-6
    ym, ys = ytr.mean(0), ytr.std(0) + 1e-6
    xtr = (xtr - xm) / xs
    xva = (xva - xm) / xs
    ytr = (ytr - ym) / ys
    yva = (yva - ym) / ys

    rng = np.random.RandomState(0)
    d = x.shape[1]
    W1 = rng.randn(d, hidden).astype(np.float32) * np.sqrt(2.0 / d)
    b1 = np.zeros(hidden, dtype=np.float32)
    W2 = rng.randn(hidden, 2).astype(np.float32) * np.sqrt(2.0 / hidden)
    b2 = np.zeros(2, dtype=np.float32)
    mW1 = np.zeros_like(W1); mb1 = np.zeros_like(b1)
    mW2 = np.zeros_like(W2); mb2 = np.zeros_like(b2)

    lr, mom, l2 = 2e-3, 0.9, 1e-5
    best, best_pack = 1e9, None

    for ep in range(epochs):
        perm = rng.permutation(len(xtr))
        for i in range(0, len(perm), 64):
            b = perm[i:i + 64]
            xb, yb = xtr[b], ytr[b]
            h = np.maximum(xb @ W1 + b1, 0.0)
            o = h @ W2 + b2
            go = (o - yb) * (2.0 / len(b))
            gW2 = h.T @ go + l2 * W2
            gb2 = go.sum(0)
            gh = (go @ W2.T) * (h > 0)
            gW1 = xb.T @ gh + l2 * W1
            gb1 = gh.sum(0)
            mW1 = mom * mW1 - lr * gW1; W1 += mW1
            mb1 = mom * mb1 - lr * gb1; b1 += mb1
            mW2 = mom * mW2 - lr * gW2; W2 += mW2
            mb2 = mom * mb2 - lr * gb2; b2 += mb2

        hv = np.maximum(xva @ W1 + b1, 0.0)
        vl = float(np.mean((hv @ W2 + b2 - yva) ** 2))
        if vl < best:
            best, best_pack = vl, (W1.copy(), b1.copy(), W2.copy(), b2.copy())
        if ep % 50 == 0 or ep == epochs - 1:
            print("epoch %3d  val %.5f" % (ep, vl))

    W1, b1, W2, b2 = best_pack
    out = os.path.join(root, "scan_policy.npz")
    np.savez(out, W1=W1, b1=b1, W2=W2, b2=b2, xm=xm, xs=xs, ym=ym, ys=ys)
    print("best val %.5f -> %s" % (best, out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
