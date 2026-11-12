#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""04 标准化兜底扫描：X_std 的下限取多少，网络才学得动。

回答的问题：256 个池化格里约 51 个在所有样本上几乎恒定（背景像素），
用 1e-3 兜底会把浮点噪声放大上千倍，变成纯噪声输入。扫描不同兜底值，
用同一个 MLP、同一份数据、同一组超参，看验证 R² 怎么变。

用法:
    python3 diag/04_std_floor_scan.py --dataset data/diag_probe.npz
    python3 diag/04_std_floor_scan.py --dataset data/diag_probe.npz --epochs 120

实测(云主机, 1200 条):
    floor    val_R²(物理)   踝 pitch 预测 std / 目标 std
    1e-3       0.3011       72.65 / 85.46
    0.005      0.7243       84.67 / 85.46
    0.01       0.9657       87.02 / 85.46
    0.02       0.9817       85.07 / 85.46   <- 最终取值
    0.05       0.9271       87.83 / 85.46
（这里的预测 std 依赖随机种子，数值与文档略有出入属正常）
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import DATASET, load_xy, require  # noqa: E402

from humanoid_e2e_recovery.e2e_nn import E2EMLP  # noqa: E402


def r2_mean(pred, true):
    out = []
    for j in range(true.shape[1]):
        if true[:, j].std() < 1e-6:
            continue
        ss_res = ((true[:, j] - pred[:, j]) ** 2).sum()
        ss_tot = ((true[:, j] - true[:, j].mean()) ** 2).sum()
        out.append(1 - ss_res / max(ss_tot, 1e-12))
    return float(np.mean(out))


def run(floor, Xtr, Ytr, Xv, Yv, epochs, lr, batch, seed):
    Xm = Xtr.mean(0)
    Xs = np.maximum(Xtr.std(0), floor)
    Ym = Ytr.mean(0)
    Ysc = np.where(Ytr.std(0) > 1e-6, Ytr.std(0), 1.0)

    Xtn = (Xtr - Xm) / Xs
    Ytn = (Ytr - Ym) / Ysc
    Xvn = (Xv - Xm) / Xs

    net = E2EMLP(256, 128, 64, 15, seed=seed)
    r = np.random.default_rng(seed)
    for ep in range(epochs):
        perm = r.permutation(len(Xtn))
        cur_lr = lr if ep < epochs * 0.6 else lr * 0.5
        for i in range(0, len(Xtn) - batch + 1, batch):
            b = perm[i:i + batch]
            net.train_step(Xtn[b], Ytn[b], lr=cur_lr)
    pred = net.predict(Xvn) * Ysc + Ym
    return r2_mean(pred, Yv), pred


def main():
    p = argparse.ArgumentParser(description="X_std 兜底值扫描")
    p.add_argument("--dataset", default=DATASET)
    p.add_argument("--floors", nargs="+", type=float,
                   default=[1e-3, 0.003, 0.005, 0.01, 0.02, 0.05])
    p.add_argument("--epochs", type=int, default=200)
    p.add_argument("--lr", type=float, default=0.005)
    p.add_argument("--batch", type=int, default=64)
    p.add_argument("--seed", type=int, default=42)
    a = p.parse_args()
    require(a.dataset, "先用 diag/02_probe_strong_push.py 生成探针数据，或 --collect")

    X, Y = load_xy(a.dataset)
    idx = np.random.default_rng(0).permutation(len(Y))
    X, Y = X[idx], Y[idx]
    nv = max(1, int(len(Y) * 0.2))
    Xv, Yv, Xtr, Ytr = X[:nv], Y[:nv], X[nv:], Y[nv:]

    print("样本 {}（训练 {} / 验证 {}），epochs={}".format(len(Y), len(Ytr), len(Yv), a.epochs))
    print("\n{:<8} {:>14} {:>24}".format("floor", "val_R2(物理)", "踝pitch预测std/目标std"))
    best = None
    for floor in a.floors:
        score, pred = run(floor, Xtr, Ytr, Xv, Yv, a.epochs, a.lr, a.batch, a.seed)
        print("{:<8} {:>+14.4f} {:>16.2f} / {:.2f}".format(
            floor, score, pred[:, 3].std(), Yv[:, 3].std()))
        if best is None or score > best[1]:
            best = (floor, score)
    print("\n最优 floor = {}（R² {:+.4f}）；训练/推理必须使用同一个值".format(best[0], best[1]))


if __name__ == "__main__":
    main()
