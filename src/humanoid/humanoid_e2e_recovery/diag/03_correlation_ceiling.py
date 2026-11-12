#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""03 信息论上限：单帧图像里到底含多少关于力矩的信息。

回答的问题：如果线性模型（岭回归）就能从 256 维池化图里把踝力矩解释到
R²≈1，那"图像信息不足"这个假设就被推翻，剩下的只能是训练侧的问题。

用法:
    python3 diag/03_correlation_ceiling.py --dataset data/diag_probe.npz
    python3 diag/03_correlation_ceiling.py --dataset data/diag_probe.npz --lam 1e-2

实测(云主机, 1200 条 (60,150) N 数据):
  Y[3] 踝 pitch: 单像素 max|corr| = 0.840 (X 维 104)
                岭回归 R² = +0.999 (预测 std 84.80, 目标 84.88)
  Y[0/4/5/9]:   R² = +0.910
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import DATASET, load_xy, require  # noqa: E402


def main():
    p = argparse.ArgumentParser(description="图像-力矩相关性上限")
    p.add_argument("--dataset", default=DATASET,
                   help="数据集 npz（02 生成的探针数据效果最好）")
    p.add_argument("--lam", type=float, default=1e-2, help="岭回归正则系数")
    a = p.parse_args()
    require(a.dataset, "先用 diag/02_probe_strong_push.py 生成探针数据")

    X, Y = load_xy(a.dataset)
    print("样本 {}, X {}, Y {}".format(len(X), X.shape, Y.shape))

    var_idx = [i for i in range(Y.shape[1]) if Y[:, i].std() > 1e-6]
    print("Y 有变化的维: {}".format(var_idx))

    Xn = (X - X.mean(0)) / np.maximum(X.std(0), 1e-9)

    print("\n=== 单个 X 像素与 Y 的最大相关系数 ===")
    for j in var_idx:
        y = Y[:, j] - Y[:, j].mean()
        r = (Xn * y[:, None]).mean(0) / max(y.std(), 1e-12)
        i = int(np.argmax(np.abs(r)))
        print("  Y[{:2d}] (std={:7.2f}): max|corr| = {:.3f}  (X 维 {})".format(
            j, Y[:, j].std(), abs(r[i]), i))

    print("\n=== 线性（岭回归）可达的 R² 上限 ===")
    A = np.hstack([Xn, np.ones((len(X), 1))])
    W = np.linalg.solve(A.T @ A + a.lam * np.eye(A.shape[1]), A.T @ Y)
    pred = A @ W
    for j in var_idx:
        ss_res = ((Y[:, j] - pred[:, j]) ** 2).sum()
        ss_tot = ((Y[:, j] - Y[:, j].mean()) ** 2).sum()
        r2 = 1 - ss_res / max(ss_tot, 1e-12)
        print("  Y[{:2d}]: R² = {:+.3f}  (std 目标 {:.2f}, 预测 {:.2f})".format(
            j, r2, Y[:, j].std(), pred[:, j].std()))

    print("\n=== 相邻样本图像的差分幅度 ===")
    print("  相邻样本平均像素变化 = {:.5f}  (X 值域 {:.3f}~{:.3f})".format(
        np.abs(np.diff(X, axis=0)).mean(), X.min(), X.max()))
    print("  单样本内部逐维 std 均值 = {:.5f}".format(X.std(0).mean()))


if __name__ == "__main__":
    main()
