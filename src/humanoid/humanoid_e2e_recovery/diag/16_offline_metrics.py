#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""16_offline_metrics.py —— 只用 NumPy 复算留出集指标（不需要 MuJoCo）

回答评审最常问的三件事，全部可在无仿真环境的机器上跑：

1. 文档里的 val MSE / RMSE / R² 能不能复现；
2. 第一层 ReLU 是不是"整片死亡"（z1 对真实样本是否全负）；
3. 输出是否真的随输入变化（跨样本逐维 std / 极差），还是恒为常数。

它不 import 本包的 sim/dataset（那两个模块依赖 mujoco），因此纯 numpy 可跑。
双帧配对与 80/20 划分的口径与 train.py 完全一致（同样的 lag、seed、val_ratio），
只是这里用**随包权重里存的** X_mean/X_std 做标准化——那正是推理端的真实口径。
脚本会额外核对"权重里的标准化参数"与"数据集现算值"是否一致（max|Δ|），
以排除"训练/推理口径错配"这类静默失败。

用法：
    python3 diag/16_offline_metrics.py
    python3 diag/16_offline_metrics.py --dataset data/e2e_dataset.npz \\
        --weights models_trained/e2e_policy.npz --val-ratio 0.2 --seed 0
"""
from __future__ import annotations

import argparse
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def make_pairs(X, Y, ep, lag):
    """与 dataset.make_pairs 同口径：只在同一回合内配对。"""
    ok = np.asarray(ep[lag:]) == np.asarray(ep[:-lag])
    idx = np.nonzero(ok)[0] + lag
    return np.concatenate([X[idx - lag], X[idx]], axis=1), Y[idx]


def relu(x):
    return np.maximum(0.0, x)


def main():
    p = argparse.ArgumentParser(description="纯 NumPy 复算留出集指标")
    p.add_argument("--dataset", default=os.path.join(ROOT, "data", "e2e_dataset.npz"),
                   help="数据集 npz（可从 Release e2e-dataset-v2 下载）")
    p.add_argument("--weights", default=os.path.join(ROOT, "models_trained", "e2e_policy.npz"),
                   help="权重 npz")
    p.add_argument("--val-ratio", type=float, default=0.2)
    p.add_argument("--seed", type=int, default=0)
    a = p.parse_args()

    for path, hint in ((a.dataset, "先下载：https://github.com/WZJ6666wudu/ros2/releases/download/"
                                    "e2e-dataset-v2/e2e_dataset.npz"),
                       (a.weights, "权重随包提供 models_trained/e2e_policy.npz")):
        if not os.path.exists(path):
            raise SystemExit("[错误] 找不到 %s\n  %s" % (path, hint))

    d = np.load(a.dataset)
    w = np.load(a.weights)
    X, Y = d["X"].astype(np.float64), d["Y"].astype(np.float64)
    print("[数据] 样本 %d 条 | X=%s Y=%s | 键 %s" % (len(X), X.shape, Y.shape, sorted(d.files)))

    lag_sec = float(w["X_LAG_SEC"]) if "X_LAG_SEC" in w.files else 0.0
    lag = int(w["X_LAG"]) if "X_LAG" in w.files else 0
    if lag > 0 and "ep" in d.files:
        sample_dt = float(d["sample_dt"]) if "sample_dt" in d.files else 0.006
        lag_impl = max(1, int(round(lag_sec / sample_dt)))
        X, Y = make_pairs(X, Y, d["ep"], lag_impl)
        print("[配对] lag_sec=%.3f → lag=%d 个采样（数据里实现为 %d）| 配成 %d 对 | X=%s"
              % (lag_sec, lag, lag_impl, len(Y), X.shape))
    else:
        print("[配对] 单帧权重（X_LAG_SEC=0），直接用原始样本")

    # —— 标准化：用权重里存的参数（推理口径），并与数据集现算值核对 ——
    X_mean, X_std = w["X_mean"].astype(np.float64), w["X_std"].astype(np.float64)
    Y_mean, Y_scale = w["Y_mean"].astype(np.float64), w["Y_scale"].astype(np.float64)
    xm_now = X.mean(axis=0)
    xs_now = np.maximum(X.std(axis=0), 0.02)
    print("[标准化] 维度核对: X_mean(%d) X_std(%d) Y_mean(%d) Y_scale(%d)%s"
          % (len(X_mean), len(X_std), len(Y_mean), len(Y_scale),
             "" if len(X_mean) == X.shape[1] else "  ← 与输入维度不匹配！"))
    if len(X_mean) == X.shape[1]:
        print("[标准化] 权重 vs 数据现算: max|ΔX_mean|=%.3e  max|ΔX_std|=%.3e（应为 0 量级）"
              % (np.abs(X_mean - xm_now).max(), np.abs(X_std - xs_now).max()))

    Xn = (X - X_mean) / X_std
    Yn = (Y - Y_mean) / Y_scale

    # —— 与 train.py 一致的 80/20 划分（同一个 seed、同一套排列）——
    rng = np.random.default_rng(a.seed)
    idx = rng.permutation(len(Yn))
    Xn, Yn, Xr, Yr = Xn[idx], Yn[idx], X[idx], Y[idx]
    nv = max(1, int(len(Yn) * a.val_ratio))
    Xv, Yv, Xvr = Xn[:nv], Yn[:nv], Xr[:nv]

    # —— 前向 ——
    z1 = Xv @ w["W1"] + w["b1"]
    a1 = relu(z1)
    z2 = a1 @ w["W2"] + w["b2"]
    a2 = relu(z2)
    pred_n = a2 @ w["W3"] + w["b3"]
    pred_phys = pred_n * Y_scale + Y_mean
    true_phys = Yv * Y_scale + Y_mean

    mse_n = float(np.mean((pred_n - Yv) ** 2))
    mse_p = float(np.mean((pred_phys - true_phys) ** 2))
    rmse_p = float(np.sqrt(mse_p))
    ybar = true_phys.mean(axis=0)
    sst = float(np.sum((true_phys - ybar) ** 2))
    sse = float(np.sum((pred_phys - true_phys) ** 2))
    r2 = 1.0 - sse / sst
    base_rmse = float(np.sqrt(np.mean((ybar[None, :] - true_phys) ** 2)))
    alive = int((z1 > 0).any(axis=0).sum())
    rate1 = float(np.mean(z1 > 0))
    alive2 = int((z2 > 0).any(axis=0).sum())
    rate2 = float(np.mean(z2 > 0))
    std_pred = pred_phys.std(axis=0)
    rng_pred = pred_phys.max(axis=0) - pred_phys.min(axis=0)
    std_true = true_phys.std(axis=0)

    print("\n[留出集] %d 对（val_ratio=%.2f, seed=%d）" % (nv, a.val_ratio, a.seed))
    print("  val MSE（标准化）   = %.5f" % mse_n)
    print("  val MSE（物理 N·m²）= %.4f   RMSE = %.4f N·m" % (mse_p, rmse_p))
    print("  R²（物理，全部维度） = %+.4f   （常数基线 RMSE = %.4f N·m，R²=0）"
          % (r2, base_rmse))
    print("  第一层 ReLU：存活 %d/%d（激活率 %.2f%%）" % (alive, z1.shape[1], rate1 * 100))
    print("  第二层 ReLU：存活 %d/%d（激活率 %.2f%%）" % (alive2, z2.shape[1], rate2 * 100))
    print("  输出跨样本逐维 std（物理）：最大 %.4f N·m（第 %d 维），目标同维 %.4f N·m"
          % (std_pred.max(), int(std_pred.argmax()), std_true[std_pred.argmax()]))
    print("  输出跨样本极差（物理）：最大 %.4f N·m（第 %d 维）"
          % (rng_pred.max(), int(rng_pred.argmax())))
    print("  踝 pitch 两维（idx 3 / 8）：std 预测 %.4f / %.4f N·m，目标 %.4f / %.4f N·m"
          % (std_pred[3], std_pred[8], std_true[3], std_true[8]))

    ok = (alive > 0 and rate1 > 0.05 and rng_pred.max() > 0 and r2 > 0)
    print("\n[判定] %s：%s" % ("PASS" if ok else "FAIL",
          "网络受输入驱动且优于常数基线" if ok
          else "未过（检查第一层存活率 / 跨样本变化 / R²）"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
