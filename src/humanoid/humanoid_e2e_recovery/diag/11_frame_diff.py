#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""11 帧间差分特征（失败路线，保留作证据）：给单帧补上近似的"变化率"。

思路：把 X_t 与 X_{t-1} 的差分拼进输入，等价于补一个速度通道。
结果失败，原因有两个，都在输出里能看到：
  1. 差分逐维 std 只有 0.00038（比单帧还小一个数量级），可学信号太弱；
  2. 给差分维单独设 1e-4 兜底时，噪声被放大 -> 训练出 nan，闭环 0.58 s。

用法:
    python3 diag/11_frame_diff.py --dataset data/e2e_dataset.npz
    python3 diag/11_frame_diff.py --dataset data/e2e_dataset.npz --epochs 120 --seconds 5

实测(云主机): 离线踝 pitch 预测 std 31.5（看着可以），闭环 0.58 s 跌倒。
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import (DATASET, MODEL_PATH, load_xy, nn_policy, require,  # noqa: E402
                     run_episode)

from humanoid_e2e_recovery.dataset import _img_to_vec, X_STD_FLOOR  # noqa: E402
from humanoid_e2e_recovery.e2e_nn import E2EMLP  # noqa: E402
from humanoid_e2e_recovery.sim import E2ESim  # noqa: E402


class DiffPolicy:
    """单帧 + 上一帧差分。只在 refresh 时更新 prev。"""

    def __init__(self, path):
        w = np.load(path)
        self.W1, self.b1 = w["W1"], w["b1"]
        self.W2, self.b2 = w["W2"], w["b2"]
        self.W3, self.b3 = w["W3"], w["b3"]
        self.fm, self.fs = w["F_mean"], w["F_std"]
        self.ym, self.ysc = w["Y_mean"], w["Y_scale"]
        self.prev = None
        self.last = np.zeros(15)

    def __call__(self, sim, refresh=False):
        if not refresh:
            return self.last
        v = _img_to_vec(sim.render_front(gray=True)).astype(np.float64)
        dv = np.zeros_like(v) if self.prev is None else v - self.prev
        self.prev = v
        f = np.concatenate([v, dv])
        f = (f - self.fm) / self.fs
        a1 = np.maximum(0, f @ self.W1 + self.b1)
        a2 = np.maximum(0, a1 @ self.W2 + self.b2)
        self.last = (a2 @ self.W3 + self.b3) * self.ysc + self.ym
        return self.last


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)

    p = argparse.ArgumentParser(description="帧间差分特征实验")
    p.add_argument("--dataset", default=DATASET)
    p.add_argument("--epochs", type=int, default=220)
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--diff-floor", type=float, default=1e-4, help="差分维独立兜底")
    p.add_argument("--out", default=os.path.join(root, "data", "diag_diff_policy.npz"))
    a = p.parse_args()
    require(a.dataset, "先 python3 main.py --collect 生成数据集")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)

    X, Y = load_xy(a.dataset)
    dX = np.vstack([np.zeros((1, X.shape[1])), np.diff(X, axis=0)])
    F = np.hstack([X, dX])
    print("特征维度 {} (单帧 {} + 差分 {})".format(F.shape[1], X.shape[1], dX.shape[1]))
    print("原始逐维 std 均值 {:.5f} | 差分逐维 std 均值 {:.5f}".format(
        X.std(0).mean(), dX.std(0).mean()))

    Fm = F.mean(0)
    Fs = np.concatenate([np.maximum(X.std(0), X_STD_FLOOR),
                         np.maximum(dX.std(0), a.diff_floor)])
    Ym = Y.mean(0)
    Ys = np.where(Y.std(0) > 1e-6, Y.std(0), 1.0)

    Fn = (F - Fm) / Fs
    Yn = (Y - Ym) / Ys
    net = E2EMLP(F.shape[1], 160, 72, 15, seed=42)
    r = np.random.default_rng(42)
    t0 = time.time()
    for ep in range(a.epochs):
        perm = r.permutation(len(Fn))
        cur_lr = 0.004 if ep < int(a.epochs * 0.6) else 0.002
        for i in range(0, len(Fn) - 64 + 1, 64):
            b = perm[i:i + 64]
            net.train_step(Fn[b], Yn[b], lr=cur_lr)
    print("[train] {:.0f}s".format(time.time() - t0))

    tr = int(len(Y) * 0.8)
    pred = net.predict(Fn[tr:]) * Ys + Ym
    print("[离线] 踝 pitch 预测 std {:.1f} / 目标 {:.1f}".format(
        pred[:, 3].std(), Y[tr:, 3].std()))

    np.savez(a.out,
             W1=net.W1, b1=net.b1, W2=net.W2, b2=net.b2, W3=net.W3, b3=net.b3,
             F_mean=Fm, F_std=Fs, Y_mean=Ym, Y_scale=Ys)
    print("saved {}".format(a.out))

    pol = DiffPolicy(a.out)

    def policy(sim, refresh):
        return pol(sim, refresh)

    sim = E2ESim(MODEL_PATH)
    sim.reset()
    fell, tilt = run_episode(sim, policy, a.seconds)
    print("\n[闭环] 单帧+差分: {}，最大倾角 {:.1f}°".format(
        "跌倒于 {:.2f}s".format(fell) if fell else "站稳 {}s".format(a.seconds),
        np.degrees(tilt)))
    sim.close()
    print("\n判读：若输出 nan 或秒倒，说明差分本身没有可用信号，别在这条路上继续加约束。")


if __name__ == "__main__":
    main()
