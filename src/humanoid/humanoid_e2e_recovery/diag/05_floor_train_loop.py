#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""05 离线达标但闭环仍倒：把 floor 提到 0.02 重训，再跑闭环。

回答的问题：04 已经把验证 R² 修到 0.98，为什么闭环还是 0.79 s 就倒？
这一步的作用就是制造"离线指标好看、闭环却失败"的对照点，
把断点逼到"训练集不含稳态样本"那一条。

用法:
    python3 diag/05_floor_train_loop.py --dataset data/diag_probe.npz
    python3 diag/05_floor_train_loop.py --dataset data/diag_probe.npz --seconds 5

实测(云主机): 离线踝 pitch 预测 std 85.07 / 目标 85.46（很好），
闭环仍 0.79 s 跌倒——因为该数据集 warmup 段没有记录，
推理起点恰是"稳态、专家力矩 0"，属于分布外。
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import (DATASET, DEFAULT_STAND_TARGET, MODEL_PATH,  # noqa: E402
                     load_xy, nn_policy, require, run_episode)

from humanoid_e2e_recovery.e2e_nn import E2EMLP  # noqa: E402
from humanoid_e2e_recovery.sim import E2ESim  # noqa: E402
from humanoid_e2e_recovery.predictor import E2EPredictor  # noqa: E402


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)

    p = argparse.ArgumentParser(description="floor=0.02 重训 + 闭环验证")
    p.add_argument("--dataset", default=DATASET)
    p.add_argument("--floor", type=float, default=0.02)
    p.add_argument("--epochs", type=int, default=220)
    p.add_argument("--seconds", type=float, default=10.0, help="闭环时长")
    p.add_argument("--out", default=os.path.join(root, "data", "diag_floor002_policy.npz"))
    a = p.parse_args()
    require(a.dataset, "先用 diag/02_probe_strong_push.py 生成探针数据")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)

    X, Y = load_xy(a.dataset)
    Xm = X.mean(0)
    Xs = np.maximum(X.std(0), a.floor)
    Ym = Y.mean(0)
    Ysc = np.where(Y.std(0) > 1e-6, Y.std(0), 1.0)
    Xn = (X - Xm) / Xs
    Yn = (Y - Ym) / Ysc

    net = E2EMLP(256, 128, 64, 15, seed=42)
    r = np.random.default_rng(42)
    for ep in range(a.epochs):
        perm = r.permutation(len(Xn))
        cur_lr = 0.005 if ep < int(a.epochs * 0.6) else 0.0025
        for i in range(0, len(Xn) - 64 + 1, 64):
            b = perm[i:i + 64]
            net.train_step(Xn[b], Yn[b], lr=cur_lr)

    net.save(a.out, extra=dict(X_mean=Xm, X_std=Xs, Y_mean=Ym, Y_scale=Ysc))
    print("saved {}".format(a.out))

    pdt = E2EPredictor(a.out)
    outs = pdt.predict_vecs(X[:400])
    print("离线: 踝 pitch 预测 std {:.1f} (目标 {:.1f})".format(
        outs[:, 3].std(), Y[:400, 3].std()))

    sim = E2ESim(MODEL_PATH)
    sim.reset()
    fell, tilt = run_episode(sim, nn_policy(pdt), a.seconds)
    print("闭环 {}s: {}，最大倾角 {:.1f}°".format(
        a.seconds, "跌倒于 {:.2f}s".format(fell) if fell else "站稳",
        np.degrees(tilt)))
    sim.close()

    if fell:
        print("\n结论：离线达标、闭环仍倒 -> 问题不在拟合能力，在训练集覆盖。"
              "\n      检查 dataset.py 的 record_warmup 是否为 True。")
    else:
        print("\n结论：该数据集已覆盖稳态，闭环可用。")


if __name__ == "__main__":
    main()
