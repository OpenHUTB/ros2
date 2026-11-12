#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""02 强扰动探针数据：采集一批"状态真的在动"的数据，供后面的信息量检验用。

回答的问题：默认 40~100 N 推力下专家把状态锁在零点附近，图像几乎不变，
可学习信号微乎其微；把推力加大能否拿到有动态的数据。

用法:
    python3 diag/02_probe_strong_push.py                       # 默认 (60,150) N、3 回合
    python3 diag/02_probe_strong_push.py --force 40 100        # 换成默认工况对比
    python3 diag/02_probe_strong_push.py --episodes 8          # 回合数

实测(云主机):
  (40,90)  N -> X 逐维 std 0.00106，Y 踝 pitch std 0.23（信号几乎为零）
  (60,150) N -> X 逐维 std 0.01097，Y 踝 pitch std 84.88（范围 -206~621）
  (100,250) N -> 专家直接发散，Y 范围 -3541~1092，不可用
所以探针数据取 (60,150) N，最终训练集才降回 (40,100) N。
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import MODEL_PATH, ROOT  # noqa: E402

from humanoid_e2e_recovery.dataset import collect  # noqa: E402


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_out = os.path.join(ROOT, "data", "diag_probe.npz")

    p = argparse.ArgumentParser(description="采集强扰动探针数据")
    p.add_argument("--force", nargs=2, type=float, default=(60.0, 150.0),
                   metavar=("MIN", "MAX"), help="推力幅值区间，默认 60 150")
    p.add_argument("--episodes", type=int, default=3)
    p.add_argument("--warmup", type=int, default=1200)
    p.add_argument("--horizon", type=int, default=1200)
    p.add_argument("--out", default=default_out, help="输出 npz")
    a = p.parse_args()

    t = time.time()
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    collect(MODEL_PATH, a.out, n_episodes=a.episodes,
            force_range=tuple(a.force), warmup=a.warmup, horizon=a.horizon,
            verbose=True)

    d = np.load(a.out)
    X, Y = d["X"], d["Y"]
    print("\n推力 {} N: {} 条, 用时 {:.0f}s".format(tuple(a.force), len(Y), time.time() - t))
    print("  X 逐维 std 均值 {:.5f} 最大 {:.5f} | X 均值 {:.3f}".format(
        X.std(0).mean(), X.std(0).max(), X.mean()))
    print("  Y 踝 pitch: 均值 {:.1f} std {:.2f} 范围 {:.1f}~{:.1f}".format(
        Y[:, 3].mean(), Y[:, 3].std(), Y[:, 3].min(), Y[:, 3].max()))
    print("  Y 非零维: {}".format(np.where(Y.std(0) > 1e-6)[0].tolist()))
    print("\n跑 03/04/05 时用 --dataset {}".format(a.out))


if __name__ == "__main__":
    main()
