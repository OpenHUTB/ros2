#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""06 快速训练 + 闭环：用现成训练入口对探针数据重训，量"离线达标"与"闭环可用"的差。

用法:
    python3 diag/06_quick_train_loop.py --dataset data/diag_probe.npz
    python3 diag/06_quick_train_loop.py --dataset data/diag_probe.npz --epochs 120

实测(云主机, 无稳态样本的探针数据): 离线输出响应 0.747（目标 5.2），
闭环 0.73 s 跌倒 —— 网络学到的量级远小于专家。
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import DATASET, MODEL_PATH, load_xy, nn_policy, require, run_episode  # noqa: E402

from humanoid_e2e_recovery.train import train  # noqa: E402
from humanoid_e2e_recovery.sim import E2ESim  # noqa: E402
from humanoid_e2e_recovery.predictor import E2EPredictor  # noqa: E402


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)

    p = argparse.ArgumentParser(description="快速训练并在闭环上验证")
    p.add_argument("--dataset", default=DATASET)
    p.add_argument("--epochs", type=int, default=200)
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--out", default=os.path.join(root, "data", "diag_quick_policy.npz"))
    a = p.parse_args()
    require(a.dataset, "先用 diag/02_probe_strong_push.py 生成探针数据")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)

    train(a.dataset, a.out, epochs=a.epochs, verbose=True)

    X, Y = load_xy(a.dataset)
    pdt = E2EPredictor(a.out)
    outs = pdt.predict_vecs(X[:500])
    print("\n=== 离线响应 ===")
    print("  输出逐维 std 最大 {:.3f} N·m | 踝 pitch 均值 {:.1f} std {:.1f}".format(
        outs.std(0).max(), outs[:, 3].mean(), outs[:, 3].std()))
    print("  实际 Y 踝 pitch 均值 {:.1f} std {:.1f}".format(
        Y[:500, 3].mean(), Y[:500, 3].std()))

    sim = E2ESim(MODEL_PATH)
    sim.reset()
    fell, tilt = run_episode(sim, nn_policy(pdt), a.seconds)
    print("\n=== 闭环 ===")
    print("  {}，最大倾角 {:.1f}°".format(
        "跌倒于 {:.2f}s".format(fell) if fell else "站稳 {}s".format(a.seconds),
        np.degrees(tilt)))
    sim.close()


if __name__ == "__main__":
    main()
