#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""14 全链路：采集 -> 训练 -> 离线响应 -> 闭环，一条命令复现最终结果。

这就是入库权重 e2e_policy.npz 的生成过程。跑完应对上：
  训练集 4272 条，X 逐维 std 均值 0.0125，Y 踝 pitch 均值 58.0 / std 33.8
  稳态首帧 网络踝 pitch 9.04 vs 专家 0.00
  空站 10 s: 跌倒于 3.53 s，最大倾角 63.6°

用法:
    python3 diag/14_pipeline_check.py                       # 写 data/ 下临时产物
    python3 diag/14_pipeline_check.py --episodes 2          # 快速小样本冒烟
    python3 diag/14_pipeline_check.py --out <npz>           # 覆盖入库权重前请先备份

注意：8 回合采集约 166 s，训练约 1~2 min，闭环 10 s 约 2 min（渲染是瓶颈）。
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import MODEL_PATH, WEIGHTS, nn_policy, run_episode  # noqa: E402

from humanoid_e2e_recovery.dataset import collect  # noqa: E402
from humanoid_e2e_recovery.train import train  # noqa: E402
from humanoid_e2e_recovery.sim import E2ESim  # noqa: E402
from humanoid_e2e_recovery.predictor import E2EPredictor  # noqa: E402
from humanoid_e2e_recovery.expert import expert_action  # noqa: E402


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)

    p = argparse.ArgumentParser(description="采集+训练+闭环 全链路")
    p.add_argument("--episodes", type=int, default=8)
    p.add_argument("--force", nargs=2, type=float, default=(40.0, 100.0))
    p.add_argument("--warmup", type=int, default=1200)
    p.add_argument("--horizon", type=int, default=400)
    p.add_argument("--epochs", type=int, default=200)
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--ds", default=os.path.join(root, "data", "diag_pipeline_ds.npz"))
    p.add_argument("--out", default=os.path.join(root, "data", "diag_pipeline_policy.npz"))
    a = p.parse_args()
    os.makedirs(os.path.dirname(a.ds), exist_ok=True)

    t = time.time()
    collect(MODEL_PATH, a.ds, n_episodes=a.episodes, force_range=tuple(a.force),
            horizon=a.horizon, warmup=a.warmup, record_every=3, verbose=True)
    print("[collect] {:.0f}s".format(time.time() - t))

    d = np.load(a.ds)
    X, Y = d["X"], d["Y"]
    print("样本 {} | X std均值 {:.5f} | Y踝pitch 均值 {:.1f} std {:.1f} 范围 {:.1f}~{:.1f}".format(
        len(Y), X.std(0).mean(), Y[:, 3].mean(), Y[:, 3].std(), Y[:, 3].min(), Y[:, 3].max()))

    train(a.ds, a.out, epochs=a.epochs, verbose=False)
    pdt = E2EPredictor(a.out)
    outs = pdt.predict_vecs(X[:2000].astype(np.float64))
    print("[离线] 踝pitch 预测 std {:.2f} / 目标 {:.2f} | 预测均值 {:.1f}".format(
        outs[:, 3].std(), Y[:2000, 3].std(), outs[:, 3].mean()))

    sim = E2ESim(MODEL_PATH)
    sim.reset()
    u0 = pdt.predict_from_image(sim.render_front(gray=True))
    print("\n[闭环] 稳态首帧: 网络踝pitch {:.2f} | 专家 {:.2f}".format(
        u0[3], expert_action(sim)[3]))
    fell, tilt = run_episode(sim, nn_policy(pdt), a.seconds)
    print("  空站 {}s: {}，最大倾角 {:.1f}°".format(
        a.seconds, "跌倒于 {:.2f}s".format(fell) if fell else "站稳",
        np.degrees(tilt)))
    sim.close()

    print("\n入库权重: {}".format(WEIGHTS))
    print("本次产物: {} / {}".format(a.ds, a.out))


if __name__ == "__main__":
    main()
