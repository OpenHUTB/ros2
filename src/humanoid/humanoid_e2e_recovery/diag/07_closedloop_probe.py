#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""07 闭环探针：网络输出到底跟专家差多少，以及"常数力矩"能不能站住。

三个子实验：
  1. 专家自身输出幅度分布（当前 expert.py 实际给多少 N·m）
  2. 同一状态下 网络 vs 专家（判断是量级错、还是方向错）
  3. 常数 +55 / 0 / -55 N·m 踝 pitch 的闭环结果
     —— 这一步是识破"旧权重站稳 10 s 其实是常数巧合"的关键

用法:
    python3 diag/07_closedloop_probe.py
    python3 diag/07_closedloop_probe.py --weights models_trained/e2e_policy.npz

实测(云主机, 旧权重): 专家踝 pitch 均值 66.17 / std 35.49；
                     网络均值 2.67 / std 0.00（恒定），幅值比 0.04。
常数 +55 -> 站 10 s（5.5°）；常数 0 -> 1.31 s 倒；常数 -55 -> 1.19 s 倒。
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import (DEFAULT_STAND_TARGET, MODEL_PATH, WEIGHTS,  # noqa: E402
                     run_episode)

from humanoid_e2e_recovery.sim import E2ESim  # noqa: E402
from humanoid_e2e_recovery.expert import expert_action  # noqa: E402
from humanoid_e2e_recovery.predictor import E2EPredictor  # noqa: E402


def main():
    p = argparse.ArgumentParser(description="闭环探针：网络 vs 专家 + 常数力矩对照")
    p.add_argument("--weights", default=WEIGHTS)
    p.add_argument("--seconds", type=float, default=10.0)
    a = p.parse_args()

    sim = E2ESim(MODEL_PATH)

    # 1. 专家输出幅度
    sim.reset()
    acts = []
    for _ in range(3000):
        u = expert_action(sim)
        acts.append(u.copy())
        sim.step(target=DEFAULT_STAND_TARGET, torque_extra=u)
    A = np.stack(acts)
    print("=== 专家输出（3000 步）===")
    print("  踝 pitch(dim3) 均值 {:.2f}  std {:.2f}  min {:.2f}  max {:.2f}".format(
        A[:, 3].mean(), A[:, 3].std(), A[:, 3].min(), A[:, 3].max()))
    print("  全部维 abs 均值 {:.3f}  abs 最大 {:.2f}".format(
        np.abs(A).mean(), np.abs(A).max()))
    print("  逐维 std: {}".format(np.round(A.std(0), 2)))

    # 2. 同一状态下的网络输出
    pdt = E2EPredictor(a.weights)
    sim.reset()
    pairs = []
    for _ in range(60):
        img = sim.render_front(gray=True)
        pairs.append((expert_action(sim).copy(), pdt.predict_from_image(img)))
        sim.step(target=DEFAULT_STAND_TARGET, torque_extra=pairs[-1][0])
    UE = np.stack([x[0] for x in pairs])
    UN = np.stack([x[1] for x in pairs])
    print("\n=== 同状态下 网络 vs 专家 ===")
    print("  专家 踝 pitch: 均值 {:.2f}  std {:.2f}  最大 {:.2f}".format(
        UE[:, 3].mean(), UE[:, 3].std(), np.abs(UE[:, 3]).max()))
    print("  网络 踝 pitch: 均值 {:.2f}  std {:.2f}  最大 {:.2f}".format(
        UN[:, 3].mean(), UN[:, 3].std(), np.abs(UN[:, 3]).max()))
    print("  幅值比 |网络| / |专家| = {:.4f}".format(
        np.abs(UN).mean() / max(np.abs(UE).mean(), 1e-9)))
    print("  逐维网络输出均值: {}".format(np.round(UN.mean(0), 3)))
    if UN[:, 3].std() < 1e-6:
        print("  -> 网络输出恒定，先查 X_mean/X_std 与当前渲染是否同源")

    # 3. 常数力矩对照
    def const_policy(v):
        def f(sim, refresh):
            u = np.zeros(15)
            u[3] = v
            u[8] = v
            return u
        return f

    for v in (55.0, -55.0, 0.0):
        sim.reset()
        fell, tilt = run_episode(sim, const_policy(v), a.seconds, enable_render=False)
        print("\n常数 {:+.0f} N·m 踝 pitch: {} (最大倾角 {:.1f}°)".format(
            v, "跌倒于 {:.2f}s".format(fell) if fell else "站稳 {}s".format(a.seconds),
            np.degrees(tilt)))

    sim.close()


if __name__ == "__main__":
    main()
