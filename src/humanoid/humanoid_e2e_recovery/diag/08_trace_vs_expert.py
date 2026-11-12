#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""08 逐帧对照：闭环时网络力矩是不是真的在跟专家。

回答的问题：如果网络输出和专家同一时刻基本重合，说明网络在跟踪专家；
那闭环失败就只能是因为缺了"专家有、单帧图像没有"的东西（速度阻尼）。

用法:
    python3 diag/08_trace_vs_expert.py
    python3 diag/08_trace_vs_expert.py --steps 600      # 跑更久看漂移
    python3 diag/08_trace_vs_expert.py --weights <npz>

实测(云主机, 当前权重, 180 步):
  网络踝 pitch 均值 61.0 std 20.15 范围 8.8~107.7
  专家踝 pitch 均值 56.6 std 22.80 范围 -1.7~128.4
  20 ms 处 34.7 vs 29.9；80 ms 处 96.1 vs 95.2 —— 基本重合
  倾角在 340 ms 内只从 0 涨到 1.62°，是缓慢漂移而非快速崩溃
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import DEFAULT_STAND_TARGET, MODEL_PATH, WEIGHTS  # noqa: E402

from humanoid_e2e_recovery.sim import E2ESim  # noqa: E402
from humanoid_e2e_recovery.predictor import E2EPredictor  # noqa: E402
from humanoid_e2e_recovery.expert import expert_action  # noqa: E402


def main():
    p = argparse.ArgumentParser(description="网络与专家逐帧对照")
    p.add_argument("--weights", default=WEIGHTS)
    p.add_argument("--steps", type=int, default=180, help="仿真步数（每步 2 ms）")
    p.add_argument("--print-every", type=int, default=10)
    a = p.parse_args()

    pdt = E2EPredictor(a.weights)
    sim = E2ESim(MODEL_PATH)
    sim.reset()

    u = np.zeros(15)
    rec = []
    print("  t(ms)  tilt(°)  pelvis_x   net踝pitch  expert踝pitch  net输出逐维std")
    for i in range(a.steps):
        if i % 2 == 0:
            u = pdt.predict_from_image(sim.render_front(gray=True))
        e = expert_action(sim)
        if i % a.print_every == 0:
            print("  {:>5.0f}  {:>7.2f}  {:>9.5f}  {:>10.2f}  {:>12.2f}  {:>14.3f}".format(
                i * 2, np.degrees(sim.torso_tilt()), sim.pelvis_x, u[3], e[3], u.std()))
        rec.append((i * 2, np.degrees(sim.torso_tilt()), sim.pelvis_x, u.copy(), e.copy()))
        sim.step(target=DEFAULT_STAND_TARGET, torque_extra=u)
        if sim.fallen():
            print("  -> 跌倒于 {:.2f}s".format(i * 0.002))
            break
    sim.close()

    tilt = np.array([r[1] for r in rec])
    n3 = np.array([r[3][3] for r in rec])
    e3 = np.array([r[4][3] for r in rec])
    print("\n=== 响应性 ===")
    print("  网络踝 pitch: 均值 {:.1f} std {:.2f} 范围 {:.1f}~{:.1f}".format(
        n3.mean(), n3.std(), n3.min(), n3.max()))
    print("  专家踝 pitch: 均值 {:.1f} std {:.2f} 范围 {:.1f}~{:.1f}".format(
        e3.mean(), e3.std(), e3.min(), e3.max()))
    print("  倾角范围 {:.2f}~{:.2f}°".format(tilt.min(), tilt.max()))
    if n3.std() > 1e-6:
        print("  corr(网络力矩, 倾角) = {:+.3f}".format(np.corrcoef(n3, tilt)[0, 1]))
    print("  corr(专家力矩, 倾角) = {:+.3f}".format(np.corrcoef(e3, tilt)[0, 1]))


if __name__ == "__main__":
    main()
