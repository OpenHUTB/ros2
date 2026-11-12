#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""09 混合控制扫描：给网络补多少专家力矩（= 补多少速度阻尼）才站得住。

这是判断"缺速度信息"的决定性实验：
  beta = 专家力矩占比。beta=0 是纯网络，beta=1 是纯专家。
如果补到 0.3 就能从 3.5 s 变成站满 10 s，说明缺口就是阻尼，不是别的。

用法:
    python3 diag/09_blend_sweep.py
    python3 diag/09_blend_sweep.py --betas 0 0.1 0.3 0.5 --seconds 10

实测(云主机, 当前权重):
  beta=0.0 -> 3.53 s 跌倒
  beta=0.3 -> 站稳 10 s，末段倾角 3.6°
  beta=0.5 -> 站稳 10 s，末段倾角 3.8°
  beta=0.7 / 1.0 因单次 290 s 超时未跑完（纯专家很稳，重跑即可）
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import CTRL_DT, DEFAULT_STAND_TARGET, MODEL_PATH, WEIGHTS  # noqa: E402

from humanoid_e2e_recovery.sim import E2ESim  # noqa: E402
from humanoid_e2e_recovery.predictor import E2EPredictor  # noqa: E402
from humanoid_e2e_recovery.expert import expert_action  # noqa: E402


def main():
    p = argparse.ArgumentParser(description="网络/专家力矩混合扫描")
    p.add_argument("--weights", default=WEIGHTS)
    p.add_argument("--betas", nargs="+", type=float, default=[0.0, 0.3, 0.5])
    p.add_argument("--seconds", type=float, default=10.0)
    a = p.parse_args()

    pdt = E2EPredictor(a.weights)
    sim = E2ESim(MODEL_PATH)
    steps = int(a.seconds / CTRL_DT)

    for beta in a.betas:
        sim.reset()
        u = np.zeros(15)
        fell = None
        for i in range(steps):
            if i % 2 == 0:
                net = pdt.predict_from_image(sim.render_front(gray=True))
                exp = expert_action(sim)
                u = beta * exp + (1 - beta) * net
            sim.step(target=DEFAULT_STAND_TARGET, torque_extra=u)
            if sim.fallen():
                fell = i * CTRL_DT
                break
        print("beta={:.1f}: {}，最终倾角 {:.1f}°".format(
            beta, "跌倒于 {:.2f}s".format(fell) if fell else "站稳 {}s".format(a.seconds),
            np.degrees(sim.torso_tilt())))

    sim.close()
    print("\n判读：若 beta 很小就能站满，缺口是阻尼（速度信息），不是训练或样本量问题。")


if __name__ == "__main__":
    main()
