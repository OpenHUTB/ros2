#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""10 控制刷新间隔：把渲染频率从 4 ms 提到 2 ms，闭环会不会变好。

回答的问题：3.5 s 上限到底是不是"图像延迟太大"造成的。
若 2 ms 与 4 ms 的结果几乎一样，就排除了延迟假设。

用法:
    python3 diag/10_render_delay.py
    python3 diag/10_render_delay.py --every 1 2 3 --seconds 6

实测(云主机): render_every=1 (2 ms) -> 3.58 s；render_every=2 (4 ms) -> 3.53 s。
两者基本无差，延迟不是主因；渲染一次约 65.6 ms，更进一步提高刷新率也不现实。
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import MODEL_PATH, WEIGHTS, nn_policy, run_episode  # noqa: E402

from humanoid_e2e_recovery.sim import E2ESim  # noqa: E402
from humanoid_e2e_recovery.predictor import E2EPredictor  # noqa: E402


def main():
    p = argparse.ArgumentParser(description="控制刷新间隔对照")
    p.add_argument("--weights", default=WEIGHTS)
    p.add_argument("--every", nargs="+", type=int, default=[1, 2], help="每隔多少仿真步渲染一次")
    p.add_argument("--seconds", type=float, default=6.0)
    a = p.parse_args()

    pdt = E2EPredictor(a.weights)
    sim = E2ESim(MODEL_PATH)
    for every in a.every:
        sim.reset()
        fell, tilt = run_episode(sim, nn_policy(pdt), a.seconds, render_every=every)
        print("刷新间隔 {} 步({} ms): {}，最大倾角 {:.1f}°".format(
            every, every * 2, "跌倒于 {:.2f}s".format(fell) if fell else "站稳",
            np.degrees(tilt)))
    sim.close()


if __name__ == "__main__":
    main()
