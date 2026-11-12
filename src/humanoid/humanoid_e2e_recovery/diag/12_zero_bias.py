#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""12 零偏校正（失败路线，保留作证据）：把稳态首帧的输出整体减掉。

思路：网络在稳态（专家力矩为 0）却输出 9.04 N·m，说明有个常数偏置；
减掉它应该立刻变好。
结果：1.38 s，比不校正的 3.53 s 更差 —— 偏置其实是有符号的"姿态修正"，
      当成纯误差减掉等于把控制方向也减掉了。

用法:
    python3 diag/12_zero_bias.py
    python3 diag/12_zero_bias.py --weights <npz> --seconds 10

实测(云主机, 当前权重): 稳态参考帧网络输出 9.04 N·m（踝 pitch）；
无校正 3.53 s 跌倒，零偏校正 1.38 s 跌倒。
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import MODEL_PATH, WEIGHTS, run_episode  # noqa: E402

from humanoid_e2e_recovery.sim import E2ESim  # noqa: E402
from humanoid_e2e_recovery.predictor import E2EPredictor  # noqa: E402


def make_policy(predictor, bias, cache=None):
    c = cache if cache is not None else {"u": np.zeros(15)}

    def policy(sim, refresh):
        if refresh:
            c["u"] = predictor.predict_from_image(sim.render_front(gray=True)) - bias
        return c["u"]

    return policy


def main():
    p = argparse.ArgumentParser(description="零偏校正对照")
    p.add_argument("--weights", default=WEIGHTS)
    p.add_argument("--seconds", type=float, default=10.0)
    a = p.parse_args()

    pdt = E2EPredictor(a.weights)

    # 用闭环起点（reset 后第一帧）当零偏参考：它代表"完全站稳、专家输出 0"的状态
    sim = E2ESim(MODEL_PATH)
    sim.reset()
    bias0 = pdt.predict_from_image(sim.render_front(gray=True))
    print("稳态参考帧的原始网络输出: {}".format(np.round(bias0, 2)))
    print("参考帧踝 pitch 偏置 = {:.2f} N·m".format(bias0[3]))

    for name, bias in (("无校正", np.zeros(15)), ("零偏校正", bias0)):
        sim.reset()
        fell, tilt = run_episode(sim, make_policy(pdt, bias), a.seconds)
        print("闭环[{}]: {}，最大倾角 {:.1f}°".format(
            name, "跌倒于 {:.2f}s".format(fell) if fell else "站稳 {}s".format(a.seconds),
            np.degrees(tilt)))
    sim.close()


if __name__ == "__main__":
    main()
