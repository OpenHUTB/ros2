# -*- coding: utf-8 -*-
"""力分配仿真入口：对一段时变期望力矩做最小范数分配，画出各肌激活度与力矩跟踪。"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from force_alloc import Muscle, allocate, torque_vector  # noqa: E402
from simple_plot import plot_series  # noqa: E402

DT = 0.01
T_END = 4.0

MUSCLES = [
    Muscle("F1", 0.03, 1000, +1),
    Muscle("F2", 0.03, 1000, +1),
    Muscle("E1", -0.03, 1000, -1),
    Muscle("E2", -0.03, 1000, -1),
]

COLORS = [(200, 40, 40), (240, 140, 40), (30, 90, 200), (40, 160, 200)]


def desired_torque(t):
    return 30.0 + 20.0 * __import__("math").sin(1.5 * t)


def run():
    ts, acts_hist, tau_des_hist, tau_ach_hist = [], [], [], []
    for i in range(int(round(T_END / DT)) + 1):
        t = i * DT
        td = desired_torque(t)
        a, ta = allocate(td, MUSCLES)
        ts.append(t)
        acts_hist.append(a)
        tau_des_hist.append(td)
        tau_ach_hist.append(ta)
    return ts, acts_hist, tau_des_hist, tau_ach_hist


def main(out_dir="."):
    os.makedirs(out_dir, exist_ok=True)
    ts, acts_hist, tau_des_hist, tau_ach_hist = run()
    n = len(MUSCLES)
    act_series = [
        (MUSCLES[k].name, ts, [row[k] for row in acts_hist], COLORS[k % len(COLORS)])
        for k in range(n)
    ]
    path1 = os.path.join(out_dir, "force_alloc.png")
    plot_series(path1, act_series, title="Muscle activations (min-norm)",
                width=900, height=560, margin=(60, 30, 30, 150),
                ylabel="activation")

    tau_series = [
        ("tau_des", ts, tau_des_hist, (120, 120, 120)),
        ("tau_ach", ts, tau_ach_hist, (200, 40, 40)),
    ]
    path2 = os.path.join(out_dir, "force_tracking.png")
    plot_series(path2, tau_series, title="Torque tracking", width=900, height=560,
                margin=(60, 30, 30, 150), ylabel="torque (Nm)")

    err = [abs(d - a) for d, a in zip(tau_des_hist, tau_ach_hist)]
    print("力矩跟踪最大误差: %.4f Nm" % max(err))
    print("已生成: %s  %s" % (path1, path2))
    return path1, path2


if __name__ == "__main__":
    main(os.path.dirname(__file__))
