# -*- coding: utf-8 -*-
"""滑模控制仿真入口：跑 SMC 与 PD 对照，并把曲线写成 PNG。"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from smc_model import simulate_smc, simulate_pd  # noqa: E402
from simple_plot import plot_series  # noqa: E402


def build_series():
    ts, xs_smc, rs = simulate_smc()
    _, xs_pd, _ = simulate_pd()
    return [
        ("ref", ts, rs, (150, 150, 150)),
        ("SMC", ts, xs_smc, (200, 40, 40)),
        ("PD", ts, xs_pd, (30, 90, 200)),
    ]


def main(out_dir="."):
    os.makedirs(out_dir, exist_ok=True)
    series = build_series()
    path = os.path.join(out_dir, "smc_response.png")
    plot_series(
        path,
        series,
        title="SMC vs PD under disturbance",
        width=900,
        height=560,
        margin=(60, 30, 30, 150),
        ylabel="position / reference",
    )
    ts, xs_smc, rs = simulate_smc()
    _, xs_pd, _ = simulate_pd()
    mae_smc = sum(abs(a - b) for a, b in zip(xs_smc, rs)) / len(xs_smc)
    mae_pd = sum(abs(a - b) for a, b in zip(xs_pd, rs)) / len(xs_pd)
    print("SMC 全程平均误差 : %.4f" % mae_smc)
    print("PD  全程平均误差 : %.4f" % mae_pd)
    print("已生成: %s" % path)
    return path


if __name__ == "__main__":
    main(os.path.dirname(__file__))
