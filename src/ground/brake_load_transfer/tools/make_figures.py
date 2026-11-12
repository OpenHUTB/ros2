# -*- coding: utf-8 -*-
"""生成本模块的两张图（需要 matplotlib，仅作图用）。

    python3 tools/make_figures.py --out ../../docs/img/ground/brake_load_transfer

输出：
* ``distance_vs_payload.png``：三种 bias 下制动距离随载重位置的变化
* ``yaw_vs_offset.png``：左右偏置导致的横摆角与跑偏量
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402


def _setup_cjk_font():
    """优先用系统里的中文字体，找不到就退回默认（此时标题会是方框，但不会报错）。"""
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                 "WenQuanYi Zen Hei", "PingFang SC"):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False


_setup_cjk_font()

from brake_load_transfer.core import (VehicleSpec, simulate_braking,  # noqa: E402
                                      simulate_braking_ideal, simulate_lateral)


def fig_distance(out_dir):
    positions = [0.3, 0.8, 1.35, 1.9, 2.4]
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    for beta, style in ((0.55, "--"), (0.62, "-"), (0.70, ":")):
        dists = []
        for x in positions:
            s = VehicleSpec(payload=300.0, payload_x=x, brake_bias=beta)
            dists.append(simulate_braking(s)["distance_m"])
        ax.plot(positions, dists, style, marker="o", label="β = %.2f" % beta)

    ideal = simulate_braking_ideal(VehicleSpec(payload=300.0))["distance_m"]
    ax.axhline(ideal, color="green", linewidth=1.2, linestyle="-.",
               label="理想 EBD %.1f m（按动态轴荷分配）" % ideal)
    ax.set_xlabel("载重质心距前轴 (m)")
    ax.set_ylabel("100 km/h 制动距离 (m)")
    ax.set_title("载重位置 vs 制动距离（载重 300 kg，μ=0.9）")
    ax.grid(alpha=0.3)
    ax.legend()
    path = os.path.join(out_dir, "distance_vs_payload.png")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def fig_lateral(out_dir):
    offsets = [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3]
    yaw, drift = [], []
    for dy in offsets:
        r = simulate_lateral(VehicleSpec(payload=300.0, payload_y=dy))
        yaw.append(r["yaw_deg"])
        drift.append(r["drift_m"])

    fig, ax1 = plt.subplots(figsize=(7.2, 4.2))
    ax1.plot(offsets, yaw, "o-", color="tab:red", label="横摆角")
    ax1.set_xlabel("载重横向偏移 (m)")
    ax1.set_ylabel("制动结束时横摆角 (°)", color="tab:red")
    ax1.tick_params(axis="y", labelcolor="tab:red")
    ax1.grid(alpha=0.3)

    ax2 = ax1.twinx()
    ax2.plot(offsets, drift, "s--", color="tab:blue", label="横向跑偏")
    ax2.set_ylabel("横向跑偏 (m)", color="tab:blue")
    ax2.tick_params(axis="y", labelcolor="tab:blue")

    ax1.set_title("左右偏置 vs 制动跑偏（载重 300 kg，100 km/h 刹停）")
    path = os.path.join(out_dir, "yaw_vs_offset.png")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=".", help="输出目录")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    for p in (fig_distance(args.out), fig_lateral(args.out)):
        print(p)


if __name__ == "__main__":
    main()
