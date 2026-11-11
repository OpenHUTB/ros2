# -*- coding: utf-8 -*-
"""生成本模块的图（需要 matplotlib）。

    python3 tools/make_figures.py --out ../../docs/img/air/uav_rotor_failure_env

输出 ``margins.png``：三种机型在"正常 / 单桨失效·可自旋 / 单桨失效·零偏航"
三种工况下的推力余量柱状图。
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

from uav_rotor_failure_env.core import (hexa, max_hover,  # noqa: E402
                                        max_hover_zero_yaw, octo, quad_x)


def _setup_cjk():
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                 "WenQuanYi Zen Hei", "PingFang SC"):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False


_setup_cjk()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=".")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    configs = [quad_x(), hexa(), octo()]
    labels = [c.name for c in configs]
    normal = [100 * max_hover(c)["margin"] for c in configs]
    free = [100 * max_hover(c, {0})["margin"] for c in configs]
    zero = [100 * max_hover_zero_yaw(c, {0})["margin"] for c in configs]

    x = np.arange(len(configs))
    w = 0.26
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    ax.bar(x - w, normal, width=w, label="正常", color="tab:green")
    ax.bar(x, free, width=w, label="单桨失效（可自旋）", color="tab:orange")
    ax.bar(x + w, zero, width=w, label="单桨失效（零偏航）", color="tab:red")
    for i, v in enumerate(zero):
        ax.text(x[i] + w, v + 1.5, "%.1f%%" % v, ha="center", fontsize=8)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("零姿态力矩下的推力余量 (%)")
    ax.set_ylim(0, 115)
    ax.set_title("单桨失效后的悬停余量：四旋翼不转就吊不动")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3, axis="y")
    path = os.path.join(args.out, "margins.png")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    print(path)


if __name__ == "__main__":
    main()
