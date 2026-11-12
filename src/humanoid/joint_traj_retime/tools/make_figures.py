# -*- coding: utf-8 -*-
"""生成本模块的两张图（需要 matplotlib）。

    python3 tools/make_figures.py --out ../../docs/img/humanoid/joint_traj_retime

输出：
* ``traj_before_after.png``：两个关节的位置曲线，重定时前后叠在一张时间轴上
* ``utilization.png``：重定时后各关节的速度/加速度占用率（1.0 = 贴上限）
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

from joint_traj_retime.core import (Limits, build_trajectory, max_time_scale,  # noqa: E402
                                    retime, utilization)
from joint_traj_retime.examples import JOINT_NAMES, example  # noqa: E402


def _setup_cjk():
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                 "WenQuanYi Zen Hei", "PingFang SC"):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False


_setup_cjk()


def fig_traj(out_dir):
    tk, qk, vmax, amax = example()
    traj = build_trajectory(tk, qk)
    r = max_time_scale(traj, Limits(vmax, amax))
    fast = retime(traj, r["alpha"])

    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    for j, style in ((0, "-"), (1, "--")):
        ax.plot(traj.t, traj.q[j], style, color="tab:blue", linewidth=1.6,
                label="%s 重定时前" % JOINT_NAMES[j])
        ax.plot(fast.t, fast.q[j], style, color="tab:red", linewidth=1.6,
                label="%s 重定时后（α=%.2f）" % (JOINT_NAMES[j], r["alpha"]))
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("关节角度 (rad)")
    ax.set_title("重定时前后：形状不变，时间轴压缩")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    path = os.path.join(out_dir, "traj_before_after.png")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def fig_utilization(out_dir):
    tk, qk, vmax, amax = example()
    traj = build_trajectory(tk, qk)
    r = max_time_scale(traj, Limits(vmax, amax))
    fast = retime(traj, r["alpha"])
    u = utilization(fast, Limits(vmax, amax))

    x = range(len(JOINT_NAMES))
    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    w = 0.38
    ax.bar([i - w / 2 for i in x], u["speed"], width=w, label="速度占用", color="tab:blue")
    ax.bar([i + w / 2 for i in x], u["accel"], width=w, label="加速度占用", color="tab:orange")
    ax.axhline(1.0, color="red", linestyle="--", linewidth=1.2, label="上限")
    ax.set_xticks(list(x))
    ax.set_xticklabels(JOINT_NAMES, rotation=20, ha="right", fontsize=8)
    ax.set_ylabel("占用率（≤1 为满足）")
    ax.set_title("重定时后各关节占用率：谁贴着上限，谁还有余量")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3, axis="y")
    path = os.path.join(out_dir, "utilization.png")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=".")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    for p in (fig_traj(args.out), fig_utilization(args.out)):
        print(p)


if __name__ == "__main__":
    main()
