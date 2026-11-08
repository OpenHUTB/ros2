#!/usr/bin/env python3
"""从 metrics.json 生成三张图：夹持力、抬升阈值、摩擦对比。

    python3 tools/plot_metrics.py out/metrics.json out
"""

import json
import os
import sys

import matplotlib
matplotlib.use("Agg")  # noqa: E402
import matplotlib.pyplot as plt


def plot_g1(d, outdir):
    lv = d["g1"]["levels"]
    a = [x["activation"] for x in lv]
    f = [x["grip_force_n"] for x in lv]
    fig, ax = plt.subplots(figsize=(5.6, 3.4))
    ax.plot(a, f, "o-", color="#4c78a8")
    ax.set_xlabel("muscle activation")
    ax.set_ylabel("contact normal force (N, two fingers)")
    ax.set_title("activation -> contact normal force (0 below contact)")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    p = os.path.join(outdir, "g1_force_vs_activation.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def plot_g2(d, outdir):
    lv = d["g2"]["levels"]
    a = [x["activation"] for x in lv]
    h = [x["lift_mm"] for x in lv]
    colors = ["#54a24b" if x["gripped"] else "#e45756" for x in lv]
    fig, ax = plt.subplots(figsize=(5.8, 3.4))
    ax.bar([f"{x:.2f}" for x in a], h, color=colors)
    ax.set_xlabel("muscle activation")
    ax.set_ylabel("lift height (mm)")
    ax.set_title("grasp threshold (green = gripped, red = slipped)")
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    p = os.path.join(outdir, "g2_lift_threshold.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def plot_g3(d, outdir):
    lv = d["g3"]["levels"]
    mus = sorted({x["friction"] for x in lv})
    acts = sorted({x["activation"] for x in lv})
    fig, ax = plt.subplots(figsize=(5.8, 3.4))
    width = 0.35
    for k, act in enumerate(acts):
        vals = [next(x["lift_mm"] for x in lv
                     if x["friction"] == mu and x["activation"] == act) for mu in mus]
        xs = [i + (k - (len(acts) - 1) / 2) * width for i in range(len(mus))]
        ax.bar(xs, vals, width=width, label=f"act={act}")
    ax.set_xticks(range(len(mus)))
    ax.set_xticklabels([f"mu={m}" for m in mus])
    ax.set_ylabel("lift height (mm)")
    ax.set_title("friction vs lift")
    ax.legend()
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    p = os.path.join(outdir, "g3_friction.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def plot_g5(d, outdir):
    lv = d["g5"]["levels"]
    targets = [x["target_n"] for x in lv]
    acts = [x["final_activation"] for x in lv]
    fig, ax = plt.subplots(figsize=(5.4, 3.2))
    ax.bar([f"{t:.0f} N" for t in targets], acts, color="#4c78a8")
    ax.set_ylabel("final activation")
    ax.set_title("force loop: activation needed per target")
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    p = os.path.join(outdir, "g5_force_loop.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def plot_g6(d, outdir):
    a = d["g6"]["no_reflex"]["lift_mm"]
    b = d["g6"]["with_reflex"]["lift_mm"]
    fig, ax = plt.subplots(figsize=(4.8, 3.2))
    ax.bar(["no reflex", "with reflex"], [a, b], color=["#e45756", "#54a24b"])
    ax.set_ylabel("lift height (mm)")
    ax.set_title("slip reflex recovery")
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    p = os.path.join(outdir, "g6_reflex.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "out/metrics.json"
    outdir = sys.argv[2] if len(sys.argv) > 2 else (os.path.dirname(path) or ".")
    os.makedirs(outdir, exist_ok=True)
    with open(path, encoding="utf-8") as f:
        d = json.load(f)
    plotters = [plot_g1, plot_g2, plot_g3]
    if "g5" in d:
        plotters.append(plot_g5)
    if "g6" in d:
        plotters.append(plot_g6)
    for fn in plotters:
        print(fn(d, outdir))


if __name__ == "__main__":
    main()
