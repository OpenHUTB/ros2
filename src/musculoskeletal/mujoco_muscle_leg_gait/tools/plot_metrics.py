#!/usr/bin/env python3
"""从 bench 导出的 metrics.json 生成曲线图（L3 刚度、L4 髋误差）。

用法：
    python3 tools/plot_metrics.py out/metrics.json out
"""

import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "out/metrics.json"
    outdir = sys.argv[2] if len(sys.argv) > 2 else (os.path.dirname(path) or ".")
    os.makedirs(outdir, exist_ok=True)

    with open(path, encoding="utf-8") as f:
        d = json.load(f)

    # ---- L3：刚度随共激活 ----
    sweep = d["l3"]["sweep"]
    fig, ax = plt.subplots(figsize=(6, 3.4))
    for limb, style in (("ascending", "o-"), ("descending", "s-")):
        pts = [r for r in sweep if r["limb"] == limb]
        ax.plot([p["base"] for p in pts], [p["stiffness_num"] for p in pts],
                style, label=limb)
    ax.axhline(0, color="gray", lw=0.8)
    ax.set_xlabel("co-activation")
    ax.set_ylabel("stiffness (N m/rad)")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    p1 = os.path.join(outdir, "l3_stiffness.png")
    fig.savefig(p1, dpi=130)
    plt.close(fig)

    # ---- L4：三种模型的髋峰值误差 ----
    l4 = d["l4"]
    keys = [("A_mono_qp", "A mono+QP"),
            ("B_bia_naive", "B bia+naive"),
            ("C_bia_qp", "C bia+QP")]
    fig, ax = plt.subplots(figsize=(6, 3.2))
    ax.bar([n for _, n in keys], [l4[k]["hip_max"] for k, _ in keys],
           color=["#4c78a8", "#e45756", "#54a24b"])
    ax.set_ylabel("hip peak error (rad)")
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    p2 = os.path.join(outdir, "l4_hip_error.png")
    fig.savefig(p2, dpi=130)
    plt.close(fig)

    print(p1)
    print(p2)


if __name__ == "__main__":
    main()
