#!/usr/bin/env python3
"""由仿真 CSV 生成轨迹与前端有效距离图。"""

import argparse
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SEGMENTS = [
    (0.0, 1.20, 12.0, 1.20, "wall"),
    (0.0, -1.20, 12.0, -1.20, "wall"),
    (4.00, 0.05, 4.00, 1.20, "glass"),
    (7.00, -1.20, 7.00, -0.05, "foam"),
    (12.0, -1.20, 12.0, 1.20, "wall"),
]


def load(path):
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            rows.append({k: float(v) if k != "action" else v for k, v in row.items()})
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="/tmp/lidar_intensity_demo.csv")
    ap.add_argument("--out", default="/tmp/lidar_intensity_demo.png")
    args = ap.parse_args()

    rows = load(args.csv)
    if not rows:
        raise SystemExit("empty csv")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))

    for x1, y1, x2, y2, kind in SEGMENTS:
        color = {"wall": "#444444", "glass": "#1f77b4", "foam": "#111111"}[kind]
        style = "--" if kind == "glass" else "-"
        ax1.plot([x1, x2], [y1, y2], style, color=color, lw=2,
                 label={"wall": "wall", "glass": "glass (weak echo)",
                        "foam": "black foam (weak echo)"}[kind] if x1 == 4.0 or x1 == 7.0 or x1 == 0.0 else None)
    xs = [r["x"] for r in rows]
    ys = [r["y"] for r in rows]
    ax1.plot(xs, ys, color="#d62728", lw=2, label="robot path")
    ax1.scatter([xs[0]], [ys[0]], color="green", s=40, label="start")
    ax1.set_xlim(-0.5, 12.5)
    ax1.set_ylim(-1.6, 1.6)
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.set_title("trajectory around weak-echo obstacles")
    ax1.grid(alpha=0.3)
    ax1.legend(loc="lower right", fontsize=7)

    ts = [r["t"] for r in rows]
    eff = [r["min_eff"] for r in rows]
    ax2.plot(ts, eff, color="#2ca02c", lw=1.6)
    ax2.axhline(0.45, color="red", ls=":", label="stop_distance 0.45 m")
    ax2.set_xlabel("t [s]")
    ax2.set_ylabel("front effective range [m]")
    ax2.set_title("front effective range (intensity-weighted)")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(args.out, dpi=140)
    print("saved " + args.out + " rows=" + str(len(rows)))


if __name__ == "__main__":
    main()
