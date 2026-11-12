#!/usr/bin/env python3
"""倒车轨迹与横向误差图。"""

import argparse
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

PATH = [(0.0, 0.0), (2.0, 0.0), (3.5, 0.8), (5.0, 0.8), (6.5, 0.0), (8.0, 0.0)]


def lateral_error(px, py):
    best = (1e9, 0.0)
    for (x1, y1), (x2, y2) in zip(PATH[:-1], PATH[1:]):
        dx, dy = x2 - x1, y2 - y1
        n2 = dx * dx + dy * dy
        if n2 < 1e-12:
            continue
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / n2))
        cx, cy = x1 + t * dx, y1 + t * dy
        d = ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5
        if d < best[0]:
            best = (d, t)
    return best[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="/tmp/pure_pursuit_reverse.csv")
    ap.add_argument("--out", default="/tmp/pure_pursuit_reverse.png")
    args = ap.parse_args()

    rows = []
    with open(args.csv, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))
    px = [p[0] for p in PATH]
    py = [p[1] for p in PATH]
    ax1.plot(px, py, "k--", lw=1.8, label="reference path")
    ax1.plot([r["x"] for r in rows], [r["y"] for r in rows], "#d62728", lw=2, label="reverse trajectory")
    ax1.scatter([px[0]], [py[0]], c="green", s=45, label="start")
    ax1.scatter([px[-1]], [py[-1]], c="blue", s=45, marker="*", label="goal")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.set_title("reverse pure-pursuit tracking")
    ax1.axis("equal")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=7)

    err = [lateral_error(r["x"], r["y"]) for r in rows]
    ax2.plot([r["t"] for r in rows], err, "#2ca02c", lw=1.6)
    ax2.axhline(0.1, color="red", ls=":", label="0.10 m target")
    ax2.set_xlabel("t [s]")
    ax2.set_ylabel("cross-track error [m]")
    ax2.set_title("cross-track error (max %.3f m)" % max(err))
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(args.out, dpi=140)
    print("saved %s rows=%d max_err=%.3f" % (args.out, len(rows), max(err)))


if __name__ == "__main__":
    main()
