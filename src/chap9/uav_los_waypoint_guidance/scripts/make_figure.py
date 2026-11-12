#!/usr/bin/env python3
"""LOS 航点跟踪轨迹图。"""

import argparse
import csv
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

WAYPOINTS = [(4.0, 0.0), (8.0, 3.0), (12.0, 0.0), (16.0, -3.0)]
PATH = [(0.0, 0.0)] + WAYPOINTS


def cross_track(px, py):
    best = None
    for (x1, y1), (x2, y2) in zip(PATH[:-1], PATH[1:]):
        dx, dy = x2 - x1, y2 - y1
        n2 = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / n2))
        cx, cy = x1 + t * dx, y1 + t * dy
        d = math.hypot(px - cx, py - cy)
        if best is None or d < best:
            best = d
    return best if best is not None else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="/tmp/uav_los_waypoint_guidance.csv")
    ap.add_argument("--out", default="/tmp/uav_los_waypoint_guidance.png")
    args = ap.parse_args()

    rows = []
    with open(args.csv, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})
    if not rows:
        raise SystemExit("empty csv")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))
    ax1.plot([w[0] for w in WAYPOINTS], [w[1] for w in WAYPOINTS], "k--o",
             lw=1.2, ms=5, label="waypoints")
    ax1.plot([r["x"] for r in rows], [r["y"] for r in rows], "#d62728", lw=2, label="UAV trajectory")
    ax1.scatter([rows[0]["x"]], [rows[0]["y"]], c="green", s=45, label="start")
    for i, (wx, wy) in enumerate(WAYPOINTS):
        ax1.annotate("wp%d" % i, (wx, wy), textcoords="offset points", xytext=(4, 4), fontsize=7)
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.set_title("LOS waypoint guidance")
    ax1.axis("equal")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=7)

    err = [cross_track(r["x"], r["y"]) for r in rows]
    ax2.plot([r["t"] for r in rows], err, "#2ca02c", lw=1.6)
    ax2.axhline(0.5, color="red", ls=":", label="0.5 m")
    ax2.set_xlabel("t [s]")
    ax2.set_ylabel("distance to waypoint path [m]")
    ax2.set_title("path deviation (max %.3f m)" % max(err))
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(args.out, dpi=140)
    print("saved %s rows=%d max_dev=%.3f" % (args.out, len(rows), max(err)))


if __name__ == "__main__":
    main()
