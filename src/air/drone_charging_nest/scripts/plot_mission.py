#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plot the mission battery / altitude curves from results/mission_log.json."""
import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default="results/mission_log.json")
    ap.add_argument("--out", default="results/mission_curve.png")
    args = ap.parse_args()

    log = json.load(open(args.log, encoding="utf-8"))
    t = [x["t"] for x in log]
    z = [x["z"] for x in log]
    bat = [x["battery"] for x in log]
    phase = [x["phase"] for x in log]

    # phase boundaries / labels
    bounds = []
    labels = []
    last = phase[0]
    start = t[0]
    for ti, p in zip(t, phase):
        if p != last:
            bounds.append(ti)
            labels.append(last)
            last = p
            start = ti
    bounds.append(t[-1])
    labels.append(last)

    fig, axes = plt.subplots(2, 1, figsize=(8.0, 4.6), sharex=True)
    axes[0].plot(t, z, color="#1f77b4", lw=1.6)
    axes[0].set_ylabel("Altitude (m)")
    axes[0].grid(alpha=0.25)
    axes[0].set_title("Drone auto charging nest - mission profile")

    axes[1].plot(t, bat, color="#2ca02c", lw=1.8)
    axes[1].set_ylabel("Battery (%)")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylim(40, 102)
    axes[1].grid(alpha=0.25)

    for ax in axes:
        for b in bounds[1:-1]:
            ax.axvline(b, color="#888888", lw=0.6, ls="--", alpha=0.5)
    for i in range(len(bounds) - 1):
        lab = labels[i]
        x = (bounds[i] + bounds[i + 1]) / 2.0
        if lab in ("CHARGING", "TAKEOFF", "LANDING"):
            axes[0].annotate(lab, (x, max(z) * 0.92), ha="center", fontsize=7, color="#555555")

    plt.tight_layout()
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    plt.savefig(args.out, dpi=150)
    print("saved", args.out)


if __name__ == "__main__":
    main()
