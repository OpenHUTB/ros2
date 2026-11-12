#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plot hand-eye calibration residuals from calibration_result.json."""
import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default="results/calibration_result.json")
    ap.add_argument("--out", default="results/calibration_error.png")
    args = ap.parse_args()

    rep = json.load(open(args.log, encoding="utf-8"))
    rows = rep["rows"]
    idx = np.array([r["pose"] + 1 for r in rows])
    terr = np.array([r["t_err_mm"] for r in rows])
    rerr = np.array([r["r_err_deg"] for r in rows])

    fig, axes = plt.subplots(2, 1, figsize=(8.0, 4.4), sharex=True)
    axes[0].bar(idx, terr, color="#2f81f7", width=0.6)
    axes[0].axhline(rep["board_translation_error_mm"]["mean"], color="#d29922", ls="--", lw=1,
                    label="mean = %.2f mm" % rep["board_translation_error_mm"]["mean"])
    axes[0].set_ylabel("Translation error (mm)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(axis="y", alpha=0.25)

    axes[1].bar(idx, rerr, color="#3fb950", width=0.6)
    axes[1].axhline(rep["board_rotation_error_deg"]["mean"], color="#d29922", ls="--", lw=1,
                    label="mean = %.3f deg" % rep["board_rotation_error_deg"]["mean"])
    axes[1].set_ylabel("Rotation error (deg)")
    axes[1].set_xlabel("Calibration pose")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].grid(axis="y", alpha=0.25)
    axes[0].set_title("Hand-eye calibration residual per pose (%s)" % rep.get("method", ""))

    plt.tight_layout()
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    plt.savefig(args.out, dpi=150)
    print("saved", args.out)


if __name__ == "__main__":
    main()
