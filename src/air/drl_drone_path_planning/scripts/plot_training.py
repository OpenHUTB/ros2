#!/usr/bin/env python3
"""Plot the evaluation curve produced by SB3's ``EvalCallback``.

This is an offline development utility (needs matplotlib); the ROS deployment
does not use it.  Run after training:

    python3 scripts/plot_training.py --npz models/evaluations.npz

Developer: 汤湘粤 (学号: 2535030067)
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from common import PACKAGE_DIR  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(PACKAGE_DIR, "..", "..", ".."))
DEFAULT_NPZ = os.path.join(PACKAGE_DIR, "models", "evaluations.npz")
DEFAULT_OUT = os.path.join(
    REPO_ROOT, "docs", "img", "drl_drone_path_planning", "training_curve.png"
)


def main() -> None:
    """Render mean reward and episode length against training steps."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("matplotlib is required for plotting") from exc

    parser = argparse.ArgumentParser(description="Plot EvalCallback history")
    parser.add_argument("--npz", default=DEFAULT_NPZ)
    parser.add_argument("--out", default=DEFAULT_OUT)
    args = parser.parse_args()

    with np.load(args.npz) as data:
        timesteps = data["timesteps"]
        results = data["results"]
        lengths = data["ep_lengths"]

    reward_mean = results.mean(axis=1)
    reward_std = results.std(axis=1)
    length_mean = lengths.mean(axis=1)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    figure, axis = plt.subplots(figsize=(8.0, 4.5))
    axis.plot(timesteps, reward_mean, color="#2ca02c", linewidth=1.8,
              label="mean evaluation reward")
    axis.fill_between(timesteps, reward_mean - reward_std,
                      reward_mean + reward_std, color="#2ca02c", alpha=0.18)
    axis.set_xlabel("training timesteps")
    axis.set_ylabel("episode reward")
    axis.grid(True, linestyle=":", alpha=0.5)

    length_axis = axis.twinx()
    length_axis.plot(timesteps, length_mean, color="#1f77b4", linewidth=1.4,
                     linestyle="--", label="mean episode length")
    length_axis.set_ylabel("episode length / steps")

    lines = axis.get_lines() + length_axis.get_lines()
    labels = [line.get_label() for line in lines]
    axis.legend(lines, labels, loc="best", fontsize=8)
    axis.set_title("PPO training history (fixed-start deterministic evaluation)",
                   fontsize=10)
    figure.tight_layout()
    figure.savefig(args.out, dpi=160)
    print("[plot] wrote %s" % args.out)


if __name__ == "__main__":
    main()
