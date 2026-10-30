"""Aggregate training results and plot comparison curves."""

import os
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "..", "results")


def _read_csv(path):
    episodes, returns = [], []
    if not os.path.exists(path):
        return episodes, returns
    with open(path, "r") as f:
        for row in csv.DictReader(f):
            val = float(row["episode_return"])
            if not np.isfinite(val):
                continue
            episodes.append(int(row["episode"]))
            returns.append(val)
    return episodes, returns


def _smooth(values, window=5):
    if len(values) < window:
        return values
    kernel = np.ones(window) / window
    return np.convolve(values, kernel, mode="valid")


def plot_all(envs=("halfcheetah", "ant", "walker2d"), algs=("ppo", "sac")):
    fig, axes = plt.subplots(1, len(envs), figsize=(5 * len(envs), 4), sharey=False)
    if len(envs) == 1:
        axes = [axes]
    colors = {"ppo": "tab:blue", "sac": "tab:orange"}
    for ax, env in zip(axes, envs):
        for alg in algs:
            eps, rets = _read_csv(os.path.join(RESULTS_DIR, f"{env}_{alg}.csv"))
            if not eps:
                continue
            smooth = _smooth(rets, 5)
            ax.plot(eps, rets, alpha=0.25, color=colors.get(alg, "gray"))
            ax.plot(eps[len(eps) - len(smooth):], smooth, linewidth=2,
                    label=alg.upper(), color=colors.get(alg, "gray"))
        ax.set_title(env.capitalize())
        ax.set_xlabel("Episode")
        ax.set_ylabel("Episode Return")
        ax.legend(loc="best")
        ax.grid(True, alpha=0.3)
    fig.suptitle("MuJoCo Locomotion: PPO vs SAC")
    fig.tight_layout()
    out_path = os.path.join(RESULTS_DIR, "comparison.png")
    fig.savefig(out_path, dpi=150)
    print(f"[Compare] Saved comparison plot to {out_path}")
    return out_path
