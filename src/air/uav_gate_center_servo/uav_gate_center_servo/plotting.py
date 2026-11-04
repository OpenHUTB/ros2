#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""文档配图：架构、深度标签、回归散点、训练曲线、轨迹、横向通过率。"""
from __future__ import annotations

import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .core.geometry import project_label  # noqa: E402
from .core.runner import mlp_policy, oracle_policy, run_episode  # noqa: E402
from .core.train import evaluate_regression  # noqa: E402

__all__ = ["make_all"]


def setup_font() -> None:
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei",
                                       "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False


def _save(fig, path: str) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print("  %s" % path)


def fig_architecture(path: str) -> None:
    fig, ax = plt.subplots(figsize=(10, 2.6))
    ax.axis("off")
    boxes = [(0.02, 0.42, "48×64\n深度图"), (0.18, 0.42, "12×16\n块平均"),
             (0.34, 0.42, "192→32→3\nMLP"), (0.52, 0.42, "P 伺服\nm/s"),
             (0.70, 0.42, "几何控制器\n四旋翼")]
    for x, y, txt in boxes:
        ax.add_patch(plt.Rectangle((x, y), 0.13, 0.24, fill=False,
                                   edgecolor="#334155", lw=1.4))
        ax.text(x + 0.065, y + 0.12, txt, ha="center", va="center", fontsize=10)
    for x in (0.15, 0.31, 0.47, 0.65):
        ax.annotate("", xy=(x + 0.03, 0.54), xytext=(x, 0.54),
                    arrowprops=dict(arrowstyle="->", color="#334155"))
    ax.text(0.40, 0.18, "训练时用仿真真值标签；推理只用深度图", ha="center",
            fontsize=9, color="#64748b")
    ax.set_xlim(0, 0.86)
    ax.set_ylim(0.1, 0.8)
    _save(fig, path)


def fig_depth_label(env, path: str) -> None:
    obs = env.reset(seed=0)
    raw = env.raw_depth()
    u, v = project_label(obs["label"], raw.shape)
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.2))
    im = axes[0].imshow(raw, cmap="viridis")
    axes[0].plot([u], [v], "r+", ms=14, mew=2)
    axes[0].set_title("48×64 深度 + 门中心投影")
    plt.colorbar(im, ax=axes[0], fraction=0.046, pad=0.02, label="m")
    axes[1].imshow(obs["depth"], cmap="viridis")
    axes[1].set_title("12×16 网络输入")
    fig.suptitle("label = [%.2f, %.2f, %.2f] m"
                 % tuple(obs["label"]), fontsize=9)
    _save(fig, path)


def fig_scatter(model, ds, path: str) -> None:
    pred = model.predict(ds.val_depth)
    y = ds.val_label
    names = ["forward", "left", "up"]
    fig, axes = plt.subplots(1, 3, figsize=(9, 3.0))
    for i, ax in enumerate(axes):
        ax.scatter(y[:, i], pred[:, i], s=8, alpha=0.5, color="#1f77b4")
        lo = float(min(y[:, i].min(), pred[:, i].min())) - 0.2
        hi = float(max(y[:, i].max(), pred[:, i].max())) + 0.2
        ax.plot([lo, hi], [lo, hi], "k--", lw=0.8)
        ax.set_xlabel("true %s (m)" % names[i])
        ax.set_ylabel("pred (m)")
        rmse = float(np.sqrt(np.mean((pred[:, i] - y[:, i]) ** 2)))
        ax.set_title("%s RMSE %.3f m" % (names[i], rmse))
    _save(fig, path)


def fig_training(hist, path: str) -> None:
    ep = hist["epoch"]
    fig, ax = plt.subplots(figsize=(6.5, 3.2))
    ax.plot(ep, hist["train_mse"], label="train MSE (standardized)")
    ax.set_xlabel("epoch")
    ax.set_ylabel("train MSE")
    ax2 = ax.twinx()
    ax2.plot(ep, hist["val_rmse_distance"], color="#d62728",
             label="val distance RMSE")
    ax2.set_ylabel("val distance RMSE (m)")
    best = hist.get("best_epoch", -1)
    if best >= 0:
        ax2.axvline(best, color="#d62728", ls=":", lw=0.8, label="_nolegend_")
        ax2.text(best, ax2.get_ylim()[1], " best %d" % best,
                 va="top", fontsize=8, color="#d62728")
    lines = [l for l in ax.get_lines() + ax2.get_lines()
             if not str(l.get_label()).startswith("_")]
    ax.legend(lines, [l.get_label() for l in lines], loc="upper right",
              fontsize=8)
    _save(fig, path)


def _plot_traj(ax_top, ax_side, pos, title: str, half: float) -> None:
    ax_top.plot(pos[:, 0], pos[:, 1], lw=1.4)
    ax_top.axvline(5.0, color="#d62728", ls="--", lw=0.8)
    ax_top.axhline(-half, color="#d62728", ls=":", lw=0.6)
    ax_top.axhline(half, color="#d62728", ls=":", lw=0.6)
    ax_top.set_xlabel("x (m)")
    ax_top.set_ylabel("y (m)")
    ax_top.set_title(title)
    ax_side.plot(pos[:, 0], pos[:, 2], lw=1.4)
    ax_side.axvline(5.0, color="#d62728", ls="--", lw=0.8)
    ax_side.set_xlabel("x (m)")
    ax_side.set_ylabel("z (m)")
    ax_side.set_title("侧视")


def fig_trajectory(env, model, path: str) -> None:
    oracle = run_episode(env, oracle_policy(env), seed=0)
    mlp = run_episode(env, mlp_policy(model, env), seed=0)
    fig, axes = plt.subplots(2, 2, figsize=(9, 6))
    _plot_traj(axes[0, 0], axes[0, 1], oracle["pos"],
               "oracle: %s %.1fs" % (oracle["reason"], oracle["time"]),
               env.gate.half)
    _plot_traj(axes[1, 0], axes[1, 1], mlp["pos"],
               "MLP: %s %.1fs" % (mlp["reason"], mlp["time"]),
               env.gate.half)
    _save(fig, path)


def fig_pass_lateral(results: dict, path: str) -> None:
    d = results["pass_vs_lateral"]
    keys = sorted(d.keys(), key=float)
    vals = [d[k] for k in keys]
    fig, ax = plt.subplots(figsize=(5.5, 3.2))
    ax.bar([float(k) for k in keys], vals, width=0.35, color="#1f77b4")
    ax.set_ylim(0, 1.05)
    ax.set_xlabel("start lateral offset (m)")
    ax.set_ylabel("pass rate")
    ax.set_title("MLP pass rate vs lateral offset")
    _save(fig, path)


def make_all(env, model, ds, hist, outdir: str) -> None:
    setup_font()
    os.makedirs(outdir, exist_ok=True)
    fig_architecture(os.path.join(outdir, "00_architecture.png"))
    fig_depth_label(env, os.path.join(outdir, "01_depth_label.png"))
    fig_scatter(model, ds, os.path.join(outdir, "02_scatter.png"))
    fig_training(hist, os.path.join(outdir, "03_training.png"))
    fig_trajectory(env, model, os.path.join(outdir, "04_trajectory.png"))
    from .cli import _default
    rp = _default("results.json")
    if os.path.exists(rp):
        with open(rp, "r", encoding="utf-8") as f:
            fig_pass_lateral(json.load(f),
                             os.path.join(outdir, "05_pass_lateral.png"))
    else:
        print("  跳过 05_pass_lateral.png（没有 results.json）")
