#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跑一遍三个场景，把闭环真飞过程录成 gif。

左栏是不开规则的直飞，右栏是开规则之后的同一场景，同一时刻同帧对比。
录的是 MuJoCo 刚体积分出来的真实位置，不是画出来的示意线。

用法（在本目录下）：python tools/make_gif.py
三个场景都会重跑一遍（MuJoCo 真飞，本机约一分多钟），然后重录四张图。
"""
from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_SIM = os.path.normpath(os.path.join(os.path.dirname(_PKG), "uav_mujoco_sim"))
for _p in (_PKG, _SIM):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

for _name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"):
    if any(f.name == _name for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.sans-serif"] = [_name]
        break
plt.rcParams["axes.unicode_minus"] = False

ROOT = os.path.dirname(_PKG)
OUT_DIR = os.path.join(ROOT, "..", "..", "docs", "img", "air", "uav_right_of_way")
C_BASE = "#7f7f7f"
C_RULE = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e"]

# 三个场景各录一段，时间轴统一从 0 开始
SCENES = [("headon", "对头相遇（各自右转 30°）"),
          ("cross", "交叉相遇（左让右）"),
          ("overtake", "追越（从左侧通过）")]


def run_scene(name):
    """跑一个场景的基线与规则两趟，返回 (sc, base_traj, rule_traj)。"""
    from uav_right_of_way.core.sim import run_fleet
    from uav_right_of_way.core.scenarios import get_scenario

    sc = get_scenario(name)
    # record_every 调小一点，出 gif 才不卡
    base = run_fleet(sc.plans, enable_rules=False, record_every=5)
    rule = run_fleet(sc.plans, enable_rules=True, record_every=5)
    return sc, dict(times=base.times, agents=base.pos), \
        dict(times=rule.times, agents=rule.pos)


def _draw(ax, traj, t_idx, color_of, title):
    ax.clear()
    ax.set_title(title, fontsize=10)
    for uid, rows in sorted(traj["agents"].items()):
        arr = np.asarray(rows)[: t_idx + 1]
        if len(arr) == 0:
            continue
        c = color_of(uid)
        ax.plot(arr[:, 0], arr[:, 1], "-", color=c, lw=1.2, alpha=0.75)
        ax.plot(arr[-1, 0], arr[-1, 1], "o", color=c, ms=7)
        ax.annotate(uid, (arr[-1, 0], arr[-1, 1]), textcoords="offset points",
                    xytext=(5, 4), fontsize=8, color=c)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25, lw=0.5)
    ax.set_xlabel("x / m", fontsize=8)
    ax.set_ylabel("y / m", fontsize=8)
    ax.tick_params(labelsize=8)


def make_gif(name, title, sc, base, rule, out):
    ids = sorted(base["agents"])
    color_of = lambda uid: C_RULE[ids.index(uid) % len(C_RULE)]  # noqa: E731

    tb = np.asarray(base["times"], float)
    tr = np.asarray(rule["times"], float)
    n = min(len(tb), len(tr))
    # 抽帧，别录几百帧进来
    frames = np.linspace(0, n - 1, min(n, 56)).astype(int)

    fig, axes = plt.subplots(1, 2, figsize=(7.8, 3.5))
    fig.suptitle(title, fontsize=11)

    def update(k):
        k = frames[k]
        _draw(axes[0], base, k, color_of, "不给规则，直接飞")
        _draw(axes[1], rule, k, color_of, "给规则，实时让路")
        fig.suptitle("%s      t = %.0f s" % (title, tb[k]), fontsize=11)
        return []

    anim = FuncAnimation(fig, update, frames=len(frames), interval=180,
                         blit=False)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    anim.save(out, writer=PillowWriter(fps=6))
    plt.close(fig)
    return out


def main():
    runs = []
    for name, title in SCENES:
        sc, base, rule = run_scene(name)
        runs.append((name, title, base, rule))
        out = os.path.join(OUT_DIR, "%s.gif" % name)
        make_gif(name, title, sc, base, rule, out)
        print("saved", os.path.normpath(out))

    # 三场景俯视总图（静态），给文档当第一张图
    make_overview(runs)


def make_overview(runs):
    fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.2))
    for ax, (name, title, base, rule) in zip(axes, runs):
        ids = sorted(base["agents"])
        for i, uid in enumerate(ids):
            ab = np.asarray(base["agents"][uid])
            ar = np.asarray(rule["agents"][uid])
            c = C_RULE[i % len(C_RULE)]
            ax.plot(ab[:, 0], ab[:, 1], "--", color=c, lw=1.0, alpha=0.40)
            ax.plot(ar[:, 0], ar[:, 1], "-", color=c, lw=1.1, alpha=0.85)
        ax.set_title(title.split("（")[0], fontsize=10)
        ax.set_aspect("equal")
        ax.grid(alpha=0.25, lw=0.5)
        ax.tick_params(labelsize=8)
        ax.set_xlabel("x / m", fontsize=8)
        ax.set_ylabel("y / m", fontsize=8)
    axes[0].plot([], [], "--", color=C_BASE, label="基线")
    axes[0].plot([], [], "-", color=C_BASE, label="规则")
    axes[0].legend(fontsize=7, loc="best")
    fig.tight_layout()
    out = os.path.join(OUT_DIR, "trajectories.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print("saved", os.path.normpath(out))


if __name__ == "__main__":
    main()
