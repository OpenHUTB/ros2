#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跑一遍三个算例，把闭环真飞过程录成 gif。

左栏是"飞过目标正上方才松手"，右栏是"按解析解提前 lead 米松手"，
两栏飞的是同一条闭环轨迹，只有投放时刻不同，同一时刻同帧对比。
录的是 MuJoCo 刚体积分出来的真实位置。

用法（在本目录下）：python tools/make_gif.py
三个算例都会重跑一遍（本机约十几秒），然后重录四张图。
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
OUT_DIR = os.path.join(ROOT, "..", "..", "docs", "img", "air", "uav_emergency_drop")
C_FLY = "#1f77b4"
C_OK = "#2ca02c"
C_BAD = "#d62728"

CASES = [("case_aid_village", "村庄群 8 点：30 m / 8 m/s"),
         ("case_aid_river", "河对岸 6 点：绕桥，30 m / 8 m/s"),
         ("case_aid_wind", "8 m/s 横风 6 点：20 m / 6 m/s")]


def run_case(name):
    """跑一个算例的闭环，返回 case / 真飞结果。"""
    from uav_emergency_drop.core.cases import CASES as _CASES
    from uav_emergency_drop.core.flyer import build_route, fly_drop_route

    case = _CASES[name]
    route, true_targets, target_idx, no_cut = build_route(case)
    # record_every 调小一点，出 gif 才不卡
    fly = fly_drop_route(route, true_targets, case.plan,
                         target_idx=target_idx, record_every=5,
                         no_cut=no_cut)
    return case, fly


def naive_release(fly, plan):
    """不提前量：飞过越顶点（目标正上方）才松手，落点解析外推。

    返回 [(t, 释放位置, 落点, 目标)]，与真飞记录同一批投放点。
    """
    out = []
    pos = np.asarray(fly.pos)
    times = np.asarray(fly.times)
    for rec in fly.releases:
        wp = np.asarray(rec.overfly_wp[:2], float)
        d = np.linalg.norm(pos[:, :2] - wp, axis=1)
        i = int(np.argmin(d))
        j0, j1 = max(i - 1, 0), min(i + 1, len(pos) - 1)
        v = (pos[j1, :2] - pos[j0, :2]) / max(times[j1] - times[j0], 1e-6)
        v *= plan.speed / max(np.linalg.norm(v), 1e-6)   # 按标称巡航速度前抛
        impact = pos[i, :2] + v * plan.t_fall + plan.drift
        out.append((float(times[i]), pos[i, :2], impact,
                    np.asarray(rec.target[:2], float)))
    return out


def _static(ax, case, fly):
    """两栏共用的静态底图：需求点 + 计划航线 + 补给点。"""
    for p in case.points:
        ax.scatter([p.x], [p.y], s=28 + 16 * p.demand, c=[p.urgency],
                   cmap="viridis", vmin=1, vmax=5, zorder=3,
                   edgecolors="k", linewidths=0.4)
    route = np.asarray(fly.route, float)
    ax.plot(route[:, 0], route[:, 1], "--", color="#7f7f7f", lw=1.1,
            zorder=2, label="计划航线")
    ax.plot([route[0, 0]], [route[0, 1]], "s", color="#7f7f7f", ms=5,
            zorder=3, label="补给点")
    ax.set_aspect("equal")
    ax.grid(alpha=0.25, lw=0.5)
    ax.tick_params(labelsize=7)
    ax.set_xlabel("x / m", fontsize=8)
    ax.set_ylabel("y / m", fontsize=8)


def make_gif(name, title, case, fly, out):
    plan = case.plan
    naive = naive_release(fly, plan)
    good = [(float(r.t_release), np.asarray(r.release_pos[:2], float),
             np.asarray(r.predicted_impact[:2], float),
             np.asarray(r.target[:2], float)) for r in fly.releases]
    miss_naive = np.mean([np.linalg.norm(im - tg) for _t, _p, im, tg in naive])
    miss_good = np.mean([np.linalg.norm(im - tg) for _t, _p, im, tg in good])

    times = np.asarray(fly.times, float)
    pos = np.asarray(fly.pos)
    frames = np.linspace(0, len(times) - 1, min(len(times), 54)).astype(int)

    fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.8))
    xs = np.concatenate([pos[:, 0], [p.x for p in case.points],
                         [im[0] for _t, _p, im, _g in naive],
                         [im[0] for _t, _p, im, _g in good]])
    ys = np.concatenate([pos[:, 1], [p.y for p in case.points],
                         [im[1] for _t, _p, im, _g in naive],
                         [im[1] for _t, _p, im, _g in good]])
    xlim = (xs.min() - 25, xs.max() + 25)
    ylim = (ys.min() - 25, ys.max() + 25)

    def draw(ax, drops, label, color, k, t_now):
        ax.clear()
        _static(ax, case, fly)
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.plot(pos[: k + 1, 0], pos[: k + 1, 1], "-", color=C_FLY, lw=1.3,
                alpha=0.85, zorder=4)
        ax.plot(pos[k, 0], pos[k, 1], "o", color=C_FLY, ms=6, zorder=6)
        landed = 0
        for t_r, rel, imp, tgt in drops:
            if t_r <= t_now:
                ax.plot([rel[0], imp[0]], [rel[1], imp[1]], ":",
                        color=color, lw=1.0, zorder=3)
                ax.plot([imp[0]], [imp[1]], "x", color=color, ms=7,
                        mew=1.6, zorder=6)
                ax.annotate("%.1f m" % np.linalg.norm(imp - tgt),
                            (imp[0], imp[1]), textcoords="offset points",
                            xytext=(4, -8), fontsize=6.5, color=color)
                landed += 1
        ax.set_title("%s    已投 %d/%d" % (label, landed, len(drops)),
                     fontsize=9)

    def update(fi):
        k = frames[fi]
        t_now = times[k]
        draw(axes[0], naive,
             "飞过目标正上方才松手（平均偏 %.1f m）" % miss_naive, C_BAD,
             k, t_now)
        draw(axes[1], good,
             "按解析提前量松手（平均偏 %.1f m）" % miss_good, C_OK,
             k, t_now)
        fig.suptitle("%s      t = %.0f s" % (title, t_now), fontsize=10.5)
        return []

    anim = FuncAnimation(fig, update, frames=len(frames), interval=180,
                         blit=False)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    anim.save(out, writer=PillowWriter(fps=6))
    plt.close(fig)
    return out


def make_overview(runs):
    """三算例俯视总图（静态），给文档当第一张图。"""
    fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.3))
    for ax, (name, title, case, fly) in zip(axes, runs):
        _static(ax, case, fly)
        route = np.asarray(fly.route, float)
        for rec in fly.releases:
            imp = np.asarray(rec.predicted_impact[:2], float)
            tgt = np.asarray(rec.target[:2], float)
            ax.plot([imp[0]], [imp[1]], "x", color=C_OK, ms=6, mew=1.4)
            ax.annotate("%.1f m" % rec.error_m, (imp[0], imp[1]),
                        textcoords="offset points", xytext=(4, -8),
                        fontsize=6, color=C_OK)
        ax.set_title(title.split("：")[0], fontsize=9)
    axes[0].legend(fontsize=6.5, loc="upper left")
    fig.tight_layout()
    out = os.path.join(OUT_DIR, "overview.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print("saved", os.path.normpath(out))


def main():
    runs = []
    for name, title in CASES:
        case, fly = run_case(name)
        runs.append((name, title, case, fly))
        short = name.replace("case_aid_", "")
        out = os.path.join(OUT_DIR, "%s.gif" % short)
        make_gif(name, title, case, fly, out)
        print("saved", os.path.normpath(out))
    make_overview(runs)


if __name__ == "__main__":
    main()
