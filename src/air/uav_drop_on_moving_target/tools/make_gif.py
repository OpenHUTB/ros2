#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成文档配图：三张双栏对比动图 + 一张总览图。

左栏是"瞄准车辆当前位置"的基线，右栏是按未来位置解算的策略；两栏飞的是
同一条 MuJoCo 真实轨迹，只有松手时刻不同。
"""
from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.normpath(os.path.join(_ROOT, "..", "uav_mujoco_sim")))

import matplotlib                                          # noqa: E402
matplotlib.use("Agg")

import matplotlib.animation as animation                    # noqa: E402
import matplotlib.pyplot as plt                             # noqa: E402
import numpy as np                                          # noqa: E402

from uav_drop_on_moving_target.core.ballistics import impact_point  # noqa: E402
from uav_drop_on_moving_target.core.cases import CASES             # noqa: E402
from uav_drop_on_moving_target.core.flyer import fly_case          # noqa: E402
from uav_drop_on_moving_target.core.release import (               # noqa: E402
    _uav_state_at, solve_release)

OUT = os.path.normpath(os.path.join(_ROOT, "..", "..", "..", "docs", "img",
                                    "air", "uav_drop_on_moving_target"))

# 中文标签必须显式指定字体，否则 matplotlib 默认字体没有 CJK 字形，图上是方框
matplotlib.rcParams["font.sans-serif"] = [
    "Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

def _set_limits(ax, lo, hi, pad=10.0):
    """保证两个方向都有跨度：恒 y（直线算例）会让 equal aspect 的变换奇异。"""
    x0, x1 = float(lo[0]), float(hi[0])
    y0, y1 = float(lo[1]), float(hi[1])
    if x1 - x0 < pad:
        x0, x1 = x0 - pad / 2, x0 + pad / 2
    if y1 - y0 < pad:
        y0, y1 = y0 - pad / 2, y0 + pad / 2
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)


C_UAV = "#1f4e9c"
C_CONVOY = "#c8641e"
C_GOOD = "#1f8a4c"
C_BAD = "#c0392b"


def _focus(rec, sol, case):
    """把视野裁到投放事件附近：整条 640 m 的路画进来就成了一条线。"""
    xs = [rec.release_pos[0], rec.impact_xy[0],
          sol.naive_impact_xy[0] if sol.naive_impact_xy is not None
          else rec.impact_xy[0]]
    ys = [rec.release_pos[1], rec.impact_xy[1],
          case.vehicle.pos_at(rec.t_release)[1]]
    return min(xs) - 130.0, max(xs) + 70.0, min(ys) - 45.0, max(ys) + 45.0


def _panel(ax, case, sol, naive_pos, naive_impact, title):
    ax.set_title(title, fontsize=9)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.25, lw=0.5)
    ax.set_xlabel("x (m)", fontsize=8)
    ax.set_ylabel("y (m)", fontsize=8)
    ax.tick_params(labelsize=7)

    track = np.asarray(case.track.points, dtype=float)
    ax.plot(track[:, 0], track[:, 1], ls="--", lw=0.9, color=C_UAV,
            alpha=0.55, label="无人机标称航迹")
    path = np.asarray(case.vehicle.waypoints, dtype=float)
    ax.plot(path[:, 0], path[:, 1], ls=":", lw=1.1, color=C_CONVOY,
            alpha=0.7, label="车队路线")

    uav_line, = ax.plot([], [], lw=1.2, color=C_UAV, label="无人机轨迹")
    convoy_pts, = ax.plot([], [], "o", ms=3, color=C_CONVOY, alpha=0.8,
                          label="车队走过的点")
    convoy_now, = ax.plot([], [], "s", ms=7, color=C_CONVOY, label="车队当前位置")
    drop_mark, = ax.plot([], [], "v", ms=8, color=C_BAD, label="松手点")
    ax.plot(naive_impact[0], naive_impact[1], "x", ms=9, color=C_BAD,
            label="基线落点")
    ax.plot(sol.impact_xy[0], sol.impact_xy[1], "*", ms=13, color=C_GOOD,
            label="本次落点")
    ax.annotate("基线偏 %.1f m" % float(np.linalg.norm(naive_impact - naive_pos)),
                xy=naive_impact, xytext=(8, 10), textcoords="offset points",
                fontsize=7, color=C_BAD)
    ax.legend(fontsize=6, loc="upper left", framealpha=0.85)
    return uav_line, convoy_pts, convoy_now, drop_mark


def make_gif(case, res, sol, path_out, fps=8, stride=4):
    tt = res.times[::stride]
    pp = res.pos[::stride]
    rec = res.releases[0]

    naive_t = sol.naive_t_release
    naive_pos = case.vehicle.pos_at(naive_t + case.cfg.t_fall)
    p3, v3 = _uav_state_at(case.track, naive_t, case.cfg)
    naive_impact = impact_point(p3, v3, case.cfg.wind, case.cfg.ball)
    sol.naive_impact_xy = naive_impact

    fig, axes = plt.subplots(1, 2, figsize=(9.2, 4.2))
    left = _panel(axes[0], case, sol, naive_pos, naive_impact,
                  "瞄准车队当前位置（基线）")
    right = _panel(axes[1], case, sol, naive_pos, naive_impact,
                   "预测车队未来位置（本模块）")

    x0, x1, y0, y1 = _focus(rec, sol, case)
    for ax in axes:
        _set_limits(ax, (x0, y0), (x1, y1), pad=0.0)

    def init():
        for art in left + right:
            art.set_data([], [])
        return left + right

    def update(i):
        t = tt[i]
        for arts in (left, right):
            uav_line, convoy_pts, convoy_now, drop_mark = arts
            uav_line.set_data(pp[:i + 1, 0], pp[:i + 1, 1])
            past = np.asarray([case.vehicle.pos_at(s)
                               for s in np.linspace(0, t, 40)])
            convoy_pts.set_data(past[:, 0], past[:, 1])
            now = case.vehicle.pos_at(t)
            convoy_now.set_data([now[0]], [now[1]])
            if t >= rec.t_release:
                drop_mark.set_data([rec.release_pos[0]], [rec.release_pos[1]])
        return left + right

    anim = animation.FuncAnimation(fig, update, init_func=init,
                                   frames=len(tt), interval=1000 // fps,
                                   blit=False)
    anim.save(path_out, writer=animation.PillowWriter(fps=fps))
    plt.close(fig)
    print("[gif] %s" % os.path.relpath(path_out, _ROOT))


def make_overview(rows, path_out):
    fig, axes = plt.subplots(1, 3, figsize=(11.5, 3.6))
    for ax, (case, res, sol) in zip(axes, rows):
        ax.set_title("%s\n落点误差 %.2f m / 基线 %.1f m"
                     % (case.title, res.releases[0].error_m, sol.naive_miss_m),
                     fontsize=8)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=0.25, lw=0.5)
        tr = np.asarray(case.track.points, dtype=float)
        ax.plot(tr[:, 0], tr[:, 1], "--", lw=0.9, color=C_UAV, alpha=0.5)
        ax.plot(res.pos[:, 0], res.pos[:, 1], lw=1.2, color=C_UAV)
        vp = np.asarray([case.vehicle.pos_at(t)
                         for t in np.linspace(0, res.times[-1], 120)])
        ax.plot(vp[:, 0], vp[:, 1], ":", lw=1.1, color=C_CONVOY)
        rec = res.releases[0]
        ax.plot(rec.release_pos[0], rec.release_pos[1], "v", ms=7, color=C_BAD)
        ax.plot(rec.impact_xy[0], rec.impact_xy[1], "*", ms=12, color=C_GOOD)
        ax.plot(rec.vehicle_at_impact[0], rec.vehicle_at_impact[1], "s", ms=6,
                mfc="none", mec=C_CONVOY)
        ax.tick_params(labelsize=7)
        ax.set_xlabel("x (m)", fontsize=8)
        x0, x1, y0, y1 = _focus(rec, sol, case)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
    fig.tight_layout()
    fig.savefig(path_out, dpi=160)
    plt.close(fig)
    print("[png] %s" % os.path.relpath(path_out, _ROOT))


def main():
    os.makedirs(OUT, exist_ok=True)
    rows = []
    for case in CASES:
        sol = solve_release(case.track, case.vehicle, case.cfg)
        res = fly_case(case.track, case.vehicle, case.cfg)
        rows.append((case, res, sol))
        make_gif(case, res, sol, os.path.join(OUT, "%s.gif" % case.name))
    make_overview(rows, os.path.join(OUT, "overview.png"))


if __name__ == "__main__":
    main()
