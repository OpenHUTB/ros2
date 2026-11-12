#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跑一遍三个算例，把闭环跟随过程录成 gif。

左栏是"直接悬在目标正上方"的朴素参考，右栏是"目标正后方 standoff
环"参考；两栏飞的是同一条闭环轨迹（同一次真飞），只有参考点不同，
同一时刻同帧对比。录的是 MuJoCo 刚体积分出来的真实位置。

用法（在本目录下）：python tools/make_gif.py
三个算例都会重跑一遍（本机约几秒），然后重录四张图。
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
OUT_DIR = os.path.join(ROOT, "..", "..", "docs", "img", "air",
                       "uav_moving_target_follow")
C_FLY = "#1f77b4"
C_OK = "#2ca02c"
C_BAD = "#d62728"
C_TGT = "#ff7f0e"

CASES = [("road", "郊外直路：6 m/s 匀速"),
         ("town", "城镇街区：两个 90° 转弯"),
         ("s_curve", "S 形连续弯：3 个折弯")]


def run_case(name):
    from uav_moving_target_follow.core.flyer import follow_target
    from uav_moving_target_follow.core.follow import FollowCfg
    from uav_moving_target_follow.core.target import make_case

    case = make_case(name)
    fly = follow_target(case, FollowCfg(), record_every=5)
    return case, fly


def _static(ax, case, fly):
    """两栏共用的静态底图：目标路径 + 起终点。"""
    path = np.asarray(case.waypoints, float)
    ax.plot(path[:, 0], path[:, 1], "--", color="#7f7f7f", lw=1.1,
            zorder=2, label="目标路径")
    ax.plot([path[0, 0]], [path[0, 1]], "s", color="#7f7f7f", ms=5,
            zorder=3, label="目标起点")
    ax.set_aspect("equal")
    ax.grid(alpha=0.25, lw=0.5)
    ax.tick_params(labelsize=7)
    ax.set_xlabel("x / m", fontsize=8)
    ax.set_ylabel("y / m", fontsize=8)


def make_gif(name, title, case, fly, out):
    cfg_standoff = 12.0
    times = np.asarray(fly.times, float)
    pos = np.asarray(fly.quad_pos)
    tgt = np.asarray(fly.tgt_pos)
    frames = np.linspace(0, len(times) - 1, min(len(times), 54)).astype(int)

    # 朴素基线：参考点 = 目标正上方；误差用同一条真实轨迹
    naive_err = np.linalg.norm(pos[:, :2] - tgt, axis=1)
    good_err = fly.follow_err

    fig, axes = plt.subplots(1, 2, figsize=(8.6, 3.9))
    xs = np.concatenate([pos[:, 0], tgt[:, 0]])
    ys = np.concatenate([pos[:, 1], tgt[:, 1]])
    xlim = (xs.min() - 20, xs.max() + 20)
    ylim = (ys.min() - 20, ys.max() + 20)

    def draw(ax, label, color, k, err_series, note):
        ax.clear()
        _static(ax, case, fly)
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.plot(pos[: k + 1, 0], pos[: k + 1, 1], "-", color=C_FLY, lw=1.3,
                alpha=0.85, zorder=4, label="无人机轨迹")
        ax.plot(tgt[: k + 1, 0], tgt[: k + 1, 1], "-", color=C_TGT, lw=1.0,
                alpha=0.7, zorder=3, label="目标轨迹")
        ax.plot(tgt[k, 0], tgt[k, 1], "o", color=C_TGT, ms=6, zorder=6)
        ax.plot(pos[k, 0], pos[k, 1], "o", color=C_FLY, ms=6, zorder=6)
        ax.plot([tgt[k, 0], pos[k, 0]], [tgt[k, 1], pos[k, 1]], ":",
                color=color, lw=1.0, zorder=5)
        e = float(err_series[k]) if k < len(err_series) else float("nan")
        ax.set_title("%s\n%s，当前偏差 %.1f m" % (label, note, e), fontsize=8.5)

    def update(fi):
        k = frames[fi]
        t_now = times[k]
        draw(axes[0], "目标正上方悬停（朴素）", C_BAD, k, naive_err,
             "转角随目标急转")
        draw(axes[1], "目标正后方 %.0f m standoff" % cfg_standoff, C_OK, k,
             good_err, "偏移方向限速绕行")
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
    fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.4))
    for ax, (name, title, case, fly) in zip(axes, runs):
        _static(ax, case, fly)
        pos = np.asarray(fly.quad_pos)
        ax.plot(pos[:, 0], pos[:, 1], "-", color=C_FLY, lw=1.2, zorder=4,
                label="无人机轨迹")
        # 直线算例 y 方向没有跨度：显式给限幅，否则等比例轴变换奇异
        xs = np.concatenate([pos[:, 0], np.asarray(case.waypoints)[:, 0]])
        ys = np.concatenate([pos[:, 1], np.asarray(case.waypoints)[:, 1]])
        ax.set_xlim(xs.min() - 20, xs.max() + 20)
        ax.set_ylim(ys.min() - 20, ys.max() + 20)
        ax.set_title("%s\n跟随误差 mean %.2f m / 转角 %.2f m"
                     % (title.split("：")[0], fly.mean_err_m, fly.turn_err_m),
                     fontsize=8)
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
        out = os.path.join(OUT_DIR, "%s.gif" % name)
        make_gif(name, title, case, fly, out)
        print("saved", os.path.normpath(out))
    make_overview(runs)


if __name__ == "__main__":
    main()
