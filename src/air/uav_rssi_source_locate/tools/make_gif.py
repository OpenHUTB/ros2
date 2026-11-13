#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跑一遍四个算例，把闭环搜索定位过程录成 gif。

左栏是俯视实飞轨迹：灰色虚线是扩张方形标称航线，蓝线是 MuJoCo 积分
出来的真实轨迹，小圆点是每次"到点即采样"的测量位置，橙色星是真信标
（算法不知道），绿叉是当时的最小二乘估计；右栏是定位误差随时间的收
敛曲线，虚线是"取最强信号航点"朴素基线的误差水平。

用法（在本目录下）：python tools/make_gif.py
四个算例都会重跑一遍（本机约十几秒），然后重录五张图。
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
                       "uav_rssi_source_locate")
C_FLY = "#1f77b4"
C_OK = "#2ca02c"
C_BAD = "#d62728"
C_BEACON = "#ff7f0e"
C_GREY = "#7f7f7f"

CASES = [("center", "区内中心"),
         ("southeast", "东南角"),
         ("northwest", "西北角（镜像歧义）"),
         ("east_edge", "东北边（贴界）")]


def run_case(name):
    from uav_rssi_source_locate.core.cases import make_scenario
    from uav_rssi_source_locate.core.flyer import Beacon, locate_beacon
    from uav_rssi_source_locate.core.rssi import RssiCfg

    sc = make_scenario(name)
    fly = locate_beacon(Beacon(sc.beacon_xy), sc.cfg, RssiCfg(),
                        record_every=5)
    return sc, fly


def naive_level(sc):
    """朴素基线（取最强信号航点）的误差水平，画成参考虚线。"""
    from uav_rssi_source_locate.core.locator import best_waypoint, expanding_square
    from uav_rssi_source_locate.core.rssi import RssiCfg

    rc = RssiCfg()
    bx, by = sc.beacon_xy
    cfg = sc.cfg

    def rssi_fn(x, y):
        return rc.rssi_at(float(np.linalg.norm(
            np.array([x, y, cfg.alt_m]) - np.array([bx, by, 0.5]))))

    est, _peak = best_waypoint(expanding_square(cfg), rssi_fn)
    return float(np.linalg.norm(est - np.asarray([bx, by])))


def _static_map(ax, sc):
    area = sc.cfg.area
    ax.add_patch(plt.Rectangle((0, 0), area[0], area[1], fill=False,
                               color="#333333", lw=0.9, zorder=2))
    pts = np.asarray(_square(sc), float)
    ax.plot(pts[:, 0], pts[:, 1], "--", color=C_GREY, lw=0.8, alpha=0.8,
            zorder=2, label="标称搜索航线")
    ax.set_xlim(-8, area[0] + 8)
    ax.set_ylim(-8, area[1] + 8)
    ax.set_aspect("equal")
    ax.grid(alpha=0.22, lw=0.5)
    ax.tick_params(labelsize=7)
    ax.set_xlabel("x / m", fontsize=8)
    ax.set_ylabel("y / m", fontsize=8)


def _square(sc):
    from uav_rssi_source_locate.core.locator import expanding_square
    return expanding_square(sc.cfg)


def make_gif(name, title, sc, fly, naive, out):
    times = np.asarray(fly.times, float)
    pos = np.asarray(fly.pos, float)
    meas = np.asarray(fly.measure_points, float) if fly.measure_points \
        else np.zeros((0, 2))
    bx, by = fly.beacon_xy
    est_t = np.asarray(fly.est_t, float)
    est_xy = np.asarray(fly.est_xy, float) if fly.est_xy else np.zeros((0, 2))
    err = np.linalg.norm(est_xy - np.asarray([bx, by]), axis=1) \
        if len(est_xy) else np.zeros(0)

    frames = np.linspace(0, len(times) - 1, min(len(times), 60)).astype(int)
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(8.8, 3.8),
                                   gridspec_kw=dict(width_ratios=[1.15, 1.0]))

    def update(fi):
        k = frames[fi]
        t_now = times[k]
        ax0.clear()
        _static_map(ax0, sc)
        ax0.plot(pos[: k + 1, 0], pos[: k + 1, 1], "-", color=C_FLY, lw=1.4,
                 zorder=4, label="实飞轨迹")
        # 已经飞过的测量段：按时间比例露出（近似，采样点无时间戳）
        n_show = int(len(meas) * (k + 1) / len(times))
        if n_show:
            ax0.plot(meas[:n_show, 0], meas[:n_show, 1], "o", color=C_FLY,
                     ms=2.2, alpha=0.55, zorder=5, label="测量点")
        ax0.plot(bx, by, "*", color=C_BEACON, ms=13, zorder=7,
                 label="真信标（未知）")
        ax0.plot(pos[k, 0], pos[k, 1], "o", color=C_FLY, ms=5.5, zorder=8)
        shown = np.nonzero(est_t <= t_now)[0] if len(est_t) else []
        if len(shown):
            i = shown[-1]
            ax0.plot(est_xy[i, 0], est_xy[i, 1], "x", color=C_OK, ms=9,
                     mew=2.0, zorder=9, label="当前估计")
            ax0.plot([est_xy[i, 0], bx], [est_xy[i, 1], by], ":",
                     color=C_OK, lw=0.9, zorder=6)
            ax0.set_title("%s      t = %.0f s，误差 %.2f m"
                          % (title, t_now, err[i]), fontsize=9)
        else:
            ax0.set_title("%s      t = %.0f s，尚未检测到信号"
                          % (title, t_now), fontsize=9)
        ax0.legend(fontsize=6.5, loc="upper right")

        ax1.clear()
        m = est_t <= t_now
        ax1.axhline(naive, ls="--", color=C_BAD, lw=1.0,
                    label="朴素基线（最强航点）%.0f m" % naive)
        if np.any(m):
            ax1.step(est_t[m], np.maximum(err[m], 1e-2), where="post",
                     color=C_OK, lw=1.4, label="三边定位误差")
            # 每轮更新的点
            ax1.plot(est_t[m], np.maximum(err[m], 1e-2), "o", color=C_OK,
                     ms=3.0)
        ax1.set_yscale("log")
        ax1.set_xlim(0, times[-1])
        ax1.set_ylim(1e-2, max(naive * 3.0, 10.0))
        ax1.set_xlabel("t / s", fontsize=8)
        ax1.set_ylabel("与真信标的水平误差 / m", fontsize=8)
        ax1.grid(alpha=0.25, lw=0.5, which="both")
        ax1.tick_params(labelsize=7)
        ax1.legend(fontsize=6.5, loc="upper right")
        fig.tight_layout()
        return []

    anim = FuncAnimation(fig, update, frames=len(frames), interval=170,
                         blit=False)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    anim.save(out, writer=PillowWriter(fps=6))
    plt.close(fig)
    return out


def make_overview(runs):
    fig, axes = plt.subplots(1, 4, figsize=(12.6, 3.5))
    for ax, (name, title, sc, fly, naive) in zip(axes, runs):
        _static_map(ax, sc)
        pos = np.asarray(fly.pos, float)
        ax.plot(pos[:, 0], pos[:, 1], "-", color=C_FLY, lw=1.0, zorder=4,
                label="实飞轨迹")
        meas = np.asarray(fly.measure_points, float) if fly.measure_points \
            else np.zeros((0, 2))
        if len(meas):
            ax.plot(meas[:, 0], meas[:, 1], "o", color=C_FLY, ms=1.8,
                    alpha=0.5, zorder=5, label="测量点")
        bx, by = fly.beacon_xy
        ax.plot(bx, by, "*", color=C_BEACON, ms=11, zorder=7)
        if fly.estimate is not None:
            ax.plot(fly.estimate[0], fly.estimate[1], "x", color=C_OK, ms=8,
                    mew=2.0, zorder=8)
        ax.set_title("%s\n误差 %.2f m / 朴素 %.0f m / %.0f s"
                     % (title.split("（")[0], fly.error_m, naive, fly.sim_s),
                     fontsize=8)
    axes[0].legend(fontsize=6.0, loc="upper right")
    fig.tight_layout()
    out = os.path.join(OUT_DIR, "overview.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print("saved", os.path.normpath(out))


def main():
    runs = []
    for name, title in CASES:
        sc, fly = run_case(name)
        naive = naive_level(sc)
        runs.append((name, title, sc, fly, naive))
        out = os.path.join(OUT_DIR, "%s.gif" % name)
        make_gif(name, title, sc, fly, naive, out)
        print("saved", os.path.normpath(out),
              "err %.2f m" % fly.error_m)
    make_overview(runs)


if __name__ == "__main__":
    main()
