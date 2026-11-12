#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成文档配图：地形与三条轨迹、前视参考机理、速度扫描、闭环飞行 GIF。

用法：
    python tools/make_figures.py             # 全部生成
    python tools/make_figures.py --skip-gif  # 只出静态图
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve()
_ROOT = _HERE.parents[1]                     # 包目录
for _p in (str(_ROOT), str(_ROOT.parent / "uav_mujoco_sim")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import matplotlib                                # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                  # noqa: E402
from matplotlib import font_manager              # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.patches import Patch             # noqa: E402

from uav_terrain_follow_agl.core.cases import get_case, sweep_cases   # noqa: E402
from uav_terrain_follow_agl.core.dem import (DigitalElevationModel,
                                            IDEAL_DEM)       # noqa: E402
from uav_terrain_follow_agl.core.flyer import fly                       # noqa: E402
from uav_terrain_follow_agl.core.control import DemPreview              # noqa: E402

OUT = _HERE.parents[4] / "docs" / "img" / "air" / "uav_terrain_follow_agl"

COLORS = {"constant_msl": "#c44e52", "reactive_agl": "#dd8452",
          "dem_preview": "#4c72b0"}
LABELS = {"constant_msl": "定高（MSL）", "reactive_agl": "纯反馈（雷达）",
          "dem_preview": "DEM 前馈 + 慢环"}
SLOPE_COLOR = "#8c8c8c"
SAFETY_COLOR = "#937860"


def _setup_font() -> None:
    """显式挑一个装了中文字形的字体，避免方框乱码。"""
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                 "Source Han Sans SC", "PingFang SC"):
        try:
            path = font_manager.findfont(name, fallback_to_default=False)
            if path and os.path.exists(path):
                plt.rcParams["font.sans-serif"] = [name]
                plt.rcParams["axes.unicode_minus"] = False
                return
        except Exception:                        # noqa: BLE001
            continue
    plt.rcParams["axes.unicode_minus"] = False


def _plant() -> str:
    try:
        import mujoco  # noqa: F401
    except Exception:                            # noqa: BLE001
        return "kinematic"
    return "mujoco"


def _runs(case_name: str, plant: str):
    case = get_case(case_name)
    return case, {name: fly(case, name, plant=plant)
                  for name in ("constant_msl", "reactive_agl", "dem_preview")}


def fig_overview(plant: str) -> Path:
    """山脊算例：地形剖面 + 三条高度轨迹 / 三条离地间隙。"""
    _setup_font()
    case, flights = _runs("ridge_8ms", plant)
    terrain = case.terrain
    xs, hs = terrain.profile(0.0, case.length_m, n=1201)

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(11.6, 4.6),
                                   gridspec_kw={"width_ratios": [1.2, 1]})
    ax0.fill_between(xs, -25.0, hs, color=SLOPE_COLOR, alpha=0.35, lw=0,
                     label="地形剖面")
    ax0.plot(xs, hs, color=SLOPE_COLOR, lw=1.2)
    ax0.plot(xs, hs + case.safety_m, color=SAFETY_COLOR, lw=1.0, ls="--",
             label=f"安全间隙 {case.safety_m:g} m")
    for name, f in flights.items():
        ax = np.array([s.x for s in f.samples])
        zz = np.array([s.z for s in f.samples])
        gap = f.metrics()["min_clearance_m"]
        ax0.plot(ax, zz, color=COLORS[name], lw=1.6,
                 label=f"{LABELS[name]}（最小间隙 {gap:.1f} m）")
    ax0.set_xlim(0, case.length_m)
    ax0.set_ylim(-25, 75)
    ax0.set_xlabel("航程 x / m")
    ax0.set_ylabel("高度 / m（ENU）")
    ax0.set_title(f"恒定离地高度 {case.agl_set_m:g} m · 巡航 {case.speed_mps:g} m/s")
    ax0.legend(loc="upper left", fontsize=8, framealpha=0.9)
    ax0.grid(alpha=0.25)

    for name, f in flights.items():
        ax = np.array([s.x for s in f.samples])
        cl = np.array([s.clearance for s in f.samples])
        ax1.plot(ax, cl, color=COLORS[name], lw=1.4, label=LABELS[name])
    ax1.axhline(0.0, color="#333333", lw=1.0)
    ax1.axhline(case.safety_m, color=SAFETY_COLOR, lw=1.0, ls="--")
    ax1.text(4, case.safety_m + 0.8, f"安全间隙 {case.safety_m:g} m",
             color=SAFETY_COLOR, fontsize=8)
    ax1.fill_between([0, case.length_m], -30, 0, color="#c44e52", alpha=0.08)
    ax1.set_xlim(0, case.length_m)
    ax1.set_ylim(-30, 30)
    ax1.set_xlabel("航程 x / m")
    ax1.set_ylabel("离地间隙 h / m")
    ax1.set_title("离地间隙（负值 = 已侵入地形）")
    ax1.legend(loc="lower left", fontsize=8, framealpha=0.9)
    ax1.grid(alpha=0.25)

    fig.tight_layout()
    path = OUT / "overview.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def fig_mechanism(plant: str) -> Path:
    """前视参考为什么能提前起爬：z_ref 与地形的分离。"""
    _setup_font()
    case = get_case("ridge_8ms")
    terrain = case.terrain
    dem = DigitalElevationModel(terrain, IDEAL_DEM)   # 机理图用理想 DEM，排除偏置干扰
    strat = DemPreview(case.limits)
    strat.reset(terrain, dem, 0.0, case.agl_set_m, case.agl_set_m)

    xs = np.arange(140.0, 320.0, 1.0)
    z_ref = np.array([strat.preview_reference(float(x), case.speed_mps) for x in xs])
    h = terrain.elevation(xs, 0.0)

    fig, ax = plt.subplots(figsize=(7.8, 4.3))
    ax.fill_between(xs, 0.0, h, color=SLOPE_COLOR, alpha=0.35, lw=0,
                    label="真实地形")
    ax.plot(xs, h + case.agl_set_m, color="#55a868", lw=1.2, ls=":",
            label=f"理想保高：地形 + {case.agl_set_m:g} m")
    ax.plot(xs, z_ref, color=COLORS["dem_preview"], lw=1.8,
            label="前馈参考 z_ref（可行性下界）")

    x_start = float(xs[np.argmax(z_ref > case.agl_set_m + 0.2)])
    lead = 280.0 - x_start
    ax.axvline(x_start, color="#4c72b0", lw=0.9, ls="--")
    ax.annotate(f"提前 {lead:.0f} m 开始爬\n(≈ A/r_max·v = 30/3·8)",
                xy=(x_start, case.agl_set_m + 1.0),
                xytext=(x_start - 78, case.agl_set_m + 9),
                fontsize=8, color="#2b4b7c",
                arrowprops=dict(arrowstyle="->", color="#2b4b7c", lw=0.9))
    ax.axvline(280.0, color="#8c8c8c", lw=0.8, ls=":")
    ax.set_xlabel("航程 x / m")
    ax.set_ylabel("高度 / m")
    ax.set_title("前馈把「瞬时爬升率」换成「提前量」")
    ax.set_xlim(xs[0], xs[-1])
    ax.set_ylim(0, 60)
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    path = OUT / "mechanism.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def fig_sweep(plant: str) -> Path:
    """速度扫描：最小间隙与 AGL 误差随速度变化。"""
    _setup_font()
    speeds = []
    data = {k: {"gap": [], "rms": []} for k in COLORS}
    for case in sweep_cases():
        speeds.append(case.speed_mps)
        for name in COLORS:
            m = fly(case, name, plant=plant).metrics()
            data[name]["gap"].append(m["min_clearance_m"])
            data[name]["rms"].append(m["rms_agl_error_m"])

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(11.0, 4.3))
    first = get_case("ridge_8ms")
    for name in COLORS:
        ax0.plot(speeds, data[name]["gap"], "o-", color=COLORS[name], lw=1.6,
                 ms=4, label=LABELS[name])
    ax0.axhline(0.0, color="#333333", lw=1.0)
    ax0.axhline(first.safety_m, color=SAFETY_COLOR, lw=1.0, ls="--",
                label=f"安全间隙 {first.safety_m:g} m")
    ax0.set_xlabel("巡航速度 / (m/s)")
    ax0.set_ylabel("最小离地间隙 / m")
    ax0.set_title("山脊地形上的最小间隙")
    ax0.set_ylim(-20, 20)
    ax0.legend(fontsize=8)
    ax0.grid(alpha=0.25)

    ax2 = ax1.twinx()
    ax2.plot(speeds, [get_case("ridge_8ms").terrain.required_climb_rate(v, 0.0, 600.0)
                      for v in speeds], color="#8172b3", lw=1.0, ls=":")
    ax2.axhline(first.limits.climb_rate_mps, color="#8172b3", lw=0.9, ls="--")
    ax2.set_ylabel("精确保高所需爬升率 / (m/s)", color="#8172b3", fontsize=9)
    ax2.tick_params(axis="y", colors="#8172b3", labelsize=8)
    ax2.text(speeds[1], first.limits.climb_rate_mps + 0.15,
             f"爬升率上限 {first.limits.climb_rate_mps:g} m/s",
             color="#8172b3", fontsize=8)
    for name in COLORS:
        ax1.plot(speeds, data[name]["rms"], "s-", color=COLORS[name], lw=1.2,
                 ms=3.5, label=LABELS[name])
    ax1.set_xlabel("巡航速度 / (m/s)")
    ax1.set_ylabel("离地高度 RMS 误差 / m")
    ax1.set_title("离地高度跟随误差")
    ax1.grid(alpha=0.25)

    fig.tight_layout()
    path = OUT / "sweep.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def fig_flight_gif(plant: str, fps: int = 12) -> Path:
    """侧视图动画：地形、三条轨迹的当前位置与离地间隙。"""
    _setup_font()
    case, flights = _runs("ridge_8ms", plant)
    terrain = case.terrain
    xs, hs = terrain.profile(0.0, case.length_m, n=800)
    stride = 40
    seq = {name: f.samples[::stride] for name, f in flights.items()}
    n = min(len(v) for v in seq.values())

    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(8.0, 5.6), height_ratios=[2.1, 1],
                                  sharex=True)
    ax.fill_between(xs, -20.0, hs, color=SLOPE_COLOR, alpha=0.35, lw=0)
    ax.plot(xs, hs, color=SLOPE_COLOR, lw=1.1, label="地形")
    ax.plot(xs, hs + case.safety_m, color=SAFETY_COLOR, lw=0.9, ls="--")
    ax.set_ylim(-20, 70)
    ax.set_xlim(0, case.length_m)
    ax.set_ylabel("高度 / m")
    ax.set_title(f"地形跟随侧视（{case.speed_mps:g} m/s，AGL {case.agl_set_m:g} m）")
    ax.grid(alpha=0.2)
    ax.legend(loc="upper left", fontsize=8)

    marks = {}
    trails = {}
    for name in COLORS:
        marks[name], = ax.plot([], [], "o", color=COLORS[name], ms=7,
                               label=LABELS[name])
        trails[name], = ax.plot([], [], color=COLORS[name], lw=1.3, alpha=0.85)
    ax.legend(loc="upper left", fontsize=8)

    texts = {name: ax2.text(0.02, 0.72 - 0.26 * i, "", fontsize=8,
                            color=COLORS[name], transform=ax2.transAxes)
             for i, name in enumerate(COLORS)}
    for name in COLORS:
        ax2.plot([s.x for s in seq[name]], [s.agl_true for s in seq[name]],
                 color=COLORS[name], lw=1.0, alpha=0.35)
    ax2.axhline(case.agl_set_m, color="#55a868", lw=0.9, ls=":")
    ax2.axhline(case.safety_m, color=SAFETY_COLOR, lw=0.9, ls="--")
    ax2.set_ylim(-6, 30)
    ax2.set_xlim(0, case.length_m)
    ax2.set_xlabel("航程 x / m")
    ax2.set_ylabel("离地高度 / m")
    ax2.grid(alpha=0.2)

    def update(i: int):
        for name in COLORS:
            xs_i = [s.x for s in seq[name][: i + 1]]
            zs_i = [s.z for s in seq[name][: i + 1]]
            marks[name].set_data([seq[name][i].x], [seq[name][i].z])
            trails[name].set_data(xs_i, zs_i)
            texts[name].set_text(
                f"{LABELS[name]}：x={seq[name][i].x:5.0f} m  "
                f"间隙={seq[name][i].clearance:5.1f} m")
        return list(marks.values()) + list(trails.values()) + list(texts.values())

    anim = FuncAnimation(fig, update, frames=n, interval=1000 / fps, blit=False)
    path = OUT / "flight.gif"
    anim.save(path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="生成地形跟随配图")
    parser.add_argument("--skip-gif", action="store_true", help="不生成 GIF")
    args = parser.parse_args(argv)

    OUT.mkdir(parents=True, exist_ok=True)
    plant = _plant()
    print(f"被控对象：{plant}（无 mujoco 时自动退到运动学模型）")
    for fn in (fig_overview, fig_mechanism, fig_sweep):
        p = fn(plant)
        print(f"  {p.relative_to(OUT.parents[3])}")
    if not args.skip_gif:
        p = fig_flight_gif(plant)
        print(f"  {p.relative_to(OUT.parents[3])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
