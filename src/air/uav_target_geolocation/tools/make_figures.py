#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成文档配图：几何构型与误差、交会角扫描曲线、闭环飞行 GIF。

用法：
    python tools/make_figures.py            # 全部生成
    python tools/make_figures.py --skip-gif # 只出静态图
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve()
_ROOT = _HERE.parents[1]                 # 包目录
for _p in (str(_ROOT), str(_ROOT.parent / "uav_mujoco_sim")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import matplotlib                                   # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                     # noqa: E402
from matplotlib import font_manager                 # noqa: E402
from matplotlib.patches import Circle               # noqa: E402

from uav_target_geolocation.core.analysis import (  # noqa: E402
    fusion_crossing_angle_deg, monte_carlo, sweep_angle)
from uav_target_geolocation.core.cases import (CASES, Station,  # noqa: E402
                                              get_case)
from uav_target_geolocation.core.geometry import los_to          # noqa: E402

OUT = _HERE.parents[4] / "docs" / "img" / "air" / "uav_target_geolocation"
RESULTS = _ROOT / "results"


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
        except Exception:                            # noqa: BLE001
            continue
    plt.rcParams["axes.unicode_minus"] = False


def _load(name: str) -> dict:
    p = RESULTS / f"{name}_result.json"
    if not p.exists():
        raise SystemExit(f"缺少 {p}，先跑 `python main.py`")
    return json.loads(p.read_text(encoding="utf-8"))


def fig_overview() -> Path:
    _setup_font()
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(11.2, 4.6),
                                   gridspec_kw={"width_ratios": [1.15, 1]})

    # ---- 左：俯视图上的三种站（0° 站为三个算例共用，画一次）
    case_style = {"single_station": ("单站", "#7f7f7f", "o"),
                  "two_station_orthogonal": ("双站 90°", "#dd8452", "s"),
                  "two_station_shallow": ("双站 12°", "#55a868", "^")}
    seen = {}
    for name, (label, color, mk) in case_style.items():
        d = _load(name)
        fused = d["algorithm"].get("fused")
        cep_txt = (f"{fused['mean_m']:.2f} m" if fused else
                   f"{d['algorithm']['single']['mean_m']:.2f} m")
        legend_label = f"{label}：{cep_txt}"
        for st in _stations_of(name):
            p = st.position
            key = round(float(np.degrees(np.arctan2(p[1], p[0]))), 1)
            if key not in seen:
                ax0.plot([p[0]], [p[1]], mk, color=color, ms=8, mfc="none",
                         mew=1.6,
                         label=None)
                seen[key] = (p, color)
            ax0.annotate("", xy=(0.0, 0.0), xytext=(p[0], p[1]),
                         arrowprops=dict(arrowstyle="->", color=color,
                                         lw=1.1, alpha=0.8,
                                         shrinkA=7, shrinkB=9))
        ax0.plot([], [], mk, color=color, ms=6, label=legend_label)

    for key, (p, color) in seen.items():
        ax0.annotate(f"{key:g}°", (p[0], p[1]),
                     textcoords="offset points", xytext=(6, 4), fontsize=8)

    ax0.plot([0.0], [0.0], "k*", ms=15, label="目标（真值）")
    ax0.set_aspect("equal")
    ax0.set_xlim(-80, 80)
    ax0.set_ylim(-85, 85)
    ax0.grid(alpha=0.25)
    ax0.set_xlabel("x / m")
    ax0.set_ylabel("y / m")
    ax0.set_title("俯视：观察站构型（全部悬停在 80 m 高）", fontsize=11)
    ax0.legend(loc="lower left", fontsize=8.5, framealpha=0.9)

    # ---- 右：误差对比
    data = _load("single_station")
    names = ["single_station", "two_station_orthogonal", "two_station_shallow"]
    labels = ["单站", "双站 90°", "双站 12°"]
    ana, mea = [], []
    for n in names:
        d = _load(n)
        ana.append(d["algorithm"]["single"]["mean_m"] if n == "single_station"
                   else d["algorithm"]["fused"]["mean_m"])
        cl = d.get("closed_loop") or {}
        mea.append((cl.get("fused") or {}).get("mean_m"))

    level = data["algorithm"]["level_assumption"]["mean_m"]
    xs = np.arange(len(names))
    w = 0.34
    b1 = ax1.bar(xs - w / 2, ana, w, label="解析（蒙特卡洛）", color="#4c72b0")
    b2 = ax1.bar(xs + w / 2, mea, w, label="实测（MuJoCo 闭环）", color="#dd8452")
    ax1.axhline(level, color="#c44e52", ls="--", lw=1.2,
                label=f"假定机体水平：{level:.1f} m")
    ax1.set_xticks(xs)
    ax1.set_xticklabels(labels)
    ax1.set_ylabel("落点误差均值 / m")
    ax1.set_title("同一目标、同一套解算，只换站与几何", fontsize=11)
    ax1.grid(axis="y", alpha=0.25)
    ax1.legend(fontsize=8.5, loc="upper left")
    for bars in (b1, b2):
        for b in bars:
            h = b.get_height()
            if h is not None and np.isfinite(h):
                ax1.annotate(f"{h:.2f}", (b.get_x() + b.get_width() / 2, h),
                             textcoords="offset points", xytext=(0, 3),
                             ha="center", fontsize=8)
    ax1.set_ylim(0, max(max(ana), max(m for m in mea if m or 0)) * 1.6)

    fig.tight_layout()
    OUT.mkdir(parents=True, exist_ok=True)
    p = OUT / "overview.png"
    fig.savefig(p, dpi=150)
    plt.close(fig)
    return p


def _stations_of(name: str):
    return list(get_case(name).stations)


def fig_sweep() -> Path:
    _setup_font()
    angles = [2.0, 3.0, 5.0, 8.0, 12.0, 20.0, 30.0, 45.0, 60.0, 90.0, 120.0]
    # n 要够大：CEP50 在 n=250 时的抽样噪声约 5%，临界角会在 42~53° 之间乱跳，
    # 图上标注跟着漂。n=1500 把噪声压到 2% 以内，标注才稳定在 45° 附近。
    out = sweep_angle(angles, sigma_deg=0.30, bias_deg=0.20, samples=120,
                      n=1500, seed=5)
    base = get_case("single_station")
    # 基线必须与曲线同统计量（CEP50）且足够收敛。之前这里取的是 mean_m 去和
    # CEP50 比，属于跨口径比较，会把临界角整体挪动十几度
    single = monte_carlo(base, n=6000, seed=5)["single"]["cep50_m"]

    fig, ax = plt.subplots(figsize=(7.2, 3.9))
    ax.plot(out["angle_deg"], out["fused_cep50_m"], "o-", color="#4c72b0",
            ms=4, label="交会 CEP50")
    ax.plot(out["angle_deg"], out["fused_cep95_m"], "s--", color="#4c72b0",
            ms=4, alpha=0.7, label="交会 CEP95")
    ax.axhline(single, color="#c44e52", lw=1.3,
               label=f"单站 CEP50 {single:.2f} m")
    ax.set_xscale("log")
    ax.set_xticks(angles)
    ax.set_xticklabels([f"{a:g}" for a in angles], fontsize=8)
    ax.set_xlabel("两站水平开角 / °（对数轴）")
    ax.set_ylabel("落点误差 / m")
    ax.set_title("交会角越小，交会反而越差——几何构型比算法更决定精度", fontsize=11)
    ax.grid(alpha=0.25, which="both")
    ax.legend(fontsize=9)
    # 标出"交会开始吃亏"的临界开角与正文用的 12°。穿越点用插值求：
    # 原来的 next(第一个 c > single) 在升序数组上第一次迭代就命中，永远返回 2°
    cross = fusion_crossing_angle_deg(out["angle_deg"], out["fused_cep50_m"],
                                      single)
    if cross is None:
        cross = max(angles)
    cross_y = float(np.interp(cross, out["angle_deg"], out["fused_cep50_m"]))
    ax.annotate(f"开角小于约 {cross:.0f}° 时交会反而更差",
                (cross, cross_y), textcoords="offset points", xytext=(14, 26),
                fontsize=8.5, color="#c44e52",
                arrowprops=dict(arrowstyle="->", color="#c44e52", lw=0.9))
    ax.plot([cross], [cross_y], "o", color="#c44e52", ms=6, mfc="none", mew=1.5)
    ax.plot([12.0], [float(np.interp(12.0, out["angle_deg"],
                                     out["fused_cep50_m"]))], "o",
            color="#55a868", ms=8, mfc="none", mew=1.8)
    ax.annotate("正文算例 12°", (12.0, float(np.interp(12.0, out["angle_deg"],
                                                      out["fused_cep50_m"]))),
                textcoords="offset points", xytext=(0, 14), fontsize=8.5,
                color="#55a868", ha="center")
    fig.tight_layout()
    OUT.mkdir(parents=True, exist_ok=True)
    p = OUT / "sweep_angle.png"
    fig.savefig(p, dpi=150)
    plt.close(fig)
    return p


def fig_flight_gif(fps: int = 12, max_frames: int = 130) -> Path:
    """真飞一遍双站正交算例，把轨迹录成 GIF。"""
    from matplotlib.animation import FuncAnimation, PillowWriter

    from uav_target_geolocation.core.flyer import fly_case

    _setup_font()
    case = get_case("two_station_orthogonal")
    res = fly_case(case, seed=21)
    traj = np.asarray(res.trajectory)
    if len(traj) == 0:
        raise SystemExit("轨迹为空，无法出图")
    step = max(1, len(traj) // max_frames)
    traj = traj[::step]

    fig = plt.figure(figsize=(5.6, 4.6))
    ax = fig.add_subplot(111, projection="3d")
    xy = traj[:, :2]
    pad = 30.0
    ax.set_xlim(xy[:, 0].min() - pad, xy[:, 0].max() + pad)
    ax.set_ylim(xy[:, 1].min() - pad, xy[:, 1].max() + pad)
    ax.set_zlim(0, 100)
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.set_zlabel("z / m")
    ax.view_init(elev=22, azim=-58)

    gg = np.linspace(-100, 100, 2)
    gx, gy = np.meshgrid(gg, gg)
    ax.plot_surface(gx, gy, np.zeros_like(gx), alpha=0.12, color="#7f7f7f",
                    linewidth=0)

    ax.plot([0], [0], [0], "k*", ms=13, label="目标")
    for i, st in enumerate(case.stations):
        p = st.position
        ax.plot([p[0]], [p[1]], [p[2]], "^", color="#dd8452", ms=7,
                label="观察站" if i == 0 else None)
        ax.plot([p[0], 0.0], [p[1], 0.0], [p[2], 0.0], "--",
                color="#dd8452", lw=0.8, alpha=0.7)

    trail, = ax.plot([], [], [], "-", color="#4c72b0", lw=1.2, alpha=0.85)
    drone, = ax.plot([], [], [], "o", color="#c44e52", ms=6)
    title = ax.set_title("", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)

    def _update(i: int):
        part = traj[: i + 1]
        trail.set_data(part[:, 0], part[:, 1])
        trail.set_3d_properties(part[:, 2])
        drone.set_data([traj[i, 0]], [traj[i, 1]])
        drone.set_3d_properties([traj[i, 2]])
        title.set_text(f"双站 90° 交会：飞行 {res.sim_s * i / max(1, len(traj) - 1):.1f} s"
                       f"（悬停处采样 120 帧）")
        return trail, drone, title

    OUT.mkdir(parents=True, exist_ok=True)
    p = OUT / "flight.gif"
    anim = FuncAnimation(fig, _update, frames=len(traj), interval=1000 / fps,
                         blit=False)
    anim.save(str(p), writer=PillowWriter(fps=fps))
    plt.close(fig)
    return p


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="生成文档配图")
    ap.add_argument("--skip-gif", action="store_true")
    ap.add_argument("--only", default=None,
                    help="只出某一张：overview / sweep / gif")
    args = ap.parse_args(argv)

    made = []
    if args.only in (None, "overview"):
        made.append(fig_overview())
    if args.only in (None, "sweep"):
        made.append(fig_sweep())
    if not args.skip_gif and args.only in (None, "gif"):
        made.append(fig_flight_gif())
    for p in made:
        print(f"[ok] {p}  ({p.stat().st_size / 1024:.0f} KB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
