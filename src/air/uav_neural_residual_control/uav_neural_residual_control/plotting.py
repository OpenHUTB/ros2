#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""文档配图：由实验结果 JSON / 数据集直接生成，不手绘。

中文字体在 Linux 上大概率缺失，:func:`use_cjk_font` 会在找不到时退回默认
字体并给出提示——图还能出，只是中文变成方框。
"""
from __future__ import annotations

import json
import os
from typing import Dict, List, Optional, Sequence

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

__all__ = ["use_cjk_font", "plot_disturbances", "plot_training", "plot_tracking",
           "plot_payload_sweep", "plot_rls_adaptation", "plot_wind_rejection",
           "make_flight_gif", "make_all"]

C = {"pid": "#d95f02", "geo": "#7570b3", "geo_res": "#1b9e77", "geo_rls": "#e7298a"}
LABEL = {"pid": "位置 PID", "geo": "几何控制 (SE3)",
         "geo_res": "几何 + 残差网络", "geo_rls": "几何 + 残差 + 在线 RLS"}
PL, WL = "#4c72b0", "#c44e52"


def use_cjk_font() -> bool:
    """配置中文字体，返回是否找到。"""
    from matplotlib import font_manager
    candidates = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                  "Source Han Sans SC", "WenQuanYi Zen Hei", "PingFang SC"]
    have = {f.name for f in font_manager.fontManager.ttflist}
    for c in candidates:
        if c in have:
            plt.rcParams["font.sans-serif"] = [c] + plt.rcParams["font.sans-serif"]
            plt.rcParams["axes.unicode_minus"] = False
            return True
    return False


def _save(fig, path: str, dpi: int = 150) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print("  图 → %s" % path)
    return path


# --------------------------------------------------------------------------- #
def plot_disturbances(outdir: str, seed: int = 0) -> str:
    """未建模效应的物理画像：阻力曲线 / 风场时程 / 载荷残差。"""
    from .core.disturbances import DragModel, WindModel, PAYLOAD_LEVELS, GRAVITY

    fig, ax = plt.subplots(1, 3, figsize=(13.5, 3.6))
    v = np.linspace(0, 12, 400)
    d = DragModel()
    fx = np.array([-d.body_force(np.array([vv, 0.0, 0.0]))[0] for vv in v])
    fz = np.array([-d.body_force(np.array([0.0, 0.0, vv]))[0] for vv in v])
    w = 0.95 * GRAVITY
    ax[0].plot(v, fx, color="#1b9e77", label="水平（机体系 x）")
    ax[0].plot(v, fz, color="#d95f02", ls="--", label="垂直（机体系 z）")
    ax[0].axhline(w * np.tan(np.deg2rad(35)), color="k", lw=0.8, ls=":",
                  label="35° 倾角可用水平推力")
    ax[0].set_xlabel("相对气流速度 (m/s)")
    ax[0].set_ylabel("阻力 (N)")
    ax[0].set_title("(a) 二次气动阻力")
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)

    wm = WindModel(mean=(3.2, -1.1, 0.0), gust=1.8, seed=seed)
    t = np.arange(0, 20.0, 0.01)
    hist = []
    wm.reset()
    for _ in t:
        wm.update(0.01)
        hist.append(wm.vel.copy())
    hist = np.asarray(hist)
    ax[1].plot(t, np.linalg.norm(hist[:, :2], axis=1), color=PL)
    ax[1].axhline(3.2, color="k", lw=0.8, ls="--", label="平均风 3.2 m/s")
    ax[1].axhline(3.2 + 1.8, color="gray", lw=0.8, ls=":",
                  label=r"$\bar{w}+\sigma$")
    ax[1].set_xlabel("时间 (s)")
    ax[1].set_ylabel("水平风速 (m/s)")
    ax[1].set_title("(b) 阵风（一阶 Gauss-Markov）")
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)

    names = list(PAYLOAD_LEVELS)
    vals = [PAYLOAD_LEVELS[n] * GRAVITY / 0.95 for n in names]
    bars = ax[2].bar(names, vals, color=["#999999", "#66c2a5", "#fc8d62", "#8da0cb"])
    for b, val in zip(bars, vals):
        ax[2].text(b.get_x() + b.get_width() / 2, val + 0.12, "%.2f" % val,
                   ha="center", fontsize=9)
    ax[2].set_ylabel(r"垂直残差 $|\Delta a_z|$ (m/s²)")
    ax[2].set_title("(c) 载荷引起的模型偏差")
    ax[2].set_ylim(0, max(vals) * 1.25)
    ax[2].grid(alpha=0.3, axis="y")
    return _save(fig, os.path.join(outdir, "01_disturbances.png"))


def plot_training(hist: Dict[str, List[float]], ds, net, outdir: str) -> str:
    """训练曲线 + 残差预测散点：网络到底学到了什么。"""
    fig, ax = plt.subplots(1, 3, figsize=(13.5, 3.8))
    ax[0].plot(np.sqrt(np.asarray(hist["train"])), label="训练", color=PL)
    ax[0].plot(np.sqrt(np.asarray(hist["val"])), label="验证", color=WL)
    ax[0].axvline(hist.get("best_epoch", 0), color="k", lw=0.8, ls=":")
    ax[0].set_xlabel("epoch")
    ax[0].set_ylabel(r"RMSE (m/s²)")
    ax[0].set_yscale("log")
    ax[0].set_title("(a) 训练收敛")
    ax[0].legend(fontsize=9)
    ax[0].grid(alpha=0.3, which="both")

    idx = np.random.default_rng(0).choice(len(ds), size=min(4000, len(ds)), replace=False)
    Yh = net.predict_batch(ds.X[idx])
    Y = ds.Y[idx]
    names = ["$x_b$", "$y_b$", "$z_b$"]
    for i in range(3):
        ax[1].scatter(Y[:, i], Yh[:, i], s=2, alpha=0.18, color=C["geo_res"])
    lim = float(np.percentile(np.abs(Y), 99.5)) * 1.15
    ax[1].plot([-lim, lim], [-lim, lim], "k--", lw=0.9)
    ax[1].set_xlim(-lim, lim)
    ax[1].set_ylim(-lim, lim)
    ax[1].set_xlabel(r"真值 $\Delta a$ (m/s²)")
    ax[1].set_ylabel(r"预测 $\hat{\Delta a}$ (m/s²)")
    ax[1].set_title(r"(b) 残差预测（机体系，三轴叠加）")
    ax[1].grid(alpha=0.3)

    err = Yh - Y
    ax[2].hist(np.linalg.norm(err, axis=1), bins=60, color=C["geo_res"], alpha=0.85)
    r = float(np.sqrt(np.mean(err ** 2)))
    ax[2].axvline(r, color="k", ls="--", lw=1.0, label="RMSE = %.3f m/s²" % r)
    ax[2].set_xlabel(r"预测误差 $|\hat{\Delta a}-\Delta a|$ (m/s²)")
    ax[2].set_ylabel("样本数")
    ax[2].set_title("(c) 预测误差分布")
    ax[2].legend(fontsize=9)
    ax[2].grid(alpha=0.3)
    return _save(fig, os.path.join(outdir, "02_residual_learning.png"))


def plot_tracking(records, outdir: str, payload: str = "heavy", wind: str = "windy",
                  traj: str = "helix") -> str:
    """同一工况下四个控制器的轨迹与误差对比。"""
    sel = [r for r in records if r.payload == payload and r.wind == wind
           and r.traj == traj and r.series]
    if not sel:
        print("  [跳过] 没有 %s/%s/%s 的带序列结果" % (payload, wind, traj))
        return ""
    fig = plt.figure(figsize=(13.5, 7.6))
    gs = fig.add_gridspec(2, 3, height_ratios=[1.25, 1.0])

    ax = fig.add_subplot(gs[0, 0])
    ref = np.asarray(sel[0].series["ref"])
    ax.plot(ref[:, 0], ref[:, 1], "k--", lw=1.6, label="参考轨迹", zorder=1)
    for r in sorted(sel, key=lambda x: x.controller):
        p = np.asarray(r.series["pos"])
        ax.plot(p[:, 0], p[:, 1], color=C[r.controller], lw=1.2,
                label=LABEL[r.controller], alpha=0.9)
    ax.set_aspect("equal")
    ax.set_xlabel("x 东 (m)")
    ax.set_ylabel("y 北 (m)")
    ax.set_title("(a) 水平轨迹（%s, %s+%s 风）" % (traj, payload, wind))
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(gs[0, 1])
    for r in sorted(sel, key=lambda x: x.controller):
        p = np.asarray(r.series["pos"])
        t = np.asarray(r.series["t"])
        ax.plot(t, p[:, 2], color=C[r.controller], lw=1.0, label=LABEL[r.controller])
    ax.plot(np.asarray(sel[0].series["t"]), ref[:, 2], "k--", lw=1.4, label="参考高度")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("z 高度 (m)")
    ax.set_title("(b) 高度跟踪")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(gs[0, 2])
    for r in sorted(sel, key=lambda x: x.controller):
        t = np.asarray(r.series["t"])
        ax.semilogy(t, np.asarray(r.series["err"]), color=C[r.controller], lw=1.0,
                    label="%s RMSE=%.3f m" % (LABEL[r.controller], r.metrics["rmse"]))
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("位置误差 (m)")
    ax.set_title("(c) 位置误差时程")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, which="both")

    # 残差通道
    rr = [r for r in sel if r.controller in ("geo_res", "geo_rls")]
    ax = fig.add_subplot(gs[1, 0])
    if rr:
        r0 = [r for r in rr if r.controller == "geo_res"][0]
        t = np.asarray(r0.series["t"])
        dt_ = np.asarray(r0.series["delta_true"])
        ax.plot(t, dt_[:, 2], color="k", lw=1.2, label=r"真值 $\Delta a_z$")
        for r in sorted(rr, key=lambda x: x.controller):
            dh = np.asarray(r.series["delta_hat"])
            ax.plot(t, dh[:, 2], color=C[r.controller], lw=1.0,
                    label="预测 " + LABEL[r.controller])
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel(r"$\Delta a_z$ (m/s²)")
    ax.set_title("(d) 垂直残差：真值 vs 预测")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(gs[1, 1])
    for r in sorted(sel, key=lambda x: x.controller):
        t = np.asarray(r.series["t"])
        ax.plot(t, np.asarray(r.series["thrust"]), color=C[r.controller], lw=1.0,
                label=LABEL[r.controller])
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("总推力 (N)")
    ax.set_title("(e) 控制代价")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(gs[1, 2])
    names = sorted({r.controller for r in sel})
    vals = [next(r.metrics["rmse"] for r in sel if r.controller == n) for n in names]
    ax.bar(range(len(names)), vals, color=[C[n] for n in names])
    for i, v in enumerate(vals):
        ax.text(i, v + max(vals) * 0.02, "%.3f" % v, ha="center", fontsize=9)
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels([LABEL[n] for n in names], rotation=18, ha="right", fontsize=8)
    ax.set_ylabel("稳态 RMSE (m)")
    ax.set_title("(f) 稳态跟踪精度")
    ax.grid(alpha=0.3, axis="y")
    return _save(fig, os.path.join(outdir, "03_tracking_compare.png"))


def plot_payload_sweep(records, outdir: str, traj: str = "helix",
                       wind: str = "windy") -> str:
    """误差随载荷的变化：静态静差 vs 在线辨识。"""
    from .core.disturbances import PAYLOAD_LEVELS
    names = [p for p in PAYLOAD_LEVELS if any(
        r.payload == p and r.traj == traj and r.wind == wind for r in records)]
    fig, ax = plt.subplots(1, 2, figsize=(11.5, 3.9))
    x = np.arange(len(names))
    for ctl in ("pid", "geo", "geo_res", "geo_rls"):
        y = []
        for p in names:
            v = [r.metrics["rmse"] for r in records
                 if r.controller == ctl and r.payload == p and r.traj == traj
                 and r.wind == wind]
            y.append(float(np.mean(v)) if v else np.nan)
        ax[0].plot(x, y, "o-", color=C[ctl], label=LABEL[ctl], lw=1.4, ms=5)
    ax[0].axvspan(2.5, len(names) - 0.5, color="gray", alpha=0.12)
    ax[0].annotate("训练集外", xy=(0.86, 0.93), xycoords="axes fraction",
                   ha="center", fontsize=8, color="gray")
    ax[0].set_xticks(x)
    ax[0].set_xticklabels(["%s\n(%.2f kg)" % (p, PAYLOAD_LEVELS[p]) for p in names],
                          fontsize=9)
    ax[0].set_ylabel("稳态 RMSE (m)")
    ax[0].set_yscale("log")
    ax[0].set_title("(a) 跟踪精度 vs 挂载载荷（%s, %s 风）" % (traj, wind))
    ax[0].legend(fontsize=9)
    ax[0].grid(alpha=0.3, which="both")

    for ctl in ("geo_res", "geo_rls"):
        y = []
        for p in names:
            v = [r.metrics["res_bias"] for r in records
                 if r.controller == ctl and r.payload == p and r.traj == traj
                 and r.wind == wind]
            y.append(float(np.mean(v)) if v else np.nan)
        ax[1].plot(x, y, "o-", color=C[ctl], label=LABEL[ctl], lw=1.4, ms=5)
    ax[1].axvspan(2.5, len(names) - 0.5, color="gray", alpha=0.12)
    ax[1].set_xticks(x)
    ax[1].set_xticklabels(names, fontsize=9)
    ax[1].set_ylabel(r"残差估计偏差 $\|\overline{\hat{\Delta a}-\Delta a}\|$ (m/s²)")
    ax[1].set_title("(b) 残差估计的常值偏差")
    ax[1].legend(fontsize=9)
    ax[1].grid(alpha=0.3)
    return _save(fig, os.path.join(outdir, "04_payload_sweep.png"))


def plot_wind_rejection(records, outdir: str, payload: str = "heavy",
                        traj: str = "helix") -> str:
    """风况扫描：反馈抗扰 vs 前馈补偿。"""
    from .core.disturbances import WIND_LEVELS
    names = [w for w in WIND_LEVELS if any(
        r.wind == w and r.traj == traj and r.payload == payload for r in records)]
    fig, ax = plt.subplots(1, 2, figsize=(11.5, 3.9))
    x = np.arange(len(names))
    for ctl in ("pid", "geo", "geo_res", "geo_rls"):
        y = []
        for w in names:
            v = [r.metrics["rmse"] for r in records
                 if r.controller == ctl and r.payload == payload and r.traj == traj
                 and r.wind == w]
            y.append(float(np.mean(v)) if v else np.nan)
        ax[0].plot(x, y, "o-", color=C[ctl], label=LABEL[ctl], lw=1.4, ms=5)
    ax[0].set_xticks(x)
    ax[0].set_xticklabels(["%s\n%.1f m/s" % (w, float(np.linalg.norm(
        np.asarray(WIND_LEVELS[w]["mean"], float)))) for w in names], fontsize=9)
    ax[0].set_ylabel("稳态 RMSE (m)")
    ax[0].set_title("(a) 抗风性能（%s, %s）" % (traj, payload))
    ax[0].legend(fontsize=9)
    ax[0].grid(alpha=0.3)

    sel = [r for r in records if r.traj == traj and r.payload == payload
           and r.wind == names[-1] and r.series]
    if sel:
        r0 = sel[0]
        t = np.asarray(r0.series["t"])
        ax[1].plot(t, np.linalg.norm(np.asarray(r0.series["delta_true"])[:, :2], axis=1),
                   color="k", lw=1.1, label=r"真值 $\|\Delta a_{xy}\|$")
        for r in sorted([s for s in sel if s.controller in ("geo_res", "geo_rls")],
                        key=lambda s: s.controller):
            dh = np.asarray(r.series["delta_hat"])
            ax[1].plot(t, np.linalg.norm(dh[:, :2], axis=1), color=C[r.controller],
                       lw=1.0, label="预测 " + LABEL[r.controller])
        ax[1].set_ylabel(r"水平残差 (m/s²)")
        ax[1].set_xlabel("时间 (s)")
        ax[1].set_title("(b) 气动残差的在线估计")
        ax[1].legend(fontsize=9)
        ax[1].grid(alpha=0.3)
    return _save(fig, os.path.join(outdir, "05_wind_rejection.png"))


def plot_rls_adaptation(records, outdir: str, payload: str = "extra",
                        wind: str = "breeze", traj: str = "helix") -> str:
    """在线自适应过程：RLS 如何在外推档位上把偏差拉回来。"""
    sel = [r for r in records if r.payload == payload and r.wind == wind
           and r.traj == traj and r.controller in ("geo_res", "geo_rls") and r.series]
    if len(sel) < 2:
        print("  [跳过] 缺少 %s/%s/%s 的残差序列" % (payload, wind, traj))
        return ""
    fig, ax = plt.subplots(1, 2, figsize=(11.5, 3.9))
    r0 = next(r for r in sel if r.controller == "geo_res")
    t = np.asarray(r0.series["t"])
    dt_ = np.asarray(r0.series["delta_true"])
    ax[0].plot(t, dt_[:, 2], color="k", lw=1.2, label=r"真值 $\Delta a_z$")
    for r in sorted(sel, key=lambda x: x.controller):
        ax[0].plot(t, np.asarray(r.series["delta_hat"])[:, 2], color=C[r.controller],
                   lw=1.1, label=LABEL[r.controller])
    ax[0].set_xlabel("时间 (s)")
    ax[0].set_ylabel(r"$\Delta a_z$ (m/s²)")
    ax[0].set_title("(a) 未见载荷 %s（+0.75 kg）下的在线辨识" % payload)
    ax[0].legend(fontsize=9)
    ax[0].grid(alpha=0.3)

    for r in sorted(sel, key=lambda x: x.controller):
        ax[1].plot(np.asarray(r.series["t"]), np.asarray(r.series["err"]),
                   color=C[r.controller], lw=1.0,
                   label="%s RMSE=%.3f m" % (LABEL[r.controller], r.metrics["rmse"]))
    ax[1].set_xlabel("时间 (s)")
    ax[1].set_ylabel("位置误差 (m)")
    ax[1].set_yscale("log")
    ax[1].set_title("(b) 对应的跟踪误差")
    ax[1].legend(fontsize=9)
    ax[1].grid(alpha=0.3, which="both")
    return _save(fig, os.path.join(outdir, "06_rls_adaptation.png"))


# --------------------------------------------------------------------------- #
def make_flight_gif(outdir: str, net_path: str, payload: str = "heavy",
                    wind: str = "windy", traj: str = "helix", fps: int = 20,
                    stride: int = 6, name: str = "07_flight.gif") -> str:
    """四控制器同场飞行的三维动画（俯视 + 侧视），用于文档演示。"""
    from matplotlib.animation import FuncAnimation, PillowWriter
    from .core import ResidualNet, make_tracker, ResidualPlant, make_trajectory
    from .core.disturbances import make_plant

    net = ResidualNet.load(net_path)
    tj = make_trajectory(traj)
    runs = {}
    for ctl in ("geo", "geo_res", "geo_rls"):
        pl = ResidualPlant(plant=make_plant(payload, wind, seed=4), scene="open_ground")
        tk = make_tracker(ctl, pl, tj, net=net)
        p0, _, _, y0, _ = tj.eval(0.0)
        pl.reset(pos=tuple(p0), yaw=float(y0))
        pos, dlt = [], []
        for k in range(int(tj.duration / pl.dt)):
            u = tk.act(k * pl.dt)
            pl.step_with_control(u)
            tk.observe()
            pos.append(pl.state().pos.copy())
            dlt.append(tk.last_residual_b[2])
        runs[ctl] = (np.asarray(pos), np.asarray(dlt))

    ref = np.asarray([tj.eval(k * 0.01)[0] for k in range(int(tj.duration / 0.01))])
    n = min(len(ref), *(len(v[0]) for v in runs.values()))
    frames = list(range(0, n, stride))

    fig = plt.figure(figsize=(11.0, 4.2))
    ax1 = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2)
    ax1.set_aspect("equal")
    ax1.set_xlim(ref[:, 0].min() - 1, ref[:, 0].max() + 1)
    ax1.set_ylim(ref[:, 1].min() - 1, ref[:, 1].max() + 1)
    ax1.plot(ref[:, 0], ref[:, 1], "k--", lw=1.2, label="参考轨迹")
    lines1 = {c: ax1.plot([], [], color=C[c], lw=2.0, label=LABEL[c])[0] for c in runs}
    ax1.set_xlabel("x 东 (m)")
    ax1.set_ylabel("y 北 (m)")
    ax1.legend(fontsize=8, loc="upper right")
    ax1.grid(alpha=0.3)
    ax1.set_title("俯视")

    ax2.set_xlim(ref[:, 2].min() - 0.3, ref[:, 2].max() + 0.3)
    ax2.set_ylim(-8.0, 1.5)
    ax2.plot(ref[:, 2], np.zeros(len(ref)), "k--", lw=1.2, label="参考高度")
    lines2 = {c: ax2.plot([], [], color=C[c], lw=2.0, label=LABEL[c])[0] for c in runs}
    ax2.set_xlabel("z 高度 (m)")
    ax2.set_ylabel(r"$\hat{\Delta a_z}$ (m/s²)")
    ax2.legend(fontsize=8, loc="lower left")
    ax2.grid(alpha=0.3)
    ax2.set_title("残差补偿")
    ttl = fig.suptitle("")

    def upd(i):
        for c, ln in lines1.items():
            ln.set_data(runs[c][0][:i + 1, 0], runs[c][0][:i + 1, 1])
        for c, ln in lines2.items():
            ln.set_data(runs[c][0][:i + 1, 2], runs[c][1][:i + 1])
        ttl.set_text("t = %.1f s   载荷 %s (+%.2f kg) / 风 %s"
                     % (i * 0.01, payload, 0.0 if payload == "none" else
                        {"light": 0.25, "heavy": 0.5, "extra": 0.75}.get(payload, 0.0),
                        wind))
        return list(lines1.values()) + list(lines2.values()) + [ttl]

    anim = FuncAnimation(fig, upd, frames=frames, interval=1000 / fps, blit=False)
    path = os.path.join(outdir, name)
    os.makedirs(outdir, exist_ok=True)
    anim.save(path, writer=PillowWriter(fps=fps), dpi=90)
    plt.close(fig)
    size = os.path.getsize(path) / 1024
    print("  动图 → %s（%.0f KB，%d 帧）" % (path, size, len(frames)))
    return path


def make_all(results_path: str, dataset_path: str, net_path: str, outdir: str,
             gif: bool = True) -> List[str]:
    from .core import Dataset, ResidualNet
    from .core.experiment import load_results
    from .core.training import evaluate_residual

    use_cjk_font()
    os.makedirs(outdir, exist_ok=True)
    _, recs = load_results(results_path)
    ds = Dataset.load(dataset_path)
    net = ResidualNet.load(net_path)
    m = evaluate_residual(net, ds)
    print("  网络在数据集上的表现：RMSE=%.3f m/s²  R²=%.3f"
          % (m["res_rmse"], m["res_r2"]))
    hist_path = os.path.join(os.path.dirname(net_path), "train_history.json")
    if os.path.exists(hist_path):
        with open(hist_path, "r", encoding="utf-8") as f:
            hist = json.load(f)
    else:
        hist = {"train": [m["res_rmse"] ** 2], "val": []}
    outs = [plot_disturbances(outdir), plot_training(hist, ds, net, outdir)]
    outs.append(plot_tracking(recs, outdir))
    outs.append(plot_payload_sweep(recs, outdir))
    outs.append(plot_wind_rejection(recs, outdir))
    outs.append(plot_rls_adaptation(recs, outdir))
    if gif:
        outs.append(make_flight_gif(outdir, net_path))
    return [o for o in outs if o]
