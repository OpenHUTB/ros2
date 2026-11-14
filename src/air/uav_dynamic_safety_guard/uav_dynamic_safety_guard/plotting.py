#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""评测配图：场景轨迹、数据分布、训练曲线、指标、时序、噪声鲁棒性。

中文字体按 Windows / Ubuntu 常见路径探测，找不到就回退 DejaVu Sans（图仍可读）。
"""
from __future__ import annotations

import json
import os
from typing import Dict, List, Optional

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                       # noqa: E402
from matplotlib import font_manager                   # noqa: E402
from matplotlib.patches import Circle, Rectangle      # noqa: E402

__all__ = ["setup_font", "make_all", "LABELS"]

_CN_FONTS = [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf",
             "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
             "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"]

LABELS = {"none": "无过滤", "brake": "刹车", "geometric": "几何教师",
          "neural_single": "单帧神经层", "neural_seq": "时序神经层"}
COLORS = {"none": "#8a8f98", "brake": "#d9a441", "geometric": "#2f9e44",
          "neural_single": "#7b61ff", "neural_seq": "#1f6feb"}


def setup_font() -> str:
    for p in _CN_FONTS:
        if os.path.exists(p):
            font_manager.fontManager.addfont(p)
            name = font_manager.FontProperties(fname=p).get_name()
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            return name
    return "DejaVu Sans"


def _label(name: str) -> str:
    return LABELS.get(name, name)


# --------------------------------------------------------------------------- #
# 1) 场景与轨迹
# --------------------------------------------------------------------------- #
def _draw_scene(ax, field, traj: Dict[str, np.ndarray]) -> None:
    for kind, pos, size in field.static:
        if kind == "box":
            ax.add_patch(Rectangle((pos[0] - size[0], pos[1] - size[1]),
                                   2 * size[0], 2 * size[1], color="#9c8a70",
                                   alpha=0.55, lw=0))
        else:
            ax.add_patch(Circle((pos[0], pos[1]), size[0], color="#7d9b76",
                                alpha=0.6, lw=0))
    for m in field.movers:
        ax.add_patch(Circle(m.center, m.radius, fill=False, ls=":",
                            lw=1.0, color="#c92a2a", alpha=0.7))
        p0 = m.pos_at(0.0)
        ax.add_patch(Circle((p0[0], p0[1]), m.size[0], color="#c92a2a",
                            alpha=0.85, lw=0))
    for name, xy in traj.items():
        if len(xy) == 0:
            continue
        ax.plot(xy[:, 0], xy[:, 1], lw=2.0, color=COLORS.get(name, None),
                label=_label(name), alpha=0.95)
        ax.plot(xy[0, 0], xy[0, 1], "o", ms=5, color=COLORS.get(name, None))
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25, ls=":")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def fig_scene(rec: dict, path: str) -> str:
    fld = rec["field"]
    spec = rec["spec"]
    fig, ax = plt.subplots(figsize=(6.6, 6.2))
    traj = {k: v["timeline"]["pos"][:, :2] for k, v in rec["eps"].items()}
    _draw_scene(ax, fld, traj)
    ax.plot(spec.start[0], spec.start[1], "s", ms=8, mfc="w", mec="k",
            label="起点")
    ax.plot(spec.goal[0], spec.goal[1], "*", ms=14, mfc="gold", mec="k",
            label="终点")
    ax.set_title("动态障碍场：5 个绕圈障碍（红圈为轨迹）+ 18 个静止障碍")
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
# 2) 数据分布
# --------------------------------------------------------------------------- #
def fig_dataset(d: Dict[str, np.ndarray], path: str) -> str:
    h = np.asarray(d["h"], float)
    u_tgt = np.asarray(d["u_tgt"], float)
    u_nom = np.asarray(d["u_nom"], float)
    hist = np.asarray(d["hist"], np.float32)
    dev = np.linalg.norm(u_tgt - u_nom, axis=1)
    fig, axes = plt.subplots(2, 2, figsize=(9.6, 6.6))
    ax = axes[0, 0]
    ax.hist(h, bins=40, color="#1f6feb", alpha=0.75)
    ax.axvline(0.8, color="#c92a2a", ls="--", lw=1.2, label="约束 margin 0.8 m")
    ax.set_xlabel("当前间距 h (m)")
    ax.set_ylabel("帧数")
    ax.set_title("示范帧的间距分布")
    ax.legend(fontsize=8)

    ax = axes[0, 1]
    ax.scatter(np.linalg.norm(u_nom[:, :2], axis=1),
               np.linalg.norm(u_tgt[:, :2], axis=1), s=2, alpha=0.15,
               color="#2f9e44")
    ax.plot([0, 4], [0, 4], "k--", lw=0.8)
    ax.set_xlabel("标称水平速度 (m/s)")
    ax.set_ylabel("示范水平速度 (m/s)")
    ax.set_title("教师对指令的修改")

    ax = axes[1, 0]
    ep = np.asarray(d["episode"], int)
    frac = [float(np.mean(dev[ep == e] > 0.1)) for e in np.unique(ep)]
    ax.hist(frac, bins=20, color="#d9a441", alpha=0.8)
    ax.set_xlabel("单集内被修改的帧占比")
    ax.set_ylabel("回合数")
    ax.set_title("干预率分布（%d 集）" % len(frac))

    ax = axes[1, 1]
    mean_grid = hist[:, 0].mean(axis=0)
    im = ax.imshow(mean_grid, aspect="auto", origin="lower", cmap="viridis",
                   extent=[-180, 180, -1.5, 1.5])
    ax.set_xlabel("方位角 (deg)")
    ax.set_ylabel("俯仰层")
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["-15°", "0°", "+15°"])
    ax.set_title("平均距离栅格（%d 帧）" % len(h))
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.03)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
# 3) 训练曲线
# --------------------------------------------------------------------------- #
def fig_training(hist: dict, path: str, title: str = "行为克隆训练") -> str:
    tr = np.asarray(hist.get("train", []), float)
    va = np.asarray(hist.get("val", []), float)
    ep = np.asarray(hist.get("val_epoch", []), float)
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    ax.semilogy(np.arange(1, len(tr) + 1), tr, color="#1f6feb", lw=1.6,
                label="训练集 MSE")
    if len(va):
        ax.semilogy(ep, va, "o-", ms=3.5, color="#c92a2a", lw=1.4,
                    label="验证集 MSE")
        best = float(np.nanmin(va))
        ax.axhline(best, color="#8a8f98", ls="--", lw=0.8)
        ax.annotate("最优 %.4f" % best, xy=(ep[int(np.nanargmin(va))], best),
                    xytext=(4, 12), textcoords="offset points", fontsize=8)
    ax.set_xlabel("epoch")
    ax.set_ylabel("MSE")
    ax.set_title(title)
    ax.grid(alpha=0.3, ls=":")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
# 4) 指标对比
# --------------------------------------------------------------------------- #
def _bars(ax, summary: Dict[str, Dict[str, float]], order: List[str],
          keys: List[str], names: List[str], ylabel: str, title: str) -> None:
    names = [n for n in order if n in summary]
    x = np.arange(len(names))
    w = 0.8 / max(len(keys), 1)
    for i, (k, nm) in enumerate(zip(keys, names)):
        vals = [summary[n][k] for n in names]
        ax.bar(x + (i - (len(keys) - 1) / 2) * w, vals, w, label=nm)
    ax.set_xticks(x)
    ax.set_xticklabels([_label(n) for n in names], fontsize=8)
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=10)
    ax.grid(axis="y", alpha=0.3, ls=":")


def fig_metrics(results: dict, path: str) -> str:
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.2))
    for ax, key, title in ((axes[0], "dynamic", "动态场景（5 个运动障碍）"),
                           (axes[1], "static", "静态场景（消融对照）")):
        block = results.get(key) or {}
        summary = block.get("summary") or {}
        order = block.get("order") or list(summary.keys())
        names = [n for n in order if n in summary]
        if not names:
            continue
        x = np.arange(len(names))
        coll = [100 * summary[n]["collision"] for n in names]
        succ = [100 * summary[n]["success"] for n in names]
        prog = [100 * summary[n]["progress"] for n in names]
        ax.bar(x - 0.26, coll, 0.25, label="碰撞率", color="#c92a2a")
        ax.bar(x, succ, 0.25, label="到达率", color="#2f9e44")
        ax.bar(x + 0.26, prog, 0.25, label="推进率", color="#1f6feb")
        for i, n in enumerate(names):
            ax.text(x[i] - 0.26, coll[i] + 1.5, "%.0f" % coll[i],
                    ha="center", fontsize=7)
        ax.set_xticks(x)
        ax.set_xticklabels([_label(n) for n in names], fontsize=8)
        ax.set_ylim(0, 112)
        ax.set_ylabel("%")
        ax.set_title(title, fontsize=10)
        ax.grid(axis="y", alpha=0.3, ls=":")
        ax.legend(fontsize=8, loc="upper right")
    fig.suptitle("12 个未见场景（seed 100–111，20 s，1.8 m/s）", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
# 5) 单集时序
# --------------------------------------------------------------------------- #
def fig_timeline(rec: dict, path: str) -> str:
    fig, axes = plt.subplots(2, 1, figsize=(8.6, 6.0), sharex=True)
    for name, ep in rec["eps"].items():
        tl = ep["timeline"]
        c = COLORS.get(name, None)
        axes[0].plot(tl["t"], tl["h"], color=c, lw=1.8, label=_label(name))
        axes[1].plot(tl["t"], tl["speed"], color=c, lw=1.6,
                     label="%s 速度" % _label(name))
    axes[0].axhline(0.0, color="k", lw=1.0)
    axes[0].axhline(0.8, color="#c92a2a", ls="--", lw=1.0,
                    label="安全 margin 0.8 m")
    axes[0].set_ylabel("间距 h (m)")
    axes[0].set_title("单集时序：动态场景 seed %d" % int(rec["spec"].seed))
    axes[0].grid(alpha=0.3, ls=":")
    axes[0].legend(fontsize=8, ncol=2)
    axes[1].set_xlabel("时间 (s)")
    axes[1].set_ylabel("速度 (m/s)")
    axes[1].grid(alpha=0.3, ls=":")
    axes[1].legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
# 6) 噪声鲁棒性
# --------------------------------------------------------------------------- #
def fig_noise(results: dict, path: str) -> str:
    clean = (results.get("dynamic") or {}).get("summary") or {}
    noisy = (results.get("noise") or {}).get("summary") or {}
    names = [n for n in ((results.get("dynamic") or {}).get("order") or [])
             if n in clean and n in noisy]
    if not names:
        names = sorted(set(clean) & set(noisy))
    fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.0))
    for ax, key, title in ((axes[0], "collision", "碰撞率 (%)"),
                           (axes[1], "success", "到达率 (%)")):
        x = np.arange(len(names))
        a = [100 * clean[n][key] for n in names]
        b = [100 * noisy[n][key] for n in names]
        ax.bar(x - 0.19, a, 0.36, label="无噪声", color="#1f6feb")
        ax.bar(x + 0.19, b, 0.36, label="0.08 m + 15% 丢点", color="#c92a2a")
        ax.set_xticks(x)
        ax.set_xticklabels([_label(n) for n in names], fontsize=8)
        ax.set_ylim(0, 112)
        ax.set_title(title, fontsize=10)
        ax.grid(axis="y", alpha=0.3, ls=":")
        ax.legend(fontsize=8)
    fig.suptitle("雷达噪声与丢点的影响（动态场景）", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
# 记录一集用于出图
# --------------------------------------------------------------------------- #
def _filters_for(env, model_seq: str, model_single: Optional[str]):
    from .core.filter import (BrakeFilter, FilterConfig, GeometricGuard,
                              NeuralGuard, NoFilter)
    from .core.network import GuardNet
    out = [NoFilter(), BrakeFilter(), GeometricGuard(env, FilterConfig())]
    if model_single and os.path.exists(model_single):
        out.append(NeuralGuard(GuardNet.load(model_single), FilterConfig(),
                               "single"))
    out.append(NeuralGuard(GuardNet.load(model_seq), FilterConfig(), "seq"))
    return out


def record_pair(model_seq: str, model_single: Optional[str], eval_seed: int,
                moving: int = 5, tries: int = 8) -> dict:
    """挑一集“无过滤会撞、时序层能到”的场景用来出图，读起来更直观。"""
    from .core.env import DynamicEnv, EnvConfig, make_episodes
    from .core.experiment import run_episode

    cfg = EnvConfig()
    env = DynamicEnv(cfg)
    filters = _filters_for(env, model_seq, model_single)
    specs = make_episodes(list(range(eval_seed, eval_seed + tries)), 1,
                          cfg=cfg, n_moving=moving)
    chosen = specs[0]
    for spec in specs:
        env.reseed_scene(spec.seed, moving)
        r_none = run_episode(env, filters[0], spec)
        env.reseed_scene(spec.seed, moving)
        r_seq = run_episode(env, filters[-1], spec)
        env.reseed_scene(spec.seed, moving)
        r_teacher = run_episode(env, filters[2], spec)
        if (r_none["collision"] and r_seq["success"]
                and r_teacher["success"]):
            chosen = spec
            break
    eps = {}
    for f in filters:
        env.reseed_scene(chosen.seed, moving)
        eps[f.name] = run_episode(env, f, chosen, record=True)
    return {"field": env.field, "spec": chosen, "eps": eps}


def make_all(results: dict, model_seq: str, model_single: Optional[str],
             outdir: str, eval_seed: int = 100,
             data_path: str = "data/dynamic_imitation.npz") -> List[str]:
    setup_font()
    os.makedirs(outdir, exist_ok=True)
    out: List[str] = []
    rec = record_pair(model_seq, model_single, eval_seed)
    out.append(fig_scene(rec, os.path.join(outdir, "01_scene.png")))
    if os.path.exists(data_path):
        with np.load(data_path, allow_pickle=False) as d:
            out.append(fig_dataset({k: np.asarray(d[k]) for k in d.files},
                                   os.path.join(outdir, "02_dataset.png")))
    hist_path = model_seq + ".json"
    if os.path.exists(hist_path):
        with open(hist_path, "r", encoding="utf-8") as f:
            hist = json.load(f)
        out.append(fig_training(hist, os.path.join(outdir, "03_training.png")))
    out.append(fig_metrics(results, os.path.join(outdir, "04_metrics.png")))
    out.append(fig_timeline(rec, os.path.join(outdir, "05_timeline.png")))
    if results.get("noise"):
        out.append(fig_noise(results, os.path.join(outdir, "06_noise.png")))
    return out
