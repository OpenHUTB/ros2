#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""评测配图：场景轨迹、数据分布、训练曲线、指标、时序、噪声鲁棒性。"""
from __future__ import annotations

import json
import os

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib import font_manager                  # noqa: E402
from matplotlib.patches import Circle, Rectangle     # noqa: E402

__all__ = ["setup_font", "make_all"]

_CN_FONTS = [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf",
             "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
             "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"]


def setup_font() -> str:
    for p in _CN_FONTS:
        if os.path.exists(p):
            font_manager.fontManager.addfont(p)
            name = font_manager.FontProperties(fname=p).get_name()
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            return name
    return "DejaVu Sans"


def fig_scene(res, path):
    fig, ax = plt.subplots(figsize=(6.4, 6.0))
    for o in res.get("scene") or []:
        x, y = o["pos"][0], o["pos"][1]
        if o["kind"] == "box":
            ax.add_patch(Rectangle((x - o["size"][0], y - o["size"][1]),
                                   2 * o["size"][0], 2 * o["size"][1],
                                   color="#8a6d3b", alpha=0.75))
        else:
            ax.add_patch(Circle((x, y), o["size"][0], color="#3c7a45",
                                alpha=0.75))
    colors = {"none": "#d93025", "learned": "#1a73e8"}
    labels = {"none": "无安全层（碰撞）", "learned": "神经安全层"}
    for key, name in (("timeline_none", "none"), ("timeline", "learned")):
        tl = res.get(key)
        if not tl:
            continue
        p = np.asarray(tl["timeline"]["pos"])
        ax.plot(p[:, 0], p[:, 1], color=colors[name], lw=2.4, label=labels[name])
        ax.plot(p[0, 0], p[0, 1], "o", color=colors[name], ms=6)
    ax.set_aspect("equal")
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.set_title("障碍场俯视图与飞行轨迹")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_data(data_path, path):
    with np.load(data_path, allow_pickle=False) as d:
        h = d["h"]
        q = d["q"]
        action = d["action"]
        rmin = np.min(d["ranges"].reshape(len(h), -1), axis=1) / 255 * 8
    fig, ax = plt.subplots(2, 2, figsize=(9.5, 6.2))
    ax[0, 0].hist(np.clip(h, -1, 4), bins=60, color="#1a73e8")
    ax[0, 0].set_title("当前间距 h 分布")
    ax[0, 0].set_xlabel("m")
    ax[0, 1].hist(np.clip(q, -1, 4), bins=60, color="#0f9d58")
    ax[0, 1].set_title("候选动作一步间距 q 分布")
    ax[0, 1].set_xlabel("m")
    sp = np.linalg.norm(action[:, :2], axis=1)
    ax[1, 0].hist(sp, bins=40, color="#e8710a")
    ax[1, 0].set_title("采集动作的水平速度")
    ax[1, 0].set_xlabel("m/s")
    ax[1, 1].hist(rmin, bins=50, color="#8e44ad")
    ax[1, 1].set_title("每帧最近回波")
    ax[1, 1].set_xlabel("m")
    for a in ax.ravel():
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_training(model_path, path):
    hist_path = model_path + ".json"
    if not os.path.exists(hist_path):
        return False
    with open(hist_path, encoding="utf-8") as f:
        h = json.load(f)
    tr, va = h.get("train"), h.get("val")
    if not tr:
        return False
    fig, ax = plt.subplots(figsize=(6.2, 4.0))
    ax.plot(tr, label="训练", lw=2)
    ax.plot(va, label="验证", lw=2)
    ax.set_yscale("log")
    ax.set_xlabel("epoch")
    ax.set_ylabel("MSE")
    ax.set_title("安全层行为克隆损失")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return True


def fig_metrics(res, path):
    s = res["summary"]
    names = [k for k in ("none", "brake", "geometric", "learned") if k in s]
    labels = {"none": "无过滤", "brake": "刹车", "geometric": "几何CBF",
              "learned": "神经安全层"}
    fig, ax = plt.subplots(1, 4, figsize=(12.5, 3.4))
    specs = [("collision", "碰撞率", True, "#d93025"),
             ("success", "到达率", True, "#0f9d58"),
             ("progress", "推进率", True, "#1a73e8"),
             ("latency_ms", "延迟 (ms)", False, "#e8710a")]
    for a, (key, title, pct, color) in zip(ax, specs):
        v = [100 * s[n][key] if pct else s[n][key] for n in names]
        a.bar([labels[n] for n in names], v, color=color)
        a.set_title(title, fontsize=11)
        a.grid(alpha=0.3, axis="y")
        a.tick_params(axis="x", rotation=18, labelsize=8)
        for i, x in enumerate(v):
            a.text(i, x, "%.1f" % x if not pct else "%.0f%%" % x,
                   ha="center", va="bottom", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_timeline(res, path):
    tl = res.get("timeline")
    if not tl:
        return False
    t = np.asarray(tl["timeline"]["t"])
    tn = np.asarray(res["timeline_none"]["timeline"]["t"])
    fig, ax = plt.subplots(3, 1, figsize=(7.2, 5.6), sharex=True)
    ax[0].plot(tn, res["timeline_none"]["timeline"]["h"], color="#d93025",
               lw=2, label="无过滤")
    ax[0].plot(t, tl["timeline"]["h"], color="#1a73e8", lw=2,
               label="神经安全层")
    ax[0].set_ylabel("扫掠间距 / m")
    ax[0].legend(fontsize=8)
    ax[1].plot(tn, res["timeline_none"]["timeline"]["speed"], color="#d93025",
               lw=1.6, label="无过滤")
    ax[1].plot(t, tl["timeline"]["speed"], color="#1a73e8", lw=1.6,
               label="神经安全层")
    ax[1].set_ylabel("速度 / (m/s)")
    ax[1].legend(fontsize=8)
    u = np.linalg.norm(np.asarray(tl["timeline"]["u"]), axis=1)
    un = np.linalg.norm(np.asarray(tl["timeline"]["u_nom"]), axis=1)
    ax[2].plot(t, un, color="#999999", lw=1.6, label="标称指令")
    ax[2].plot(t, u, color="#1a73e8", lw=1.6, label="修正后")
    ax[2].set_ylabel("指令模长 / (m/s)")
    ax[2].legend(fontsize=8)
    ax[2].set_xlabel("t / s")
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return True


def fig_noise(res, path):
    rows = res.get("noise") or []
    if not rows:
        return False
    clean = res["summary"]["learned"]

    def agg(key):
        v = [float(r[key]) for r in rows]
        return float(np.mean(v))

    fig, ax = plt.subplots(figsize=(5.6, 3.6))
    keys = [("collision", "碰撞率", True), ("success", "到达率", True),
            ("progress", "推进率", True), ("h_min", "最小间距", False)]
    x = np.arange(len(keys))
    a = [100 * clean[k] if p else clean[k] for k, _, p in keys]
    b = [100 * agg(k) if p else agg(k) for k, _, p in keys]
    ax.bar(x - 0.2, a, 0.4, label="无噪声", color="#1a73e8")
    ax.bar(x + 0.2, b, 0.4, label="测距噪声 + 15% 丢点", color="#e8710a")
    ax.set_xticks(x)
    ax.set_xticklabels([t for _, t, _ in keys])
    ax.set_title("神经安全层的鲁棒性")
    ax.grid(alpha=0.3, axis="y")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return True


def make_all(model_path: str, res: dict, outdir: str,
             data_path: str = "data/safety_dataset.npz") -> None:
    setup_font()
    os.makedirs(outdir, exist_ok=True)
    fig_scene(res, os.path.join(outdir, "01_scene.png"))
    if os.path.exists(data_path):
        fig_data(data_path, os.path.join(outdir, "02_dataset.png"))
    fig_training(model_path, os.path.join(outdir, "03_training.png"))
    fig_metrics(res, os.path.join(outdir, "04_metrics.png"))
    fig_timeline(res, os.path.join(outdir, "05_timeline.png"))
    fig_noise(res, os.path.join(outdir, "06_noise.png"))
