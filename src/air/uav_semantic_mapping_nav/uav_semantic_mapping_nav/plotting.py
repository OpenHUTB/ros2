#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""配图。只用 matplotlib + numpy，不依赖 ROS 与 mujoco。"""
from __future__ import annotations

import os
from typing import Dict, Optional, Sequence

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib import font_manager                  # noqa: E402
from matplotlib.colors import ListedColormap         # noqa: E402

from .core.labels import CLASS_NAMES_CN, N_CLASS, SEMANTIC_RGB

__all__ = ["setup_font", "fig_seg_samples", "fig_training", "fig_maps",
           "fig_paths", "fig_metrics", "fig_scene", "make_gif",
           "semantic_cmap"]

_CN_FONTS = [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf",
             "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
             "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"]

MAP_STYLE = {"geometric": ("几何代价图", "#1a73e8"),
             "semantic": ("语义代价图", "#0f9d58"),
             "oracle": ("语义（真值分割）", "#e8710a")}


def setup_font() -> str:
    for p in _CN_FONTS:
        if os.path.exists(p):
            font_manager.fontManager.addfont(p)
            name = font_manager.FontProperties(fname=p).get_name()
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            return name
    return "DejaVu Sans"


def semantic_cmap() -> ListedColormap:
    return ListedColormap(SEMANTIC_RGB)


# --------------------------------------------------------------------------- #
def fig_seg_samples(ep, pred: Dict[str, np.ndarray], path: str,
                    n: int = 4) -> None:
    """RGB / 真值语义 / 预测语义 / 置信度，各取几帧。"""
    idx = np.linspace(0, len(ep) - 1, n).astype(int)
    fig, axes = plt.subplots(4, n, figsize=(3.1 * n, 11.0))
    cmap = semantic_cmap()
    for k, i in enumerate(idx):
        axes[0][k].imshow(ep.rgb[i])
        axes[0][k].set_title("RGB  第 %d 帧" % i, fontsize=9)
        axes[1][k].imshow(ep.seg[i], cmap=cmap, vmin=0, vmax=N_CLASS - 1)
        axes[1][k].set_title("真值语义", fontsize=9)
        axes[2][k].imshow(pred["seg"][i], cmap=cmap, vmin=0, vmax=N_CLASS - 1)
        axes[2][k].set_title("网络预测", fontsize=9)
        im = axes[3][k].imshow(pred["conf"][i], cmap="viridis", vmin=0, vmax=1)
        axes[3][k].set_title("置信度", fontsize=9)
        for r in range(4):
            axes[r][k].axis("off")
    fig.colorbar(im, ax=axes[3], fraction=0.03, pad=0.01)
    fig.suptitle("语义分割：RGB → 真值 → 预测 → 置信度", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(path, dpi=120)
    plt.close(fig)


def fig_training(net, ds, path: str) -> None:
    """逐类 IoU 与混淆矩阵。"""
    from .core.metrics import confusion, iou_per_class, miou, pixel_acc
    from .core.segdata import SegDataset       # noqa: F401
    from .core.training import evaluate

    m = evaluate(net, ds, list(range(len(ds.episodes))))   # 与 --evaluate 同口径
    iou = iou_per_class(m["conf"])
    fig, ax = plt.subplots(1, 2, figsize=(14, 5.2))
    xs = np.arange(N_CLASS)
    ax[0].bar(xs, np.nan_to_num(iou), color=SEMANTIC_RGB)
    ax[0].set_xticks(xs)
    ax[0].set_xticklabels(CLASS_NAMES_CN, fontsize=9)
    ax[0].set_ylabel("IoU")
    ax[0].set_title("逐类 IoU（mIoU %.3f，像素准确率 %.3f）"
                    % (m["miou"], m["pixel_acc"]))
    ax[0].grid(alpha=0.3, axis="y")
    for i, v in enumerate(iou):
        if v == v:
            ax[0].text(i, v, "%.2f" % v, ha="center", va="bottom", fontsize=8)

    c = m["conf"].astype(float)
    cn = c / np.maximum(c.sum(axis=1, keepdims=True), 1)
    im = ax[1].imshow(cn, cmap="Blues", vmin=0, vmax=1)
    ax[1].set_xticks(xs), ax[1].set_yticks(xs)
    ax[1].set_xticklabels(CLASS_NAMES_CN, rotation=45, fontsize=8)
    ax[1].set_yticklabels(CLASS_NAMES_CN, fontsize=8)
    ax[1].set_xlabel("预测"), ax[1].set_ylabel("真值")
    ax[1].set_title("混淆矩阵（按真值行归一化）")
    fig.colorbar(im, ax=ax[1], fraction=0.045)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_maps(g_pred, g_truth, path: str) -> None:
    """三张图：占据概率、语义（网络）、语义（真值）。"""
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.4))
    ext = (-g_pred.cfg.size / 2, g_pred.cfg.size / 2) * 2
    axes[0].imshow(g_pred.probability().T, origin="lower", cmap="Greys",
                   vmin=0, vmax=1, extent=ext)
    axes[0].set_title("占据概率（几何）")
    axes[1].imshow(g_pred.semantic().T, origin="lower", cmap=semantic_cmap(),
                   vmin=0, vmax=N_CLASS - 1, extent=ext)
    axes[1].set_title("语义地图（网络预测）")
    axes[2].imshow(g_truth.semantic().T, origin="lower", cmap=semantic_cmap(),
                   vmin=0, vmax=N_CLASS - 1, extent=ext)
    axes[2].set_title("语义地图（真值分割）")
    for a in axes:
        a.set_aspect("equal")
        a.set_xlabel("x / m")
    axes[0].set_ylabel("y / m")
    fig.tight_layout()
    fig.savefig(path, dpi=125)
    plt.close(fig)


def fig_paths(grid, maps: Dict[str, np.ndarray], scenarios, rows, path: str) -> None:
    """三张代价图 + 各自规划出的路径。"""
    keys = ["geometric", "semantic", "oracle"]
    fig, axes = plt.subplots(1, len(keys), figsize=(5.4 * len(keys), 5.6))
    ext = (-grid.cfg.size / 2, grid.cfg.size / 2) * 2
    for a, k in zip(axes, keys):
        c = maps[k].copy()
        finite = np.isfinite(c)
        vmax = float(np.nanpercentile(c[finite], 95)) if finite.any() else 3.0
        shown = np.where(finite, c, vmax * 1.4)
        im = a.imshow(shown.T, origin="lower", cmap="RdYlGn_r", vmin=0.6,
                      vmax=max(vmax, 3.0), extent=ext)
        for sc in scenarios:
            sel = [r for r in rows if r["map"] == k and r["scenario"] == sc.name]
            if not sel or not sel[0]["ok"]:
                continue
            from .core.planner import astar, smooth_path
            s, g = sc.cells(grid)
            r = astar(maps[k], s, g)
            if not r.ok:
                continue
            cells = smooth_path(maps[k], r.cells)
            xy = np.asarray([grid.centre(a_, b_) for a_, b_ in cells])
            a.plot(xy[:, 0], xy[:, 1], "-", color=MAP_STYLE[k][1], lw=2.0)
            a.plot(*sc.start, "k^", ms=7)
            a.plot(*sc.goal, "k*", ms=11)
        a.set_title(MAP_STYLE[k][0])
        a.set_aspect("equal")
        a.set_xlabel("x / m")
        fig.colorbar(im, ax=a, fraction=0.045, label="通行代价")
    axes[0].set_ylabel("y / m")
    fig.tight_layout()
    fig.savefig(path, dpi=125)
    plt.close(fig)


def fig_metrics(rows, path: str) -> None:
    from .core.experiment import summarise

    s = summarise(rows)
    keys = [("ok", "成功率"), ("length", "路径长度 / m"),
            ("collision", "碰撞采样点数"), ("uncertainty", "语义不确定性"),
            ("expand", "A* 展开节点数")]
    fig, axes = plt.subplots(1, len(keys), figsize=(18, 4.2))
    names = [k for k in MAP_STYLE if k in s]
    for a, (k, title) in zip(axes, keys):
        vals = [s[n].get(k, float("nan")) for n in names]
        a.bar([MAP_STYLE[n][0] for n in names], vals,
              color=[MAP_STYLE[n][1] for n in names])
        a.set_title(title, fontsize=11)
        a.grid(alpha=0.3, axis="y")
        a.tick_params(axis="x", rotation=20, labelsize=8)
        for i, v in enumerate(vals):
            if v == v:
                a.text(i, v, "%.2f" % v, ha="center", va="bottom", fontsize=8)
        finite = [v for v in vals if v == v]
        if finite and max(finite) == 0:      # 柱高为 0 时看着像没画
            a.set_ylim(0, 1)
            a.text(0.5, 0.9, "全部为 0", transform=a.transAxes,
                   ha="center", va="top", fontsize=9, color="#555")
    fig.tight_layout()
    fig.savefig(path, dpi=125)
    plt.close(fig)


def fig_scene(ep, ds, path: str) -> None:
    """扫描航线在各场景上的俯视图。"""
    n = len(ds.episodes)
    fig, axes = plt.subplots(1, n, figsize=(4.4 * n, 3.4), squeeze=False)
    for k, e in enumerate(ds.episodes):
        a = axes[0][k]
        a.plot(e.pos[:, 0], e.pos[:, 1], "-", color="#1a73e8", lw=1.6)
        a.plot([e.pos[0, 0]], [e.pos[0, 1]], "k^", ms=8)
        a.set_title(e.meta.get("spec", "第 %d 集" % k), fontsize=9)
        # 长直航线（50 m 直廊）在等比例下会压成一条线，补足短边到 1:4
        dx = float(np.ptp(e.pos[:, 0]))
        dy = float(np.ptp(e.pos[:, 1]))
        if dx > 0 and dy > 0:
            if dx / dy > 4.0:
                c = 0.5 * float(e.pos[:, 1].min() + e.pos[:, 1].max())
                h = dx / 4.0
                a.set_ylim(c - h / 2, c + h / 2)
            elif dy / dx > 4.0:
                c = 0.5 * float(e.pos[:, 0].min() + e.pos[:, 0].max())
                w = dy / 4.0
                a.set_xlim(c - w / 2, c + w / 2)
        a.set_aspect("equal")
        a.grid(alpha=0.3)
        a.set_xlabel("x / m")
    axes[0][0].set_ylabel("y / m")
    fig.suptitle("采集用的弓字形扫描航线", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=125)
    plt.close(fig)


# --------------------------------------------------------------------------- #
def make_gif(frames: Sequence[np.ndarray], path: str, fps: int = 6,
             max_width: int = 900) -> None:
    from PIL import Image

    imgs = [Image.fromarray(np.asarray(f, dtype=np.uint8)) for f in frames]
    if not imgs:
        return
    if imgs[0].width > max_width:
        h = int(imgs[0].height * max_width / imgs[0].width)
        imgs = [im.resize((max_width, h), Image.BILINEAR) for im in imgs]
    head = imgs[0].convert("P", palette=Image.ADAPTIVE, colors=200)
    rest = [im.convert("P", palette=Image.ADAPTIVE, colors=200) for im in imgs[1:]]
    head.save(path, save_all=True, append_images=rest,
              duration=int(1000 / fps), loop=0, optimize=True)
