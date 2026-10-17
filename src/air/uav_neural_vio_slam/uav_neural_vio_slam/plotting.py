#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""配图与动图。只用 matplotlib + numpy，不依赖 ROS 与 mujoco。"""
from __future__ import annotations

import os
from typing import Dict, List, Optional, Sequence

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib import font_manager                  # noqa: E402

__all__ = ["setup_font", "fig_world", "fig_frontend", "fig_trajectories",
           "fig_metrics", "fig_outage", "fig_map", "make_gif"]

#: 后端 → (图例名, 颜色)
BACKEND_STYLE = {
    "odom": ("纯里程计", "#9aa0a6"),
    "ekf": ("EKF-SLAM", "#1a73e8"),
    "ekf_learned": ("EKF + 神经观测模型", "#e8710a"),
    "fastslam": ("FastSLAM 1.0", "#0f9d58"),
}

_CN_FONTS = [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf",
             "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
             "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"]


def setup_font() -> str:
    """显式加载中文字体。matplotlib 默认字体画中文是方块。"""
    for p in _CN_FONTS:
        if os.path.exists(p):
            font_manager.fontManager.addfont(p)
            name = font_manager.FontProperties(fname=p).get_name()
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            return name
    return "DejaVu Sans"


# --------------------------------------------------------------------------- #
def fig_world(world, traj: np.ndarray, path: str) -> None:
    """世界俯视图 + 八字航线。"""
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 6.0))
    pos = world.positions
    for x, y in pos:
        a1.add_patch(plt.Circle((x, y), 0.13, color="#d9534f"))
    for o in world.scene.objects:
        if not o.name.startswith("pole"):
            a1.add_patch(plt.Rectangle((o.pos[0] - o.size[0] / 2,
                                        o.pos[1] - o.size[1] / 2),
                                       o.size[0], o.size[1], color="#78808a"))
    a1.plot(traj[:, 0], traj[:, 1], "-", color="#1a73e8", lw=1.6)
    a1.plot([0], [0], "k^", ms=9)
    a1.set_title("SLAM 基准世界：%d 根立柱（红）+ %d 个遮挡方柱（灰）"
                 % (len(pos), sum(1 for o in world.scene.objects
                                  if not o.name.startswith("pole"))))
    a1.set_xlabel("x / m")
    a1.set_ylabel("y / m")
    a1.set_aspect("equal")
    a1.grid(alpha=0.3)

    a2.plot(traj[:, 0], traj[:, 1], "-", color="#e8710a", lw=1.8)
    a2.plot([traj[0, 0]], [traj[0, 1]], "k^", ms=9, label="起点")
    a2.plot([traj[len(traj) // 4, 0]], [traj[len(traj) // 4, 1]], "ks", ms=7,
            label="1/4 周期")
    a2.legend(loc="upper right")
    a2.set_title("八字航线：自带一次回环（自交点）")
    a2.set_xlabel("x / m")
    a2.set_aspect("equal")
    a2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_frontend(pred: Dict[str, np.ndarray], ep, path: str) -> None:
    """前端精度：逐帧误差、误差分布、不确定度标定。"""
    err = np.linalg.norm(pred["dp"][:, :2] - ep.dp[:, :2], axis=1)
    t = np.arange(len(err)) * 0.1
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.4))

    ax[0].plot(t, err * 1000, lw=0.8, color="#1a73e8")
    ax[0].set_title("逐帧位移增量误差（中位 %.2f mm）"
                    % (np.median(err) * 1000))
    ax[0].set_xlabel("时间 / s")
    ax[0].set_ylabel("|Δp 误差| / mm")
    ax[0].grid(alpha=0.3)

    e = np.sort(err * 1000)
    ax[1].plot(e, np.arange(1, len(e) + 1) / len(e), color="#0f9d58")
    ax[1].set_title("误差累积分布")
    ax[1].set_xlabel("|Δp 误差| / mm")
    ax[1].set_ylabel("累积比例")
    ax[1].set_xlim(0, np.percentile(e, 99) * 1.05)
    ax[1].grid(alpha=0.3)

    # 标定：预测 σ 与实际误差的比值随时间
    sig = np.maximum(pred["sigma"][:, 0], 1e-9)
    ratio = np.cumsum(err) / np.maximum(np.cumsum(sig), 1e-9)
    ax[2].plot(t, ratio, color="#e8710a")
    ax[2].axhline(1.0, ls="--", color="k", lw=1)
    ax[2].set_title("不确定度标定比（实际/预测，收敛值 %.2f）" % ratio[-1])
    ax[2].set_xlabel("时间 / s")
    ax[2].set_ylabel("比值")
    ax[2].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_trajectories(results: Sequence, path: str) -> None:
    """各后端轨迹与真值对比（每个后端一个子图）。"""
    n = len(results)
    cols = min(n, 4)
    rows = int(np.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(3.9 * cols, 3.9 * rows),
                             squeeze=False)
    for k, r in enumerate(results):
        a = axes[k // cols][k % cols]
        a.plot(r.gt[:, 0], r.gt[:, 1], "-", color="k", lw=1.4, label="真值")
        a.plot(r.poses[:, 0], r.poses[:, 1], "--",
               color=BACKEND_STYLE.get(r.backend, ("", "#999"))[1], lw=1.4,
               label="估计")
        a.set_title("%s\nATE %.3f m" % (
            BACKEND_STYLE.get(r.backend, (r.backend,))[0],
            r.metrics.get("ate_raw", float("nan"))), fontsize=10)
        a.set_aspect("equal")
        a.grid(alpha=0.3)
        a.legend(fontsize=8, loc="upper right")
    for k in range(n, rows * cols):
        axes[k // cols][k % cols].axis("off")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_metrics(summary: Dict[str, Dict[str, float]], path: str) -> None:
    """四个指标的柱状对比。"""
    keys = [("ate_raw", "ATE / m（不对齐）"), ("rpe_t", "RPE / m（Δ=1 s）"),
            ("lm_rmse", "路标 RMSE / m"), ("map_iou", "地图 IoU")]
    order = [b for b in BACKEND_STYLE if b in summary]
    fig, axes = plt.subplots(1, 4, figsize=(16, 4.2))
    for a, (k, title) in zip(axes, keys):
        vals = [summary[b].get(k, float("nan")) for b in order]
        a.bar([BACKEND_STYLE[b][0] for b in order], vals,
              color=[BACKEND_STYLE[b][1] for b in order])
        a.set_title(title, fontsize=11)
        a.grid(alpha=0.3, axis="y")
        a.tick_params(axis="x", rotation=25, labelsize=8)
        for i, v in enumerate(vals):
            if np.isfinite(v):
                a.text(i, v, "%.3f" % v, ha="center", va="bottom", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_outage(by_outage: Dict[float, Dict[str, Dict[str, float]]],
               path: str) -> None:
    """GPS 中断时长 → ATE 退化曲线。"""
    xs = sorted(by_outage)
    fig, a = plt.subplots(figsize=(7.6, 4.8))
    for b, (label, color) in BACKEND_STYLE.items():
        ys = [by_outage[x].get(b, {}).get("ate_raw", float("nan")) for x in xs]
        if all(not np.isfinite(v) for v in ys):
            continue
        a.plot(xs, ys, "o-", label=label, color=color, lw=1.6)
    a.set_xlabel("GPS 中断时长 / s")
    a.set_ylabel("ATE / m")
    a.set_title("GPS 中断下的退化")
    a.grid(alpha=0.3)
    a.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_map(results: Sequence, path: str) -> None:
    """占用栅格：估计 vs 真值。"""
    rows = [r for r in results if r.grid is not None]
    if not rows:
        return
    fig, axes = plt.subplots(1, len(rows), figsize=(4.4 * len(rows), 4.4),
                             squeeze=False)
    for k, r in enumerate(rows):
        a = axes[0][k]
        a.imshow(r.gt_grid.T, origin="lower", cmap="Greys", vmin=0.0, vmax=1.0,
                 extent=(-28, 28, -28, 28), alpha=0.55)
        a.imshow(r.grid.T, origin="lower", cmap="autumn_r", vmin=0.0, vmax=1.0,
                 extent=(-28, 28, -28, 28), alpha=0.55)
        a.plot(r.gt[:, 0], r.gt[:, 1], "b-", lw=0.8)
        a.set_title("%s  IoU %.2f" % (
            BACKEND_STYLE.get(r.backend, (r.backend,))[0],
            r.metrics.get("map_iou", float("nan"))), fontsize=10)
        a.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# --------------------------------------------------------------------------- #
def make_gif(frames: Sequence[np.ndarray], path: str, fps: int = 10,
             max_width: int = 900) -> None:
    """把一组 RGB 帧写成 GIF（<10 MB 是课程要求）。"""
    from PIL import Image

    imgs = [Image.fromarray(np.asarray(f, dtype=np.uint8)) for f in frames]
    if not imgs:
        return
    if imgs[0].width > max_width:
        h = int(imgs[0].height * max_width / imgs[0].width)
        imgs = [im.resize((max_width, h), Image.BILINEAR) for im in imgs]
    palette = imgs[0].convert("P", palette=Image.ADAPTIVE, colors=200)
    rest = [im.convert("P", palette=Image.ADAPTIVE, colors=200) for im in imgs[1:]]
    palette.save(path, save_all=True, append_images=rest, duration=int(1000 / fps),
                 loop=0, optimize=True)
