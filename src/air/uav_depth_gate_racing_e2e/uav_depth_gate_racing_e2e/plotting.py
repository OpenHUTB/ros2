#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""配图。只用 matplotlib + numpy，不依赖 ROS。"""
from __future__ import annotations

import json
import os
import time
from typing import Dict, List, Optional, Sequence

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib import font_manager                  # noqa: E402

from .core.expert import GateExpert

__all__ = ["setup_font", "fig_track", "fig_expert_rollout", "fig_policy_rollout",
           "fig_training", "fig_compare", "fig_latency", "make_gif"]

_CN_FONTS = [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf",
             "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
             "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"]

ARCH_LABEL = {"mlp": "MLP", "cnn": "CNN", "gru": "CNN+GRU",
              "attn": "CNN+注意力"}


def setup_font() -> str:
    for p in _CN_FONTS:
        if os.path.exists(p):
            font_manager.fontManager.addfont(p)
            name = font_manager.FontProperties(fname=p).get_name()
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            return name
    return "DejaVu Sans"


# --------------------------------------------------------------------------- #
def fig_track(env, path: str) -> None:
    """赛道俯视图与侧视图。"""
    gates = env.track.gates
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 4.0))
    xs = [g.x for g in gates]
    a1.plot(xs, [g.y for g in gates], "o-", color="#e8710a", lw=1.4)
    for g in gates:
        a1.plot([g.x, g.x], [g.y - g.half, g.y + g.half], "-",
                color="#1a73e8", lw=3)
        a1.text(g.x, g.y + g.half + 0.35, "G%d" % (g.idx + 1), ha="center",
                fontsize=8)
    a1.set_title("门赛道俯视图（竖线为门开口的横向范围）")
    a1.set_xlabel("x / m")
    a1.set_ylabel("y / m")
    a1.set_aspect("equal")
    a1.grid(alpha=0.3)

    a2.plot(xs, [g.z for g in gates], "o-", color="#e8710a", lw=1.4)
    for g in gates:
        a2.plot([g.x, g.x], [g.z - g.half, g.z + g.half], "-",
                color="#1a73e8", lw=3)
    a2.set_title("侧视图（竖线为门开口的高度范围）")
    a2.set_xlabel("x / m")
    a2.set_ylabel("z / m")
    a2.set_aspect("equal")
    a2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _rollout(env, policy=None, expert=True, max_steps: int = 600, seed: int = 0):
    """跑一轮，返回逐帧位置与门状态。"""
    rng = np.random.default_rng(seed)
    obs = env.reset(seed=seed, jitter=0.0)
    ex = GateExpert() if expert else None
    h = policy.init_hidden(1) if policy is not None else None
    scale = np.array([env.cfg.max_horiz, env.cfg.max_horiz, env.cfg.max_vert])
    pos, passed = [], []
    done = False
    k = 0
    while not done and k < max_steps:
        if expert:
            a = ex.act(env)
        else:
            a, _, _, h, _ = policy.act(obs, h, deterministic=True, rng=rng)
            # 策略输出是 tanh 的 [−1,1]，乘速度上限得到物理指令。漏乘时
            # 无人机以 1/4 速度飞，图上"通过 4/8"是输出未还原，不是策略水平。
            a = a[0].astype(np.float64) * scale
        obs, _, done, info = env.step(a)
        pos.append(env.quad.state().pos.copy())
        passed.append(info["passed"])
        k += 1
    return np.asarray(pos), np.asarray(passed), info


def fig_expert_rollout(env, path: str, seed: int = 0) -> None:
    pos, passed, info = _rollout(env, expert=True, seed=seed)
    _traj_fig(env, pos, passed, info, "专家（真值位姿 + 比例控制）", path)


def fig_policy_rollout(env, policy, path: str, seed: int = 0) -> None:
    pos, passed, info = _rollout(env, policy=policy, expert=False, seed=seed)
    _traj_fig(env, pos, passed, info, "策略（深度图 → 速度指令）", path)


def _traj_fig(env, pos, passed, info, title, path) -> None:
    gates = env.track.gates
    fig, ax = plt.subplots(1, 2, figsize=(14, 4.8))
    # 等比例坐标轴 + 色标不能交给 tight_layout：色标会横穿面板
    fig.subplots_adjust(left=0.06, right=0.98, top=0.84, bottom=0.26,
                        wspace=0.16)
    if len(pos):
        sc = ax[0].scatter(pos[:, 0], pos[:, 2], c=np.arange(len(pos)),
                           cmap="viridis", s=8)
        ax[0].plot(pos[:, 0], pos[:, 2], "-", color="k", lw=0.6, alpha=0.5)
        cax = fig.add_axes([0.32, 0.09, 0.36, 0.035])
        fig.colorbar(sc, cax=cax, orientation="horizontal", label="帧序号")
    for g in gates:
        ax[0].plot([g.x, g.x], [g.z - g.half, g.z + g.half], "-",
                   color="#e8710a", lw=3)
    ax[0].set_title("%s\n侧视图：通过 %d/%d 门，用时 %.1f s，碰撞 %d"
                    % (title, info["passed"], len(gates), info["time"],
                       info["collisions"]))
    ax[0].set_xlabel("x / m")
    ax[0].set_ylabel("z / m")
    ax[0].set_aspect("equal")
    ax[0].grid(alpha=0.3)

    if len(pos):
        ax[1].plot(pos[:, 0], pos[:, 1], "-", color="#1a73e8", lw=1.4)
    for g in gates:
        ax[1].plot([g.x, g.x], [g.y - g.half, g.y + g.half], "-",
                   color="#e8710a", lw=3)
    ax[1].set_title("俯视图")
    ax[1].set_xlabel("x / m")
    ax[1].set_ylabel("y / m")
    ax[1].set_aspect("equal")
    ax[1].grid(alpha=0.3)
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_training(hist: Dict, path: str) -> None:
    """DAgger / PPO 训练曲线。hist 为 JSON 里保存的字典。

    DAgger 面板同时画损失与**闭环通过门数**：只看开环 MSE 会误判——训练/验证
    集都是专家数据时 MSE 一路降，但策略早就退化了（实测三轮后 MSE 0.0094
    看着不错，闭环 0/8）。
    """
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
    d = hist.get("dagger", {})
    if d:
        ep = np.arange(len(d.get("loss", [])))
        ax[0].plot(ep, d.get("loss", []), "o-", color="#1a73e8", label="训练损失")
        ax[0].plot(ep, d.get("val_mse", []), "s-", color="#0f9d58",
                   label="验证 MSE")
        ax[0].set_xlabel("DAgger 轮次")
        ax[0].set_title("DAgger：行为克隆")
        ax[0].grid(alpha=0.3)
        if d.get("passed"):
            ax1b = ax[0].twinx()
            ax1b.plot(ep[:len(d["passed"])], d["passed"], "^--",
                      color="#e8710a", label="闭环通过门数")
            ax1b.set_ylabel("通过门数 / 8")
            ax1b.set_ylim(0, 8.4)
            h1, l1 = ax[0].get_legend_handles_labels()
            h2, l2 = ax1b.get_legend_handles_labels()
            ax[0].legend(h1 + h2, l1 + l2, loc="center left", fontsize=8)
        else:
            ax[0].legend()
    p = hist.get("ppo", {})
    if p:
        it = np.arange(len(p.get("reward", [])))
        ax[1].plot(it, p.get("reward", []), "o-", color="#0f9d58")
        ax[1].set_xlabel("PPO 迭代")
        ax[1].set_ylabel("累计回报")
        ax[1].set_title("PPO：训练回报")
        ax[1].grid(alpha=0.3)
        if p.get("completed"):
            ax2 = ax[1].twinx()
            ax2.plot(it, [100 * v for v in p["completed"]], "s--",
                     color="#e8710a", label="完成率")
            ax2.set_ylabel("完成率 / %")
            ax2.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_compare(rows: Sequence[Dict], path: str) -> None:
    """对比实验②：四种网络结构的完成率 / 通过门数 / 延迟。"""
    if not rows:
        return
    names = [ARCH_LABEL.get(r["arch"], r["arch"]) for r in rows]
    fig, axes = plt.subplots(1, 4, figsize=(17, 4.2))
    # 用"通过门数"而不是"完成率"：完成率的分母是 8 道门，策略只有 1~2 门时
    # 四个柱子全是 0，图上看不出任何差别。
    specs = [("bc_passed", "BC 后通过门数 / 8", False),
             ("ppo_passed", "PPO 后通过门数 / 8", False),
             ("ppo_collisions", "PPO 后擦框数", False),
             ("latency_ms", "单帧推理延迟 / ms", False)]
    for a, (k, title, pct) in zip(axes, specs):
        v = [r.get(k, float("nan")) for r in rows]
        v = [100 * x if pct and x == x else x for x in v]
        a.bar(names, v, color=["#1a73e8", "#0f9d58", "#e8710a", "#8e44ad"][:len(v)])
        a.set_title(title, fontsize=11)
        a.grid(alpha=0.3, axis="y")
        a.tick_params(axis="x", rotation=18, labelsize=9)
        for i, x in enumerate(v):
            if x == x:
                a.text(i, x, "%.1f" % x, ha="center", va="bottom", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_latency(path: str) -> None:
    """推理延迟 vs 输入分辨率（CPU 单线程）。"""
    from .core.policy import Policy, PolicyConfig

    hws = [(48, 64), (64, 80), (96, 96)]
    fig, ax = plt.subplots(figsize=(7.4, 4.6))
    for arch in ("mlp", "cnn", "gru", "attn"):
        ms = []
        for hw in hws:
            p = Policy(PolicyConfig(arch=arch, depth_hw=hw))
            obs = {"depth": np.zeros((1, hw[0], hw[1]), np.uint8),
                   "vel": np.zeros((1, 3), np.float32)}
            h = p.init_hidden(1)
            p.act(obs, h, deterministic=True)          # 预热
            t0 = time.perf_counter()
            for _ in range(20):
                p.act(obs, h, deterministic=True)
            ms.append((time.perf_counter() - t0) / 20 * 1000.0)
        ax.plot([h * w for h, w in hws], ms, "o-",
                label=ARCH_LABEL.get(arch, arch))
    ax.axhline(50.0, ls="--", color="r", lw=1, label="20 Hz 预算 (50 ms)")
    ax.set_xlabel("输入像素数")
    ax.set_ylabel("单帧推理 / ms")
    ax.set_title("CPU 单线程推理延迟")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# --------------------------------------------------------------------------- #
def make_gif(frames: Sequence[np.ndarray], path: str, fps: int = 10,
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
