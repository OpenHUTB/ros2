#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""可选绘图：把时间序列与扫描结果导成 PNG。

为什么是"可选"：核心实验只需要 `mujoco + numpy`，绘图依赖 matplotlib 属于
"锦上添花"。评测机没装 matplotlib 时，CSV 仍然完整导出、退出码仍为 0，
只是跳过 PNG —— 这样"跑通"不会被一个绘图库卡住。

坐标轴与图例用英文：matplotlib 默认字体没有中文字形，写中文会变成一排方框
（评审明确要求过"图本身要能自证"）。中文说明写在文档的图注里。
"""

import os

FIGSIZE = (10.0, 6.0)
DPI = 120


def matplotlib_available():
    try:
        import matplotlib  # noqa: F401
        return True
    except Exception:  # noqa: BLE001
        return False


def _pyplot():
    import matplotlib
    matplotlib.use("Agg")          # 无显示环境也能出图
    import matplotlib.pyplot as plt
    return plt


def export_all(results, sweep, cfg, save_dir):
    """导出全部图，返回生成的文件列表；没有 matplotlib 时返回空列表。"""
    if not matplotlib_available():
        print("[提示] 未安装 matplotlib，跳过 PNG 绘图（CSV 已导出，不影响结果）。")
        print("       需要出图时：pip install matplotlib")
        return []
    os.makedirs(save_dir, exist_ok=True)
    out = []
    out.append(plot_responses(results, save_dir))
    out.append(plot_lean_sweep(sweep, cfg, save_dir))
    return [p for p in out if p]


def plot_responses(results, save_dir):
    """三个场景的倾角 / 位置曲线。"""
    plt = _pyplot()
    fig, axes = plt.subplots(2, 1, figsize=FIGSIZE, dpi=DPI, sharex=True)
    styles = {"balance": ("#1f77b4", "balance (lean recovery)"),
              "position_step": ("#2ca02c", "position step 1.0 m"),
              "disturbance": ("#d62728", "velocity push 1.5 m/s")}
    for res in results:
        color, label = styles.get(res.name, ("#555555", res.name))
        axes[0].plot(res.t, [v * 57.2957795 for v in res.angle],
                     color=color, label=label, linewidth=1.2)
        axes[1].plot(res.t, res.x, color=color, label=label, linewidth=1.2)

    axes[0].axhline(0.0, color="#999999", linewidth=0.8, linestyle="--")
    axes[0].set_ylabel("pole angle (deg)")
    axes[0].set_title("Two-wheel balance car: closed-loop response (PID cascade)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(alpha=0.3)

    axes[1].axhline(0.0, color="#999999", linewidth=0.8, linestyle="--")
    axes[1].set_ylabel("chassis position x (m)")
    axes[1].set_xlabel("time (s)")
    axes[1].legend(loc="lower right", fontsize=8)
    axes[1].grid(alpha=0.3)

    path = os.path.join(save_dir, "balance_responses.png")
    fig.tight_layout()
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    return path


def plot_lean_sweep(sweep, cfg, save_dir):
    """初始倾角 → 稳定时间 / 成功率。"""
    plt = _pyplot()
    leans = [r["initial_lean"] * 57.2957795 for r in sweep]
    settles = [r["angle_settle"] if r["angle_settle"] == r["angle_settle"] else float("nan")
               for r in sweep]
    success = [r["success"] for r in sweep]

    fig, ax = plt.subplots(figsize=(8.0, 4.5), dpi=DPI)
    ax.plot(leans, settles, marker="o", color="#1f77b4", label="settle time (s, nan = failed)")
    for lx, st, ok in zip(leans, settles, success):
        if not ok:
            ax.axvspan(lx - 0.6, lx + 0.6, color="#d62728", alpha=0.12)
    ax.set_xlabel("initial pole angle (deg)")
    ax.set_ylabel("settle time (s)")
    ax.set_title("Recoverable initial lean (red band = failed to settle within tolerance)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)

    path = os.path.join(save_dir, "lean_sweep.png")
    fig.tight_layout()
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    return path
