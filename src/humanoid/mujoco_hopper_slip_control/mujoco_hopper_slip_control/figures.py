# -*- coding: utf-8 -*-
"""图表：高度收敛、相位时程、刚度扫描、抗扰恢复、演示动图。"""

from pathlib import Path
from typing import List, Optional

import numpy as np

import mujoco_hopper_slip_control.config as config

_TARGET = "#1f77b4"
_MEASURED = "#d95f02"
_GRID = dict(alpha=0.25, linestyle="--", linewidth=0.6)


def _plt():
    """延迟导入 matplotlib，避免 --test 在无绘图依赖时失败。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "figure.dpi": 130,
        "font.size": 9,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "grid.linestyle": "--",
    })
    return plt


def plot_height(result, path: Path) -> Path:
    """顶点高度逐跳收敛。"""
    plt = _plt()
    fig, ax = plt.subplots(figsize=(6.2, 3.0))
    hops = result.hops
    idx = [h.index for h in hops]
    apex = [h.z_apex for h in hops]
    target = result.stats.get("target_height", config.TARGET_HEIGHT)
    ax.plot(idx, apex, "o-", ms=3.4, lw=1.2, color=_MEASURED, label="apex height (sim)")
    ax.axhline(target, color=_TARGET, ls="--", lw=1.3, label=f"target {target:.2f} m")
    ax.axhline(np.mean(apex[-config.SETTLE_HOPS:]), color="0.35", lw=0.9,
               ls=":", label="steady mean")
    ax.set_xlabel("hop index (-)")
    ax.set_ylabel("torso apex height (m)")
    ax.set_title(f"Apex height convergence  ({len(hops)} hops)")
    ax.legend(loc="lower right", frameon=False, fontsize=8)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return path


def plot_phase(result, path: Path, t_max: Optional[float] = 4.0) -> Path:
    """躯干高度与腿压缩量时程，底纹标出支撑相。"""
    plt = _plt()
    log = result.log
    t = log["t"]
    if t_max is not None:
        m = t <= t_max
        t = t[m]
        z = log["z"][m]
        q = log["q"][m]
        contact = log["contact"][m].astype(bool)
    else:
        z, q, contact = log["z"], log["q"], log["contact"].astype(bool)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6.4, 4.0), sharex=True)
    ax1.plot(t, z, lw=1.4, color=_MEASURED)
    ax1.axhline(config.TARGET_HEIGHT, color=_TARGET, ls="--", lw=1.0,
                label=f"target {config.TARGET_HEIGHT:.2f} m")
    ax1.set_ylabel("torso height z (m)")
    ax1.set_title("Hopping cycle: height and leg compression")
    ax1.legend(frameon=False, fontsize=8, loc="lower right")
    ax2.plot(t, q, lw=1.4, color="#54a24b")
    ax2.set_ylabel("leg compression q (m)")
    ax2.set_xlabel("time (s)")

    # 先让数据定下 y 范围，再按 ylim 铺底纹，否则底纹会铺满 0~1 全高
    for ax in (ax1, ax2):
        lo, hi = ax.get_ylim()
        for s, e in _contact_segments(contact, t):
            ax.axvspan(s, e, color="#4c78a8", alpha=0.16, lw=0, zorder=0)
        ax.set_ylim(lo, hi)
    ax1.text(0.01, 0.94, "shaded = stance", transform=ax1.transAxes,
             fontsize=7.5, color="#2b5b8c", va="top")

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return path


def _contact_segments(contact, t):
    """把接触布尔序列转成 [(t_start, t_end), ...]。"""
    out, start = [], None
    for i, v in enumerate(contact):
        if v and start is None:
            start = t[i]
        elif not v and start is not None:
            out.append((start, t[i]))
            start = None
    if start is not None:
        out.append((start, t[-1]))
    return out


def plot_sweep(rows: List[dict], path: Path) -> Path:
    """弹簧刚度 → 支撑时间 / 峰值力 / 占空比。"""
    plt = _plt()
    k = [r["stiffness"] for r in rows]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.4, 2.9))

    ax1.plot(k, [r["stance_mean"] * 1e3 for r in rows], "o-", color=_MEASURED,
             ms=4, label="stance time")
    ax1.plot(k, [r["flight_mean"] * 1e3 for r in rows], "s--", color=_TARGET,
             ms=4, label="flight time")
    ax1.set_xlabel("leg stiffness k (N/m)")
    ax1.set_ylabel("duration (ms)")
    ax1.set_title("Phase durations vs stiffness")
    ax1.legend(frameon=False, fontsize=8)

    ax2.plot(k, [r["peak_force"] for r in rows], "o-", color="#7b3294", ms=4)
    ax2.set_xlabel("leg stiffness k (N/m)")
    ax2.set_ylabel("peak leg force (N)")
    ax2.set_title("Peak leg force vs stiffness")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return path


def plot_disturbance(result, path: Path) -> Path:
    """抗扰实验：水平速度时程 + 扰动时刻。"""
    plt = _plt()
    log = result.log
    t0 = result.extra.get("disturb_t", config.DISTURB_T)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 2.9))
    ax1.plot(log["t"], log["vx"], lw=1.0, color=_MEASURED)
    ax1.axvline(t0, color="#c1121f", ls="--", lw=1.1,
                label=f"impulse +{result.extra.get('disturb_dv', 0):.1f} m/s")
    ax1.axvspan(t0, t0 + (result.stats.get("recovery_time") or 0), color="#c1121f", alpha=0.10)
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("horizontal velocity vx (m/s)")
    ax1.set_title("Disturbance response")
    ax1.legend(frameon=False, fontsize=8)

    ax2.plot(log["t"], log["z"], lw=1.0, color="#1f77b4")
    ax2.axvline(t0, color="#c1121f", ls="--", lw=1.1)
    ax2.axhline(config.TARGET_HEIGHT, color="0.4", ls=":", lw=0.9)
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("torso height z (m)")
    ax2.set_title("Height during recovery")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return path


def render_gif(target_height: float = config.TARGET_HEIGHT, path: Optional[Path] = None,
               duration: float = 4.0, fps: int = 16, size=(480, 270),
               camera: str = "track", speed: float = 1.0) -> Path:
    """离屏渲染一段弹跳动图。

    :param speed: 播放倍速，1.0 为实时，0.5 为慢放
    :param fps: 16 帧/秒、480×270 时约 3 MB，给 PR 模板要求的 5 MB 留足余量
    """
    from PIL import Image
    from mujoco_hopper_slip_control.sim import HopperSim

    path = Path(path or "docs/img/humanoid/mujoco_hopper_slip_control/demo.gif")
    sim = HopperSim(target_height=target_height)
    sim.run(1.2)                                   # 先跳过启动瞬态

    n_frames = int(duration * fps)
    every = max(1, int(round(1.0 / (fps * config.TIMESTEP * speed))))
    frames = []
    for _ in range(n_frames):
        for _ in range(every):
            sim.step()
        frames.append(Image.fromarray(sim.render(size[0], size[1], camera)))
    sim.close()

    path.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(path, save_all=True, append_images=frames[1:],
                   duration=int(1000 / fps), loop=0, optimize=True)
    return path
