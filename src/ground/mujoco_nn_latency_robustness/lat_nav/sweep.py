"""实验矩阵与绘图：延迟 × 频率 × 补偿方式的闭环汇总。"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from . import track
from .env import LatencyEnv
from .nav import run_episode

DELAYS_MS = (0, 200, 400, 600, 800, 1000)
RATES_HZ = (10, 20, 50)
COMPS = ("none", "extrap", "model", "oracle")
COMP_LABEL = {"none": "no compensation", "extrap": "linear extrapolation",
              "model": "model forward (warped scan)", "oracle": "model forward (oracle scan)"}
COMPLETE = ("none", "extrap", "model")       # 真车可实现的三种；oracle 只作为上界


def run_matrix(policy, delays=DELAYS_MS, comps=COMPS, rate_hz: float = 20.0,
               shapes=track.SHAPES, seeds=(0, 1), max_time: float = 60.0) -> list[dict]:
    """一维矩阵：固定控制频率，扫延迟与补偿方式。"""
    out = []
    for sh in shapes:
        env = LatencyEnv(sh, seed=0)
        for d in delays:
            for c in comps:
                runs = [run_episode(env, policy=policy, comp=c, delay_ms=d, rate_hz=rate_hz,
                                    max_time=max_time, seed=s) for s in seeds]
                out.append({
                    "shape": sh, "delay_ms": d, "comp": c, "rate_hz": rate_hz,
                    "rms": float(np.mean([r["rms"] for r in runs])),
                    "rms_max": float(np.max([r["rms"] for r in runs])),
                    "success": float(np.mean([r["completed"] for r in runs])),
                    "collide_frames": int(np.sum([r["collide_frames"] for r in runs])),
                    "min_clear": float(np.min([r["min_clear"] for r in runs])),
                    "cpu_ms": float(np.mean([r["mean_cpu_ms"] for r in runs])),
                })
    return out


def run_rate_matrix(policy, rates=RATES_HZ, delay_ms: float = 600.0, comps=COMPS,
                    shapes=track.SHAPES, seeds=(0, 1),
                    max_time: float = 60.0) -> list[dict]:
    """二维矩阵里固定延迟、扫控制频率的那一半。"""
    out = []
    for sh in shapes:
        env = LatencyEnv(sh, seed=0)
        for rt in rates:
            for c in comps:
                runs = [run_episode(env, policy=policy, comp=c, delay_ms=delay_ms,
                                    rate_hz=rt, max_time=max_time, seed=s) for s in seeds]
                out.append({
                    "shape": sh, "delay_ms": delay_ms, "comp": c, "rate_hz": rt,
                    "rms": float(np.mean([r["rms"] for r in runs])),
                    "rms_max": float(np.max([r["rms"] for r in runs])),
                    "success": float(np.mean([r["completed"] for r in runs])),
                    "collide_frames": int(np.sum([r["collide_frames"] for r in runs])),
                    "min_clear": float(np.min([r["min_clear"] for r in runs])),
                    "cpu_ms": float(np.mean([r["mean_cpu_ms"] for r in runs])),
                })
    return out


def summarize(rows: list[dict], key: str = "delay_ms") -> dict:
    """按 key 聚合（对赛道与种子取平均）。"""
    keys = sorted({r[key] for r in rows})
    out = {}
    for c in COMPS:
        sub = [r for r in rows if r["comp"] == c]
        if not sub:
            continue
        out[c] = {
            "x": keys,
            "rms": [float(np.mean([r["rms"] for r in sub if r[key] == k])) for k in keys],
            "success": [float(np.mean([r["success"] for r in sub if r[key] == k])) for k in keys],
            "collide": [int(np.sum([r["collide_frames"] for r in sub if r[key] == k]))
                        for k in keys],
        }
    return out


def plot_delay_sweep(rows: list[dict], path: str | Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    agg = summarize(rows, "delay_ms")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))
    colors = {"none": "#d1495b", "extrap": "#edae49", "model": "#2e86ab",
              "oracle": "#6a4c93"}
    for c, d in agg.items():
        ax1.plot(d["x"], d["rms"], "o--" if c == "oracle" else "o-",
                 color=colors[c], label=COMP_LABEL[c])
        ax2.plot(d["x"], [100.0 * s for s in d["success"]],
                 "o--" if c == "oracle" else "o-", color=colors[c], label=COMP_LABEL[c])
    ax1.set_xlabel("actuation delay [ms]")
    ax1.set_ylabel("lateral RMS error [m]")
    ax1.set_title("tracking accuracy vs delay")
    ax1.grid(alpha=0.3)
    ax1.legend()
    ax2.set_xlabel("actuation delay [ms]")
    ax2.set_ylabel("completion rate [%]")
    ax2.set_title("completion rate vs delay")
    ax2.set_ylim(-5, 105)
    ax2.grid(alpha=0.3)
    ax2.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_rate_sweep(rows: list[dict], path: str | Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    agg = summarize(rows, "rate_hz")
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    colors = {"none": "#d1495b", "extrap": "#edae49", "model": "#2e86ab",
              "oracle": "#6a4c93"}
    for c, d in agg.items():
        ax.plot(d["x"], d["rms"], "o--" if c == "oracle" else "o-",
                color=colors[c], label=COMP_LABEL[c])
    ax.set_xscale("log")
    ax.set_xticks(list(RATES_HZ))
    ax.set_xticklabels([str(r) for r in RATES_HZ])
    ax.set_xlabel("control rate [Hz] @ 600 ms delay")
    ax.set_ylabel("lateral RMS error [m]")
    ax.set_title("effect of control rate")
    ax.grid(alpha=0.3, which="both")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_traces(runs: dict, path: str | Path, shape: str = "kidney") -> None:
    """三条轨迹叠在同一张俯视图上（真值中心线 + 墙体边界）。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path_pts = track.centerline(shape)
    fig, ax = plt.subplots(figsize=(6.4, 6.0))
    ax.plot(path_pts[:, 0], path_pts[:, 1], "k--", lw=1.0, label="centerline")
    for off in (track.LANE_HALF_WIDTH, -track.LANE_HALF_WIDTH):
        b = path_pts + off * track.normals(path_pts)
        ax.plot(b[:, 0], b[:, 1], color="0.55", lw=0.8)
    colors = {"none": "#d1495b", "extrap": "#edae49", "model": "#2e86ab",
              "oracle": "#6a4c93"}
    for c, frames in runs.items():
        if not frames:
            continue
        f = np.asarray(frames)
        ax.plot(f[:, 0], f[:, 1], color=colors[c], lw=1.6,
                ls="--" if c == "oracle" else "-", label=COMP_LABEL[c])
    ax.set_aspect("equal")
    ax.set_title(f"{shape} track: three policies at 600 ms delay")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
