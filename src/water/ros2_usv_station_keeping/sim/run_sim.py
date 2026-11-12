"""跑全部场景并生成文档配图（纯 matplotlib，不依赖 ROS）。"""
from __future__ import annotations

import argparse
import math
import pathlib

try:  # 绘图是可选依赖：没装 matplotlib 时仍然能跑数值、出指标表
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover - 取决于运行环境
    matplotlib = None
    plt = None

from ros2_usv_station_keeping.pilot import PilotConfig

from .experiments import EpisodeMetrics, run_episode
from .scenarios import SCENARIOS, get_scenario


def _style() -> None:
    if plt is None:
        return
    plt.rcParams.update(
        {
            "figure.dpi": 130,
            "savefig.dpi": 130,
            "font.size": 9,
            "axes.grid": True,
            "grid.alpha": 0.25,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "figure.autolayout": True,
        }
    )


def plot_trajectories(results: dict[str, tuple[EpisodeMetrics, object]], path: pathlib.Path) -> None:
    if plt is None:
        return
    fig, axes = plt.subplots(2, 3, figsize=(10.5, 6.2))
    for ax, scenario in zip(axes.ravel(), SCENARIOS):
        _, trace = results[scenario.key]
        ax.plot(trace.x, trace.y, lw=1.4, color="#1f77b4", label="trajectory")
        ax.plot([trace.x[0]], [trace.y[0]], "o", ms=4, color="#2ca02c", label="start")
        ax.plot([0.0], [0.0], "*", ms=10, color="#d62728", label="target")
        ax.plot([trace.x[-1]], [trace.y[-1]], "s", ms=4, color="#ff7f0e", label="end")
        ax.set_title(scenario.key.replace("_", " "), fontsize=8)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
    axes[0, 0].legend(fontsize=6, loc="best")
    fig.suptitle("Closed-loop trajectories (20 Hz, sensor noise included)", fontsize=11)
    fig.savefig(path)
    plt.close(fig)


def plot_errors(results: dict[str, tuple[EpisodeMetrics, object]], path: pathlib.Path) -> None:
    if plt is None:
        return
    fig, ax = plt.subplots(figsize=(7.6, 3.6))
    for scenario in SCENARIOS:
        _, trace = results[scenario.key]
        ax.plot(trace.time, trace.distance, lw=1.2, label=scenario.key)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("distance to target [m]")
    ax.set_yscale("log")
    ax.axhline(0.15, ls="--", lw=0.8, color="grey")
    ax.text(2, 0.16, "dead-band 0.15 m", fontsize=7, color="grey")
    ax.set_title("Position error over time")
    ax.legend(fontsize=7, ncol=2)
    fig.savefig(path)
    plt.close(fig)


def plot_control(trace, path: pathlib.Path) -> None:
    if plt is None:
        return
    fig, axes = plt.subplots(3, 1, figsize=(8.2, 6.0), sharex=True)
    axes[0].plot(trace.time, trace.distance, color="#1f77b4", lw=1.2)
    axes[0].set_ylabel("error [m]")
    axes[0].set_title("Cross current 0.25 m/s: what the controller is doing")
    axes[1].plot(trace.time, trace.crab_deg, color="#9467bd", lw=1.0)
    axes[1].set_ylabel("crab angle [deg]")
    axes[2].plot(trace.time, trace.thrust_left, lw=1.0, label="left")
    axes[2].plot(trace.time, trace.thrust_right, lw=1.0, label="right")
    axes[2].set_ylabel("thrust [N]")
    axes[2].set_xlabel("time [s]")
    axes[2].legend(fontsize=7)
    fig.savefig(path)
    plt.close(fig)


def plot_compare(path: pathlib.Path) -> None:
    if plt is None:
        return
    keys = ["cross_current", "oblique_current"]
    labels = ["cross 0.25", "oblique 0.25"]
    on_vals, off_vals = [], []
    for key in keys:
        on, _ = run_episode(key, PilotConfig(use_crab_compensation=True))
        off, _ = run_episode(key, PilotConfig(use_crab_compensation=False))
        on_vals.append(on.rms_error)
        off_vals.append(off.rms_error)

    fig, ax = plt.subplots(figsize=(5.2, 3.4))
    idx = range(len(keys))
    ax.bar([i - 0.18 for i in idx], on_vals, width=0.34, label="crab compensation ON", color="#2ca02c")
    ax.bar([i + 0.18 for i in idx], off_vals, width=0.34, label="OFF (heading straight at target)", color="#d62728")
    ax.set_xticks(list(idx))
    ax.set_xticklabels(labels)
    ax.set_ylabel("RMS holding error [m]")
    ax.set_title("Why the crab angle matters")
    ax.legend(fontsize=7)
    for i, (a, b) in enumerate(zip(on_vals, off_vals)):
        ax.text(i - 0.18, a, f"{a:.2f}", ha="center", va="bottom", fontsize=7)
        ax.text(i + 0.18, b, f"{b:.2f}", ha="center", va="bottom", fontsize=7)
    fig.savefig(path)
    plt.close(fig)


def plot_sweep(path: pathlib.Path) -> None:
    if plt is None:
        return
    speeds = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40]
    on_vals, off_vals = [], []
    for speed in speeds:
        from dataclasses import replace

        from ros2_usv_station_keeping.vehicle import VehicleState

        from .scenarios import Scenario
        from .water_world import EnvironmentConfig

        sc = Scenario(
            key="sweep",
            title="sweep",
            summary="",
            start=VehicleState(x=3.0, y=2.0, psi=2.4),
            target=(0.0, 0.0),
            env=EnvironmentConfig(current_x=0.0, current_y=speed, seed=9),
            efficiency=(1.0, 1.0),
            duration=110.0,
        )
        m_on, _ = run_episode(sc, PilotConfig(use_crab_compensation=True), keep_trace=False)
        m_off, _ = run_episode(sc, PilotConfig(use_crab_compensation=False), keep_trace=False)
        on_vals.append(m_on.rms_error)
        off_vals.append(m_off.rms_error)

    fig, ax = plt.subplots(figsize=(5.6, 3.4))
    ax.plot(speeds, on_vals, "o-", lw=1.4, ms=4, label="crab ON")
    ax.plot(speeds, off_vals, "s--", lw=1.2, ms=4, label="crab OFF")
    ax.set_xlabel("cross current [m/s]")
    ax.set_ylabel("RMS holding error [m]")
    ax.set_title("Holding accuracy vs current strength")
    ax.legend(fontsize=7)
    fig.savefig(path)
    plt.close(fig)


def run_all(outdir: pathlib.Path, with_sweep: bool = True) -> dict[str, tuple[EpisodeMetrics, object]]:
    _style()
    outdir.mkdir(parents=True, exist_ok=True)
    results = {sc.key: run_episode(sc) for sc in SCENARIOS}

    plot_trajectories(results, outdir / "traces.png")
    plot_errors(results, outdir / "error_curves.png")
    plot_control(results["cross_current"][1], outdir / "control_signals.png")
    plot_compare(outdir / "crab_compare.png")
    if with_sweep:
        plot_sweep(outdir / "current_sweep.png")
    return results


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="跑场景并生成配图")
    parser.add_argument("--out", default="docs/water/ros2_usv_station_keeping", help="图片输出目录")
    parser.add_argument("--no-sweep", action="store_true", help="跳过参数扫描（较慢）")
    args = parser.parse_args(argv)

    outdir = pathlib.Path(args.out)
    results = run_all(outdir, with_sweep=not args.no_sweep)

    print(f"{'scenario':22s}{'RMS[m]':>9s}{'max[m]':>9s}{'settle[s]':>11s}{'energy':>9s}{'crab[deg]':>11s}{'health':>10s}")
    for key, (m, _) in results.items():
        settle = "inf" if math.isinf(m.settle_time) else f"{m.settle_time:.1f}"
        print(
            f"{key:22s}{m.rms_error:9.3f}{m.max_error:9.2f}{settle:>11s}"
            f"{m.energy:9.1f}{m.mean_crab_deg:11.1f}{m.final_health:>10s}"
        )
    if plt is None:
        print("\n未安装 matplotlib，跳过绘图（指标表照常输出）")
    else:
        print(f"\n图片已写入 {outdir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
