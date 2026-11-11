#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一次真实运行的离线分析：指标 JSON/Markdown + 轨迹/误差/速度图 + GIF。"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK JP", "Noto Sans CJK SC", "Droid Sans Fallback",
    "WenQuanYi Zen Hei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

from diff_drive_astar_odom_nav.astar import path_length_world
from diff_drive_astar_odom_nav.map_utils import load_map


def read_csv(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            rows.append({k: float(v) for k, v in row.items()})
    return rows


def column(rows, key):
    return np.array([r[key] for r in rows], dtype=float)


def analyze_run(run_dir):
    run_dir = Path(run_dir)
    traj = read_csv(run_dir / "trajectory.csv")
    plan = read_csv(run_dir / "plan.csv") if (run_dir / "plan.csv").exists() else []
    status = {}
    for name in ("status.json", "run_info.json"):
        p = run_dir / name
        if p.exists():
            status.update(json.loads(p.read_text(encoding="utf-8")))

    t = column(traj, "t")
    gt_x, gt_y = column(traj, "truth_x"), column(traj, "truth_y")
    od_x, od_y = column(traj, "odom_x"), column(traj, "odom_y")
    cross = column(traj, "cross_track")
    odom_cross = column(traj, "odom_cross_track")
    cmd_v, cmd_w = column(traj, "cmd_v"), column(traj, "cmd_w")

    plan_points = [(r["x"], r["y"]) for r in plan]
    planned_length = path_length_world(plan_points) if plan_points else 0.0
    traveled_gt = float(np.sum(np.hypot(np.diff(gt_x), np.diff(gt_y))))
    traveled_odom = float(np.sum(np.hypot(np.diff(od_x), np.diff(od_y))))
    drift = np.hypot(gt_x - od_x, gt_y - od_y)
    valid = np.isfinite(cross)
    cross_valid = cross[valid]
    duration = float(t[-1] - t[0]) if len(t) else 0.0
    goal = plan_points[-1] if plan_points else (0.0, 0.0)
    final_error = float(np.hypot(gt_x[-1] - goal[0], gt_y[-1] - goal[1])) if len(t) else float("nan")

    metrics = {
        "status": status.get("status", "UNKNOWN"),
        "planned_length_m": round(planned_length, 3),
        "traveled_length_m": round(traveled_gt, 3),
        "traveled_odom_length_m": round(traveled_odom, 3),
        "duration_s": round(duration, 2),
        "final_error_m": round(final_error, 3),
        "cross_track_rms_m": round(float(np.sqrt(np.mean(cross_valid ** 2))) if len(cross_valid) else float("nan"), 4),
        "cross_track_mean_abs_m": round(float(np.mean(np.abs(cross_valid))) if len(cross_valid) else float("nan"), 4),
        "cross_track_max_m": round(float(np.max(np.abs(cross_valid))) if len(cross_valid) else float("nan"), 4),
        "odom_drift_final_m": round(float(drift[-1]), 4) if len(drift) else float("nan"),
        "odom_drift_rms_m": round(float(np.sqrt(np.mean(drift ** 2))) if len(drift) else float("nan"), 4),
        "max_cmd_linear_mps": round(float(np.max(cmd_v)), 3),
        "plan_points": len(plan_points),
        "samples": len(traj),
    }
    (run_dir / "metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    write_markdown(run_dir, metrics)
    plot_all(run_dir, traj, plan_points, metrics)
    return metrics


def write_markdown(run_dir, m):
    lines = [
        "# 一次真实运行的关键指标",
        "",
        "| 指标 | 数值 |",
        "| --- | --- |",
        "| 运行状态 | %s |" % m["status"],
        "| 规划路径长度 | %.1f cm / %.3f m |" % (m["planned_length_m"] * 100, m["planned_length_m"]),
        "| 实际行驶长度（Gazebo 真值） | %.3f m |" % m["traveled_length_m"],
        "| 总耗时 | %.2f s |" % m["duration_s"],
        "| 终点位置误差 | %.1f cm |" % (m["final_error_m"] * 100),
        "| 横向误差 RMS / 均值 / 最大 | %.1f / %.1f / %.1f cm |" % (
            m["cross_track_rms_m"] * 100, m["cross_track_mean_abs_m"] * 100,
            m["cross_track_max_m"] * 100),
        "| 里程计漂移 最终 / RMS | %.1f / %.1f cm |" % (
            m["odom_drift_final_m"] * 100, m["odom_drift_rms_m"] * 100),
        "| 最大线速度指令 | %.3f m/s |" % m["max_cmd_linear_mps"],
        "| 路径点 / 采样行数 | %d / %d |" % (m["plan_points"], m["samples"]),
        "",
    ]
    (run_dir / "metrics.md").write_text("\n".join(lines), encoding="utf-8")


def draw_map(ax):
    grid_map = load_map(Path(__file__).resolve().parents[1] / "maps" / "static_map.yaml")
    extent = [grid_map.origin_x, grid_map.origin_x + grid_map.width * grid_map.resolution,
              grid_map.origin_y, grid_map.origin_y + grid_map.height * grid_map.resolution]
    ax.imshow(grid_map.grid, cmap="Greys", origin="lower", extent=extent, alpha=0.85)
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")


def plot_all(run_dir, traj, plan_points, metrics):
    t = column(traj, "t")
    gt_x, gt_y = column(traj, "truth_x"), column(traj, "truth_y")
    od_x, od_y = column(traj, "odom_x"), column(traj, "odom_y")

    # 1) 轨迹对比
    fig, ax = plt.subplots(figsize=(6.4, 6.0))
    draw_map(ax)
    if plan_points:
        px, py = zip(*plan_points)
        ax.plot(px, py, "-", color="#1f77b4", lw=2.0, label="A* 全局路径")
        ax.plot(plan_points[0][0], plan_points[0][1], "o", color="#1f77b4", ms=6, label="起点")
        ax.plot(plan_points[-1][0], plan_points[-1][1], "*", color="#1f77b4", ms=12, label="目标")
    ax.plot(gt_x, gt_y, "-", color="#2ca02c", lw=1.8, label="Gazebo 真值")
    ax.plot(od_x, od_y, "--", color="#ff7f0e", lw=1.4, label="轮式里程计")
    ax.set_title("轨迹对比  RMS 横向误差 %.1f cm  终点误差 %.1f cm"
                 % (metrics["cross_track_rms_m"] * 100, metrics["final_error_m"] * 100))
    ax.legend(loc="lower right", fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(run_dir / "trajectory.png", dpi=160)
    plt.close(fig)

    # 2) 横向误差与里程计漂移
    fig, axes = plt.subplots(2, 1, figsize=(7.2, 5.4), sharex=True)
    axes[0].plot(t, np.abs(column(traj, "cross_track")) * 100, color="#d62728", lw=1.2)
    axes[0].set_ylabel("横向误差 / cm")
    axes[0].set_title("里程计闭环跟踪误差（相对 A* 路径）")
    axes[0].grid(alpha=0.3)
    drift = np.hypot(gt_x - od_x, gt_y - od_y) * 100
    axes[1].plot(t, drift, color="#9467bd", lw=1.2)
    axes[1].set_ylabel("里程计漂移 / cm")
    axes[1].set_xlabel("时间 / s")
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(run_dir / "error.png", dpi=160)
    plt.close(fig)

    # 3) 速度指令
    fig, axes = plt.subplots(2, 1, figsize=(7.2, 4.8), sharex=True)
    axes[0].plot(t, column(traj, "cmd_v"), color="#1f77b4", lw=1.2)
    axes[0].set_ylabel("线速度 / (m/s)")
    axes[0].grid(alpha=0.3)
    axes[1].plot(t, column(traj, "cmd_w"), color="#ff7f0e", lw=1.2)
    axes[1].set_ylabel("角速度 / (rad/s)")
    axes[1].set_xlabel("时间 / s")
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(run_dir / "speed.png", dpi=160)
    plt.close(fig)

    make_gif(run_dir, t, gt_x, gt_y, od_x, od_y, plan_points)


def make_gif(run_dir, t, gt_x, gt_y, od_x, od_y, plan_points):
    """由真实轨迹数据生成动画（不加任何虚构点），控制在 5 MB 以内。"""
    stride = max(1, len(t) // 90)
    fig, ax = plt.subplots(figsize=(5.2, 5.0))
    draw_map(ax)
    if plan_points:
        px, py = zip(*plan_points)
        ax.plot(px, py, "-", color="#1f77b4", lw=1.6, alpha=0.8, label="A* 路径")
    gt_line, = ax.plot([], [], "-", color="#2ca02c", lw=1.8, label="Gazebo 真值")
    od_line, = ax.plot([], [], "--", color="#ff7f0e", lw=1.2, label="里程计")
    robot = ax.add_patch(plt.Circle((gt_x[0], gt_y[0]), 0.22, color="#2ca02c", alpha=0.5))
    title = ax.set_title("t = 0.0 s")
    ax.legend(loc="lower right", fontsize=8)

    frames = list(range(0, len(t), stride))

    def update(i):
        idx = frames[i]
        gt_line.set_data(gt_x[:idx + 1], gt_y[:idx + 1])
        od_line.set_data(od_x[:idx + 1], od_y[:idx + 1])
        robot.center = (gt_x[idx], gt_y[idx])
        title.set_text("t = %.1f s" % t[idx])
        return gt_line, od_line, robot, title

    anim = FuncAnimation(fig, update, frames=len(frames), interval=100, blit=False)
    anim.save(run_dir / "trajectory.gif", writer=PillowWriter(fps=10), dpi=80)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="分析一次 A* + 里程计闭环运行")
    parser.add_argument("run_dir", nargs="?", default="results/latest")
    args = parser.parse_args()
    metrics = analyze_run(args.run_dir)
    print(json.dumps(metrics, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
