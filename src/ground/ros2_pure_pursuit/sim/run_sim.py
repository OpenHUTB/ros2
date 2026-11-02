#!/usr/bin/env python3
"""离线仿真入口：不依赖 ROS，直接跑通路径跟踪 + 避障并输出图表。

用法示例：
    python3 sim/run_sim.py --path eight
    python3 sim/run_sim.py --path s_curve --speed 1.0 --no-avoid
    python3 sim/run_sim.py --path circle --out results/circle.png
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from ros2_pure_pursuit.obstacle import ObstacleAvoider, synthetic_scan  # noqa: E402
from ros2_pure_pursuit.paths import circle, figure_eight, s_curve, straight  # noqa: E402
from ros2_pure_pursuit.pure_pursuit import PurePursuitController  # noqa: E402
from ros2_pure_pursuit.robot import DiffDriveRobot  # noqa: E402

ROBOT_RADIUS = 0.35   # 机器人等效外接半径 (m)，用于碰撞判定
PATH_BUILDERS = {
    "circle": lambda: circle(radius=8.0),
    "eight": lambda: figure_eight(a=10.0),
    "s_curve": lambda: s_curve(length=24.0, amp=3.0),
    "straight": lambda: straight(length=20.0),
}
def obstacles_along_path(
    path, fractions: list[float], offset: float = 0.80, radius: float = 0.50
) -> list[tuple[float, float, float]]:
    """沿路径法线方向布障：障碍中心距路径 offset，迫使机器人绕行。

    取 offset < radius + ROBOT_RADIUS 时，关闭避障会直接碰撞；
    开启避障后机器人会减速并向空旷侧转向绕过。
    """
    pts = path.points
    obstacles = []
    for f in fractions:
        i = int(f * (len(pts) - 1))
        p, pn = pts[i], pts[min(i + 1, len(pts) - 1)]
        d = pn - p
        n = float(np.linalg.norm(d))
        if n < 1e-9:
            continue
        t = d / n
        normal = np.array([-t[1], t[0]])
        obstacles.append((float(p[0] + normal[0] * offset),
                          float(p[1] + normal[1] * offset),
                          radius))
    return obstacles


def run(path_name: str, speed: float, avoid: bool, max_steps: int = 2000):
    """跑一轮仿真，返回 (指标字典, 轨迹数组, 路径点, 误差序列)。"""
    path = PATH_BUILDERS[path_name]()
    obstacles = obstacles_along_path(path, [0.25, 0.60])

    robot = DiffDriveRobot(dt=0.05)
    # 起点沿路径切线后退 0.3 m，避免初始朝向偏差引入的横向误差
    p0, p1 = path.points[0], path.points[1]
    tan = p1 - p0
    tan = tan / max(float(np.linalg.norm(tan)), 1e-9)
    robot.reset(
        x=float(p0[0] - tan[0] * 0.3),
        y=float(p0[1] - tan[1] * 0.3),
        theta=float(np.arctan2(tan[1], tan[0])),
    )
    ctrl = PurePursuitController()
    avoider = ObstacleAvoider()

    traj, ctes, speeds, fronts, risks = [], [], [], [], []
    min_clearance = float("inf")
    collision = False
    lap_done = False
    traveled = 0.0

    for _ in range(max_steps):
        v_cmd, w_cmd, info = ctrl.compute(path, robot.pose, speed)

        if avoid:
            scan, amin, ainc = synthetic_scan(
                robot.pose.x, robot.pose.y, robot.pose.theta, obstacles
            )
            av = avoider.evaluate(scan, amin, ainc)
            v_cmd *= av.slow_factor
            w_cmd += av.turn_bias
            fronts.append(av.min_front)
            risks.append(av.risk)
            min_clearance = min(min_clearance, av.min_front)
        else:
            fronts.append(float("nan"))
            risks.append(0.0)

        robot.step(v_cmd, w_cmd)
        traj.append((robot.pose.x, robot.pose.y, robot.pose.theta))
        ctes.append(info.cte)
        speeds.append(robot.v)

        # 闭合路径按累计里程判定跑满一圈，避免自交路径上最近点索引跳变误判
        traveled += abs(robot.v) * robot.dt
        if path.closed and traveled >= path.length * 0.98:
            lap_done = True

        for ox, oy, orad in obstacles:
            if np.hypot(robot.pose.x - ox, robot.pose.y - oy) < orad + ROBOT_RADIUS:
                collision = True
        if collision:
            break
        if info.finished or lap_done:
            break

    traj = np.array(traj)
    ctes = np.array(ctes)
    metrics = {
        "steps": len(traj),
        "time": len(traj) * robot.dt,
        "max_cte": float(np.max(np.abs(ctes))) if len(ctes) else 0.0,
        "mean_cte": float(np.mean(np.abs(ctes))) if len(ctes) else 0.0,
        "rms_cte": float(np.sqrt(np.mean(ctes**2))) if len(ctes) else 0.0,
        "min_front": float(min_clearance) if np.isfinite(min_clearance) else float("nan"),
        "mean_speed": float(np.mean(speeds)) if speeds else 0.0,
        "collision": collision,
        "finished": bool(info.finished or lap_done),
    }
    return metrics, traj, path.points, ctes, np.array(risks), obstacles


def plot(path_name: str, traj, path_pts, ctes, risks, obstacles, out: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(16, 4.6))

    ax = axes[0]
    ax.plot(path_pts[:, 0], path_pts[:, 1], "--", color="#888888", lw=1.6, label="reference path")
    ax.plot(traj[:, 0], traj[:, 1], "-", color="#d62728", lw=1.8, label="robot trajectory")
    for ox, oy, orad in obstacles:
        ax.add_patch(plt.Circle((ox, oy), orad, color="#1f77b4", alpha=0.35))
        ax.add_patch(plt.Circle((ox, oy), orad, fill=False, ec="#1f77b4", ls="--"))
    ax.plot(traj[0, 0], traj[0, 1], "o", color="#2ca02c", ms=9, label="start")
    ax.plot(traj[-1, 0], traj[-1, 1], "*", color="#ff7f0e", ms=14, label="end")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"Trajectory - {path_name}")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)

    axes[1].plot(ctes, color="#9467bd", lw=1.5)
    axes[1].axhline(0.0, color="k", lw=0.8)
    axes[1].set_xlabel("step")
    axes[1].set_ylabel("cross-track error (m)")
    axes[1].set_title(f"Tracking error (RMS={np.sqrt(np.mean(ctes**2)):.3f} m)")
    axes[1].grid(alpha=0.3)

    axes[2].plot(risks, color="#e377c2", lw=1.5)
    axes[2].set_xlabel("step")
    axes[2].set_ylabel("obstacle risk 0-1")
    axes[2].set_ylim(-0.05, 1.05)
    axes[2].set_title("Front obstacle risk")
    axes[2].grid(alpha=0.3)

    fig.tight_layout()
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    fig.savefig(out, dpi=140)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description="纯追踪路径跟踪离线仿真")
    ap.add_argument("--path", choices=list(PATH_BUILDERS), default="eight")
    ap.add_argument("--speed", type=float, default=0.9)
    ap.add_argument("--no-avoid", action="store_true", help="关闭避障，用于对照实验")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    out = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "results",
        f"{args.path}{'_noavoid' if args.no_avoid else ''}.png",
    )

    metrics, traj, path_pts, ctes, risks, obstacles = run(
        args.path, args.speed, not args.no_avoid
    )
    plot(args.path, traj, path_pts, ctes, risks, obstacles, out)

    print(f"路径={args.path}  避障={'关' if args.no_avoid else '开'}  期望速度={args.speed} m/s")
    print("-" * 52)
    print(f"  仿真步数        : {metrics['steps']}")
    print(f"  仿真时长        : {metrics['time']:.2f} s")
    print(f"  最大横向误差    : {metrics['max_cte']:.4f} m")
    print(f"  平均横向误差    : {metrics['mean_cte']:.4f} m")
    print(f"  均方根误差(RMS) : {metrics['rms_cte']:.4f} m")
    print(f"  前向最小测距    : {metrics['min_front']:.3f} m")
    print(f"  平均行进速度    : {metrics['mean_speed']:.3f} m/s")
    print(f"  是否发生碰撞    : {'是' if metrics['collision'] else '否'}")
    print(f"  图表已保存      : {os.path.abspath(out)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
