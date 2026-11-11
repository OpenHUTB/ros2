#!/usr/bin/env python3
"""机型对比：同一套动作序列下，H1 与 H1_2 的速度跟踪、能耗与稳定性指标。

    python scripts/compare_gaits.py
    python scripts/compare_gaits.py --csv saved_models/gait_compare.csv

指标定义（均为仿真真值，无需额外传感器）：

* **速度跟踪 RMSE**：前进阶段机体系 vx 与指令 vx 的均方根误差，越小越贴合指令；
* **偏航跟踪 RMSE**：转向阶段 wz 与指令 wz 的均方根误差；
* **最低骨盆高度**：全程最低点，衡量是否「压腿」「蹲着走」；
* **单位距离能耗 CoT**：``∫Σ|τ_i·q̇_i|dt / (m·g·d)``，无量纲，越小越省力；
* **横漂**：全程横向位移 |y|，反映左右腿是否对称。
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_h1_walk import ROUTINE, Walker, load_specs, routine_command  # noqa: E402

GRAVITY = 9.81


def rmse(values) -> float:
    """均方根误差；没有样本时返回 nan（对应阶段未出现）。"""
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else float("nan")


def rollout(robot: str, specs) -> dict:
    walker = Walker(robot, specs)
    spec = walker.spec
    total = sum(seconds for _, seconds, _, _ in ROUTINE)

    mass = float(np.sum(walker.model.body_mass))
    energy = 0.0
    path_len = 0.0
    lowest = 9.9
    forward_err, yaw_err = [], []
    forward_speed = []
    start_xy = walker.data.qpos[:2].copy()

    while walker.data.time < total:
        cmd, label = routine_command(walker.data.time)
        walker.step(cmd)

        v_body = walker.base_lin_vel_body()
        yaw_rate = float(walker.data.qvel[5])
        path_len += float(np.linalg.norm(v_body)) * walker.control_dt
        if label in ("前进", "后退"):
            forward_err.append(float(v_body[0]) - float(cmd[0]))
        if label == "前进":
            forward_speed.append(float(v_body[0]))
        if label in ("边走边左转", "原地右转"):
            yaw_err.append(yaw_rate - float(cmd[2]))

        # 机械功率：Σ|τ_i · q̇_i|
        tau = walker.data.actuator_force
        qd = walker.data.qvel[6 : 6 + spec.num_actions]
        energy += float(np.sum(np.abs(tau * qd))) * walker.control_dt
        lowest = min(lowest, walker.base_height)

    displacement = float(np.linalg.norm(walker.data.qpos[:2] - start_xy))
    cot = energy / (mass * GRAVITY * path_len) if path_len > 1e-6 else float("inf")
    return {
        "robot": robot,
        "mass_kg": mass,
        "path_m": path_len,
        "net_m": displacement,
        "forward_speed": float(np.mean(forward_speed)) if forward_speed else float("nan"),
        "min_height": lowest,
        "vx_rmse": rmse(forward_err),
        "wz_rmse": rmse(yaw_err),
        "energy_j": energy,
        "cot": cot,
        "drift_y": float(abs(walker.data.qpos[1])),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--csv", type=Path, default=None, help="把指标写入 CSV")
    args = parser.parse_args()

    specs = load_specs()
    rows = [rollout(robot, specs) for robot in specs]

    header = (
        f"{'机型':<6}{'质量':>7}{'路径长':>8}{'前进均速':>9}"
        f"{'最低高度':>9}{'vx RMSE':>9}{'wz RMSE':>9}{'CoT':>7}"
    )
    print()
    print(header)
    print("-" * len(header) * 2)
    for r in rows:
        print(
            f"{r['robot']:<6}{r['mass_kg']:>6.1f}kg{r['path_m']:>7.2f}m"
            f"{r['forward_speed']:>8.2f}m/s"
            f"{r['min_height']:>8.2f}m{r['vx_rmse']:>8.3f}{r['wz_rmse']:>9.3f}{r['cot']:>7.2f}"
        )
    print()

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f"已写入 {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
