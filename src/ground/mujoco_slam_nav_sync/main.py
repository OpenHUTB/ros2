#!/usr/bin/env python3
"""mujoco_slam_nav_sync — 课程入口。

    python main.py --demo     训练局部规划网络 + 三种模式对照 + 出图出 GIF（默认）
    python main.py --sweep    定位漂移 × SLAM 开关 的对照实验
    python main.py --test     单元测试
    python main.py --ros      ROS 2 节点（Humble）

Windows 双击 main.bat，Linux 执行 ./main.sh，均等价于 `--demo`。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _fmt_table(agg: dict) -> str:
    head = f"{'配置':<24}{'成功率':>7}{'碰撞':>5}{'ATE均值':>9}{'ATE最大':>9}{'路径':>8}"
    rows = [head]
    for key, v in agg.items():
        rows.append(f"{key:<24}{v['success_rate']*100:>6.0f}%{v['crashes']:>5}"
                    f"{v['ate_mean']:>9.3f}{v['ate_max']:>9.3f}{v['path_mean']:>8.2f}")
    return "\n".join(rows)


def run_demo(out_dir: str, n_train: int, epochs: int) -> int:
    from slam_nav.env import DiffDriveEnv
    from slam_nav.nav import NavConfig, build_truth_grid, run_episode
    from slam_nav.sweep import aggregate, plot_map, plot_sweep, sweep
    from slam_nav.train import plot_training, save_gif, save_report, train

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    env = DiffDriveEnv("maze")

    print("== 1/4 生成专家数据并训练局部规划网络 ==")
    net, history, _ = train(env, n=n_train, epochs=epochs)
    plot_training(history, out / "training_curve.png")
    weights = net.save(out / "planner.npz")
    print(f"参数量 {net.config()['params']}，权重已保存 {weights}")

    print("\n== 2/4 三种模式对照（专家 / 神经网络 × SLAM / 死推算）==")
    cfg = NavConfig(steps=900)
    runs = []
    trials = [
        ("expert + SLAM", dict(net=None, use_slam=True, slip=0.0)),
        ("MLP + SLAM", dict(net=net, use_slam=True, slip=0.0)),
        ("MLP + dead-reckoning", dict(net=net, use_slam=False, slip=0.10)),
    ]
    for name, kw in trials:
        res = run_episode(env, cfg=cfg, record=kw.get("net") is not None and kw["use_slam"], **kw)
        runs.append({"name": name, "success": res.success, "crashed": res.crashed,
                     "steps": res.steps, "ate_rmse": res.ate_rmse,
                     "final_dist": res.final_dist, "path_length": res.path_length,
                     "map_known": res.map_known})
        print(f"[{name:22s}] 成功={res.success} 碰撞={res.crashed} 步数={res.steps} "
              f"ATE={res.ate_rmse:.3f} m 终点误差={res.final_dist:.2f} m")
        if res.frames:
            size, frames = save_gif(res.frames, out / "nav_slam.gif")
            print(f"    动图: {out/'nav_slam.gif'} ({size/1024:.0f} KB, {frames} 帧)")
        if kw["use_slam"] and res.grid is not None:
            plot_map(res.grid, build_truth_grid(env), out / "map_slam.png",
                     traj=[(t[0][0], t[0][1], t[1][0], t[1][1]) for t in res.trace],
                     goal=DiffDriveEnv.GOAL)

    print("\n== 3/4 定位漂移扫描（SLAM vs 死推算）==")
    records = sweep(env, net=net, slips=(0.0, 0.05, 0.10, 0.20), seeds=(0, 1, 2, 3, 4), cfg=cfg)
    agg = aggregate(records)
    plot_sweep(records, out / "sweep.png")

    print("\n== 4/4 指标落盘 ==")
    save_report(history, {"trials": runs, "sweep": records, "aggregate": agg}, out)
    print(_fmt_table(agg))
    print(f"\n指标: {out/'metrics.json'}")
    print(f"图  : {out/'training_curve.png'}  {out/'sweep.png'}  {out/'map_slam.png'}")
    return 0


def run_sweep(out_dir: str, n_train: int, epochs: int) -> int:
    from slam_nav.env import DiffDriveEnv
    from slam_nav.nav import NavConfig
    from slam_nav.sweep import aggregate, plot_sweep, sweep
    from slam_nav.train import save_report, train

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    env = DiffDriveEnv("maze")
    print("== 先训练局部规划网络 ==")
    net, history, _ = train(env, n=n_train, epochs=epochs)
    print("\n== 漂移扫描 ==")
    records = sweep(env, net=net, cfg=NavConfig(steps=900))
    agg = aggregate(records)
    plot_sweep(records, out / "sweep.png")
    save_report(history, {"trials": [], "sweep": records, "aggregate": agg}, out)
    print("\n" + _fmt_table(agg))
    return 0


def run_tests() -> int:
    import pytest

    here = Path(__file__).resolve().parent
    return int(pytest.main(["-q", str(here / "test")]))


def run_ros() -> int:
    from slam_nav.node import main as node_main

    return node_main()


def main() -> int:
    parser = argparse.ArgumentParser(description="MuJoCo EKF-SLAM 与导航同步仿真")
    parser.add_argument("--demo", action="store_true", help="训练 + 对照 + 出图（默认）")
    parser.add_argument("--sweep", action="store_true", help="定位漂移扫描")
    parser.add_argument("--test", action="store_true", help="单元测试")
    parser.add_argument("--ros", action="store_true", help="ROS 2 节点")
    parser.add_argument("--output-dir", default="results")
    parser.add_argument("--n-train", type=int, default=2000)
    parser.add_argument("--epochs", type=int, default=80)
    args = parser.parse_args()

    if args.test:
        return run_tests()
    if args.ros:
        return run_ros()
    if args.sweep:
        return run_sweep(args.output_dir, args.n_train, args.epochs)
    return run_demo(args.output_dir, args.n_train, args.epochs)


if __name__ == "__main__":
    sys.exit(main())
