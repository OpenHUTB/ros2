#!/usr/bin/env python3
"""mujoco_mppi_neural_tracker — 课程入口。

    python main.py --demo    蒸馏 MPPI + 三种控制器对照 + 出图出 GIF（默认）
    python main.py --bench   控制器 × 感知噪声 的批量对照
    python main.py --test    单元测试
    python main.py --ros     ROS 2 节点

Windows 双击 main.bat，Linux 执行 ./main.sh。
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np
from pathlib import Path


def _table(agg: dict) -> str:
    head = f"{'配置':<12}{'横向RMS':>9}{'RMS最大':>9}{'完成率':>8}{'耗时ms':>9}"
    rows = [head]
    for k, v in agg.items():
        rows.append(f"{k:<12}{v['rms_mean']:>9.3f}{v['rms_max']:>9.3f}"
                    f"{v['completion'] * 100:>7.0f}%{v['ms_mean']:>9.2f}")
    return "\n".join(rows)


def run_demo(out_dir: str, episodes: int, epochs: int) -> int:
    from mppi_nn.env import AckermannEnv
    from mppi_nn.nav import collect_expert, run_episode
    from mppi_nn.net import TrackerNet
    from mppi_nn.sweep import aggregate, plot_sweep, plot_track, sweep
    from mppi_nn.train import dagger, plot_training, save_gif, save_report, train

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    env = AckermannEnv(0)

    print("== 1/5 用 MPPI 当专家采样 ==")
    x, y = collect_expert(env, episodes=episodes, seed=0)
    print(f"样本 {x.shape[0]} 条，输入 {x.shape[1]} 维")

    print("\n== 2/5 模仿学习 ==")
    net = TrackerNet(seed=0)
    hist = train(net, x, y, epochs=epochs)
    print(f"参数量 {net.config()['params']}")

    print("\n== 3/5 DAgger 一轮纠偏 ==")
    xd, yd = dagger(env, net, rounds=1, samples=400)
    hist += train(net, np.vstack([x, xd, xd]), np.vstack([y, yd, yd]), epochs=max(20, epochs // 3))
    weight_path = net.save(out / "policy.npz")
    net.save(pathlib.Path(__file__).resolve().parent / "models" / "policy.npz")
    print(f"权重已写入 {weight_path} 与 models/policy.npz（两者必须一致，已由 --bench 校验）")

    print("\n== 4/5 三种控制器闭环对照 ==")
    results = {}
    for ctrl in ("pp", "mppi", "nn"):
        rec = ctrl == "mppi"          # 只有 MPPI 需要录帧做 gif
        r = run_episode(env, controller=ctrl, noise=0.0, seed=0, net=net, record=rec)
        results[ctrl] = {"rms": r.rms, "max": r.max_err, "completed": r.completed,
                         "steps": r.steps, "ms": r.mean_ms}
        print(f"[{ctrl:4s}] RMS={r.rms:.3f} m  max={r.max_err:.3f} m  "
              f"完成={r.completed}  步数={r.steps}  {r.mean_ms:.2f} ms/步")
        if rec and r.frames:
            size, n = save_gif(r.frames, out / "track_mppi.gif")
            print(f"    动图 {out/'track_mppi.gif'} ({size/1024:.0f} KB, {n} 帧)")
        plot_track(r.trace, env.path, out / f"trace_{ctrl}.png",
                   f"{ctrl.upper()} tracking (RMS {r.rms:.3f} m)")

    print("\n== 5/5 感知噪声扫描 ==")
    runs = sweep(env, net=net, seeds=tuple(range(5)))
    agg = aggregate(runs)
    plot_sweep(runs, out / "sweep.png")
    plot_training(hist, out / "training_curve.png")
    save_report(net.config(), hist,
                {"rollouts": results, "sweep": runs, "aggregate": agg}, agg, out)

    print("\n" + _table(agg))
    print(f"\n产物: {out}")
    return 0


def run_bench(out_dir: str, seeds: int) -> int:
    from mppi_nn.env import AckermannEnv
    from mppi_nn.net import TrackerNet
    from mppi_nn.sweep import aggregate, plot_sweep, sweep
    from mppi_nn.train import save_report

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    env = AckermannEnv(0)
    net = TrackerNet()
    weights = out / "policy.npz"
    if weights.exists():
        net.load(weights)
        print(f"载入权重 {weights}")
    else:
        print("! 没有 policy.npz，nn 分支会使用未训练网络（仅用于对比跑通）")
    runs = sweep(env, net=net, seeds=tuple(range(seeds)))
    agg = aggregate(runs)
    plot_sweep(runs, out / "sweep.png")
    save_report(net.config(), [], {"rollouts": {}, "sweep": runs, "aggregate": agg}, agg, out)
    print("\n" + _table(agg))
    return 0


def run_tests() -> int:
    import pytest
    here = Path(__file__).resolve().parent
    return int(pytest.main(["-q", str(here / "test")]))


def run_ros() -> int:
    from mppi_nn.node import main as node_main
    return node_main()


def main() -> int:
    ap = argparse.ArgumentParser(description="MPPI 与蒸馏神经网络轨迹跟踪对比")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--bench", action="store_true")
    ap.add_argument("--test", action="store_true")
    ap.add_argument("--ros", action="store_true")
    ap.add_argument("--output-dir", default="results")
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--seeds", type=int, default=2)
    a = ap.parse_args()
    if a.test:
        return run_tests()
    if a.ros:
        return run_ros()
    if a.bench:
        return run_bench(a.output_dir, a.seeds)
    return run_demo(a.output_dir, a.episodes, a.epochs)


if __name__ == "__main__":
    sys.exit(main())
