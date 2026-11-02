"""命令行入口：数据生成 / 训练 / 评测 / 演示动图 / ROS 节点 / 自检。

子命令内部都是**延迟导入**，这样：

* ``--selftest`` 在没有 mujoco、没有 ROS 的机器上也能跑一部分；
* ``--eval`` 只需要 torch + numpy，不需要 MuJoCo；
* 只有 ``--gen-data`` / ``--demo`` 才真的需要仿真底座。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

PACKAGE_NAME = "uav_rotor_fault_detector_nn"
DEFAULT_DATA = "results/detector_dataset.npz"
DEFAULT_CHECKPOINT = "results/detector.pt"


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="main.py",
        description="四旋翼桨故障的健康度估计（1D-CNN + GRU）",
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--selftest", action="store_true", help="跑一遍内置自检")
    group.add_argument("--gen-data", dest="gen_data", nargs="?", const=DEFAULT_DATA,
                       metavar="OUT.npz", help="生成仿真数据集（需要 MuJoCo 底座）")
    group.add_argument("--train", action="store_true", help="训练检测网络（需要 torch）")
    group.add_argument("--eval", dest="eval", nargs="?", const="",
                       metavar="SPLIT", help="评测检查点，默认 test")
    group.add_argument("--demo", dest="demo", nargs="?", const="results/detection.gif",
                       metavar="OUT.gif", help="生成故障检测演示动图")
    group.add_argument("--node", action="store_true", help="启动 ROS2 节点（需要 rclpy）")

    # 数据
    parser.add_argument("--episodes", type=int, default=240, help="数据生成时的飞行次数")
    parser.add_argument("--duration", type=float, default=8.0, help="每次飞行时长（秒）")
    parser.add_argument("--window", type=int, default=100, help="滑窗长度（步）")
    parser.add_argument("--stride", type=int, default=10, help="滑窗步长（步）")
    parser.add_argument("--scenes", nargs="+", default=["open_ground", "corridor", "obstacle_field"])
    parser.add_argument("--data", default=DEFAULT_DATA)
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT)

    # 训练
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--lr", type=float, default=2e-3)
    parser.add_argument("--patience", type=int, default=6)
    parser.add_argument("--threads", type=int, default=None)
    parser.add_argument("--seed", type=int, default=0)

    # 评测
    parser.add_argument("--json", default="results/detector_metrics.json")
    parser.add_argument("--csv", default="results/detector_by_fault.csv")

    # 演示
    parser.add_argument("--fault", default="loss", help="演示用的故障类型")
    parser.add_argument("--rotor", type=int, default=0)
    parser.add_argument("--onset", type=float, default=2.0)
    parser.add_argument("--severity", type=float, default=1.0)
    parser.add_argument("--scene", default="open_ground")
    parser.add_argument("--fps", type=int, default=10)

    return parser.parse_args(argv)


# --------------------------------------------------------------------- 子命令
def cmd_selftest(args: argparse.Namespace) -> int:
    from .selftest import run_selftest

    return run_selftest(verbose=True)


def cmd_gen_data(args: argparse.Namespace) -> int:
    from .data import build_dataset, save_dataset
    from ._compat import load_base

    api = load_base()
    out = Path(args.gen_data)
    started = time.time()

    def progress(done: int, total: int, plan) -> None:
        if done == 1 or done % 20 == 0 or done == total:
            print(f"[{done:3d}/{total}] {plan.describe():<28s} 已用 {time.time() - started:5.1f}s",
                  flush=True)

    dataset = build_dataset(
        api,
        n_episodes=args.episodes,
        seed=args.seed,
        scenes=tuple(args.scenes),
        duration=args.duration,
        window=args.window,
        stride=args.stride,
        progress=progress,
    )
    save_dataset(out, dataset)

    n_crash = int(dataset["crashed"].sum())
    print(f"\n样本 {dataset['X'].shape[0]} 条，飞行 {len(dataset['kinds'])} 次，"
          f"其中坠机 {n_crash} 次")
    print(f"数据集已写入 {out}（{out.stat().st_size / 1e6:.2f} MB，用时 {time.time() - started:.1f}s）")
    return 0


def cmd_train(args: argparse.Namespace) -> int:
    from .train import TrainConfig, train_detector

    cfg = TrainConfig(
        epochs=args.epochs,
        batch_size=args.batch_size,
        lr=args.lr,
        patience=args.patience,
        seed=args.seed,
        threads=args.threads,
    )
    train_detector(args.data, args.checkpoint, cfg, verbose=True)
    return 0


def cmd_eval(args: argparse.Namespace) -> int:
    from .evaluate import evaluate_checkpoint, format_report, save_prediction_csv, save_report

    split = args.eval or "test"
    result = evaluate_checkpoint(args.checkpoint, args.data, split)
    print(format_report(result))
    save_report(result, args.json)
    save_prediction_csv(result, args.csv)
    print(f"\n指标已写入 {args.json} 与 {args.csv}")
    return 0


def cmd_demo(args: argparse.Namespace) -> int:
    from .render import make_detection_gif

    out = Path(args.demo)
    started = time.time()
    path = make_detection_gif(
        args.checkpoint,
        out,
        kind=args.fault,
        rotor=args.rotor,
        onset=args.onset,
        severity=args.severity,
        scene=args.scene,
        seed=args.seed,
        duration=args.duration,
        fps=args.fps,
    )
    size = path.stat().st_size / 1e6
    print(f"已生成 {path}（{size:.2f} MB，用时 {time.time() - started:.1f}s）")
    return 0


def cmd_node(args: argparse.Namespace) -> int:
    from .ros_node import main as node_main

    return node_main([])


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    if args.selftest:
        return cmd_selftest(args)
    if args.gen_data is not None:
        return cmd_gen_data(args)
    if args.train:
        return cmd_train(args)
    if args.eval is not None:
        return cmd_eval(args)
    if args.demo is not None:
        return cmd_demo(args)
    if args.node:
        return cmd_node(args)
    return 1


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
