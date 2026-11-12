"""命令行入口：``python main.py --<子命令>``。

所有子命令都是"能单独跑"的：没有 ROS、没有 torch 也能跑 ``--selftest``，
训练与评测只需要 numpy + torch，只有 ``--node`` 需要 rclpy。
"""

from __future__ import annotations

import argparse
import sys

DEFAULT_DATASET = "results/allocator_dataset.npz"
DEFAULT_CHECKPOINT = "results/allocator.pt"


def _hint(missing: str, path: str) -> str:
    steps = {
        "dataset": ("python main.py --gen-data results/allocator_dataset.npz",
                    "约 30 s（80 次飞行）"),
        "checkpoint": ("python main.py --train",
                       "约 2 s（40 epoch，纯 CPU）"),
    }
    first, cost = steps.get(missing, ("", ""))
    return (f"找不到{('数据集' if missing == 'dataset' else '权重')}：{path}\n"
            f"先跑这一步（{cost}）：\n    {first}")


def _require(path: str, missing: str) -> None:
    from pathlib import Path

    if Path(path).is_file():
        return
    print(_hint(missing, path), file=sys.stderr)
    raise SystemExit(2)


# --------------------------------------------------------------------------- #
# 子命令
# --------------------------------------------------------------------------- #
def cmd_selftest(args) -> int:
    from .selftest import run_selftest

    return run_selftest(verbose=not args.quiet)


def cmd_gen_data(args) -> int:
    from . import data as D
    from ._compat import load_base

    api = load_base()
    print(f"生成数据：{args.episodes} 次飞行 × {args.seconds} s，"
          f"抽稀步长 {args.stride}")

    def progress(done, total, kind, traj, crashed):
        flag = "坠机" if crashed else "存活"
        print(f"  [{done:3d}/{total}] {kind:<12s} {traj:<7s} {flag}")

    dataset = D.build_dataset(api, n_episodes=args.episodes,
                              seconds=args.seconds, sample_stride=args.stride,
                              seed=args.seed, progress=progress)
    D.save_dataset(args.out, dataset)
    print(f"已保存 {args.out}：{dataset['X'].shape[0]} 个样本，"
          f"{dataset['crashed_episodes']}/{dataset['episodes']} 次坠机")
    return 0


def cmd_train(args) -> int:
    from . import data as D
    from . import train as T

    _require(args.dataset, "dataset")
    cfg = T.TrainConfig(epochs=args.epochs, batch_size=args.batch_size,
                        lr=args.lr, patience=args.patience,
                        seed=args.seed, threads=args.threads or None)
    dataset = D.load_dataset(args.dataset)
    print(f"数据集：{dataset['X'].shape[0]} 个样本 / {dataset['episodes']} 次飞行")
    meta = T.train_residual(args.dataset, args.checkpoint, cfg,
                            verbose=not args.quiet)
    print(f"参数量 {meta['n_parameters']}，最佳 epoch {meta['best_epoch']}，"
          f"验证 MAE {meta['best_val_mae']:.5f}，测试 MAE {meta['test_mae']:.5f}，"
          f"耗时 {meta['train_seconds']} s")
    print(f"检查点已保存 {args.checkpoint}")
    return 0


def cmd_eval(args) -> int:
    from . import data as D
    from . import evaluate as E
    from . import model as M
    from ._compat import load_base

    _require(args.dataset, "dataset")
    _require(args.checkpoint, "checkpoint")
    api = load_base()
    net, meta = M.load_checkpoint(args.checkpoint)
    print(f"神经网络：{meta.get('n_parameters')} 个参数，"
          f"验证 MAE {meta.get('best_val_mae')}")

    rows = E.flight_table(api, trajectories=args.trajectories,
                          seconds=args.seconds, seed=args.seed, net=net)
    print()
    print(E.format_table(rows))
    print()
    print("按模式汇总：")
    print(E.format_dicts(E.aggregate_rows(rows, "模式")))

    if args.json:
        E.save_json(args.json, [r.as_dict() for r in rows])
        print(f"\n明细已保存 {args.json}")
    return 0


def cmd_demo(args) -> int:
    from . import faults as FL
    from . import model as M
    from . import render as R

    _require(args.checkpoint, "checkpoint")
    net = None
    if args.plan != "healthy":
        net, _ = M.load_checkpoint(args.checkpoint)
    plan = {
        "healthy": FL.FaultPlan(kind="healthy"),
        "degradation": FL.FaultPlan(kind="degradation", rotor=args.rotor,
                                    onset=args.onset, severity=args.severity),
        "stuck": FL.FaultPlan(kind="stuck", rotor=args.rotor, onset=args.onset),
        "loss": FL.FaultPlan(kind="loss", rotor=args.rotor, onset=args.onset),
        "dual_loss": FL.FaultPlan(kind="dual_loss", rotor=args.rotor,
                                  rotor2=(args.rotor + 2) % 4, onset=args.onset),
    }[args.plan]
    print(f"录制演示动图：{plan.label()} / {args.trajectory}")
    R.make_altitude_gif(args.out, plan=plan, trajectory=args.trajectory,
                        seconds=args.seconds, fps=args.fps, net=net)
    print(f"已生成 {args.out}")
    return 0


def cmd_node(args) -> int:
    from .ros_node import main as node_main

    return node_main(args.ros_args)


# --------------------------------------------------------------------------- #
# 参数
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="无人机神经容错飞行控制（加权分配 + 神经网络残差）")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--selftest", action="store_true", help="跑自检（不需要 ROS）")
    group.add_argument("--gen-data", nargs="?", const=DEFAULT_DATASET,
                       metavar="OUT.npz", help="生成训练数据")
    group.add_argument("--train", action="store_true", help="训练残差网络")
    group.add_argument("--eval", action="store_true", help="对比四种控制模式")
    group.add_argument("--demo", nargs="?", const="results/mode_comparison.gif",
                       metavar="OUT.gif", help="录演示动图")
    group.add_argument("--node", action="store_true", help="启动 ROS2 节点")

    parser.add_argument("--dataset", default=DEFAULT_DATASET)
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT)
    parser.add_argument("--episodes", type=int, default=80)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--stride", type=int, default=4)
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--patience", type=int, default=8)
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--json", default="results/mode_comparison.json")
    parser.add_argument("--trajectories", nargs="+", default=["hover", "zigzag"])
    parser.add_argument("--trajectory", default="hover")
    parser.add_argument("--plan", default="degradation",
                        choices=["healthy", "degradation", "stuck", "loss",
                                 "dual_loss"])
    parser.add_argument("--rotor", type=int, default=1)
    parser.add_argument("--onset", type=float, default=2.0)
    parser.add_argument("--severity", type=float, default=0.5)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--ros-args", nargs=argparse.REMAINDER, default=[])
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.selftest:
        return cmd_selftest(args)
    if args.gen_data:
        args.out = args.gen_data
        return cmd_gen_data(args)
    if args.train:
        return cmd_train(args)
    if args.eval:
        return cmd_eval(args)
    if args.demo:
        args.out = args.demo
        return cmd_demo(args)
    if args.node:
        return cmd_node(args)
    return 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
