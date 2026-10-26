#!/usr/bin/env python3
"""pix2cmd — end-to-end driving (camera image -> steering/throttle) with a numpy CNN.

Course entry point. Run without arguments for the full offline demo:

    python main.py --demo

Other modes:
    python main.py --test              run the unit tests
    python main.py --ros               start the ROS 2 node (Humble / Noetic)
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def run_demo(output_dir: str, epochs: int, n_train: int, n_val: int, lr: float, steps: int) -> int:
    from pix2cmd.train import (
        closed_loop_drive,
        plot_predictions,
        plot_samples,
        plot_training,
        save_gif,
        save_report,
        train,
    )

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    print("== 1/4 生成合成道路数据集并训练 CNN ==")
    net, history, (xva, yva) = train(epochs=epochs, n_train=n_train, n_val=n_val, lr=lr)

    print("== 2/4 导出训练曲线与预测散点 ==")
    plot_training(history, out / "training_curve.png")
    plot_predictions(net, xva, yva, out / "predictions.png")
    plot_samples(net, xva, yva, out / "samples.png")

    print("== 3/4 闭环驾驶演示 ==")
    frames, metrics = closed_loop_drive(net, steps=steps)
    gif_bytes, gif_frames = save_gif(frames, out / "drive_demo.gif")

    print("== 4/4 写出指标 ==")
    summary = save_report(history, metrics, out)

    final = history[-1]
    print("\n--- 训练结果 ---")
    print(f"参数数量            : {net.config()['params']}")
    print(f"最终 train MSE      : {final['train_mse']}")
    print(f"验证 MAE (steering) : {final['val_mae_steering']}")
    print(f"验证 MAE (throttle) : {final['val_mae_throttle']}")
    print("\n--- 闭环驾驶 ---")
    for key, value in summary["closed_loop"].items():
        print(f"{key:28}: {value}")
    print(f"\n动图: {out / 'drive_demo.gif'}  ({gif_bytes / 1024:.0f} KB, {gif_frames} 帧)")
    print(f"图  : {out / 'training_curve.png'}")
    print(f"      {out / 'predictions.png'}")
    print(f"      {out / 'samples.png'}")
    print(f"指标: {out / 'metrics.json'}")
    return 0


def run_tests() -> int:
    import pytest

    here = Path(__file__).resolve().parent
    return int(pytest.main(["-q", str(here / "test")]))


def run_ros() -> int:
    from pix2cmd.node import main as node_main

    return node_main()


def main() -> int:
    parser = argparse.ArgumentParser(description="pix2cmd end-to-end driving demo")
    parser.add_argument("--demo", action="store_true", help="离线训练 + 评估 + 导出图与动图（默认）")
    parser.add_argument("--test", action="store_true", help="运行单元测试")
    parser.add_argument("--ros", action="store_true", help="启动 ROS 2 节点")
    parser.add_argument("--headless", action="store_true", help="无显示环境（默认即为离线渲染）")
    parser.add_argument("--output-dir", default="results", help="结果输出目录")
    parser.add_argument("--epochs", type=int, default=12)
    parser.add_argument("--n-train", type=int, default=1200)
    parser.add_argument("--n-val", type=int, default=300)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--steps", type=int, default=120, help="闭环演示步数")
    args = parser.parse_args()

    if args.test:
        return run_tests()
    if args.ros:
        return run_ros()
    return run_demo(args.output_dir, args.epochs, args.n_train, args.n_val, args.lr, args.steps)


if __name__ == "__main__":
    sys.exit(main())
