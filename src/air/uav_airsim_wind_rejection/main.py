#!/usr/bin/env python3
"""统一入口。

    python3 main.py --collect                 采集抗风样本
    python3 main.py --train                   训练补偿网络
    python3 main.py --demo --gif demo.gif     有风实飞演示并录 gif
    python3 main.py --baseline                无补偿对照
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

WEIGHTS = "models_trained/wind.npz"
DATA = "models_trained/wind_data.npz"


def cmd_collect(args):
    from uav_airsim_wind_rejection.dataset import collect
    collect(out_path=DATA, episodes=args.episodes, steps=args.steps, seed=args.seed)


def cmd_train(args):
    from uav_airsim_wind_rejection.dataset import load
    from uav_airsim_wind_rejection.train import train

    X, Y, _, _ = load(DATA)
    print("载入 %d 条样本，输入 %s 标签 %s" % (len(X), X.shape, Y.shape))
    net, metrics = train(X, Y, epochs=args.epochs, lr=args.lr, out_path=WEIGHTS)
    print("完成：", metrics)


def cmd_demo(args):
    from uav_airsim_wind_rejection.demo import run_demo
    stats = run_demo(weights=WEIGHTS, out_gif=args.gif, steps=args.steps,
                     wind=(args.wind_x, args.wind_y))
    print("演示统计：", stats)


def cmd_baseline(args):
    from uav_airsim_wind_rejection.demo import run_baseline
    print("对照：", run_baseline(steps=args.steps, wind=(args.wind_x, args.wind_y)))


def main():
    ap = argparse.ArgumentParser(description="AirSim 无人机抗风补偿（神经网络）")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--collect", action="store_true")
    g.add_argument("--train", action="store_true")
    g.add_argument("--demo", action="store_true")
    g.add_argument("--baseline", action="store_true")

    ap.add_argument("--episodes", type=int, default=6)
    ap.add_argument("--steps", type=int, default=100)
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--lr", type=float, default=2e-3)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--gif", default="demo.gif")
    ap.add_argument("--wind_x", type=float, default=6.0)
    ap.add_argument("--wind_y", type=float, default=0.0)
    args = ap.parse_args()

    if args.collect:
        cmd_collect(args)
    elif args.train:
        cmd_train(args)
    elif args.demo:
        cmd_demo(args)
    elif args.baseline:
        cmd_baseline(args)


if __name__ == "__main__":
    main()
