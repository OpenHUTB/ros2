#!/usr/bin/env python3
"""统一入口。

    python3 main.py --collect               采集深度-TTC 样本
    python3 main.py --train                 训练 TTC 预测网络
    python3 main.py --demo --gif demo.gif   飞向墙的实飞演示
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

WEIGHTS = "models_trained/ttc.npz"
DATA = "models_trained/ttc_data.npz"


def cmd_collect(args):
    from uav_airsim_ttc_predictor.dataset import collect
    collect(out_path=DATA, episodes=args.episodes, steps=args.steps, seed=args.seed)


def cmd_train(args):
    from uav_airsim_ttc_predictor.dataset import load
    from uav_airsim_ttc_predictor.train import train

    X, Y = load(DATA)
    print("载入 %d 条样本，输入 %s 标签 %s" % (len(X), X.shape, Y.shape))
    net, metrics = train(X, Y, epochs=args.epochs, lr=args.lr, out_path=WEIGHTS)
    print("完成：", metrics)


def cmd_demo(args):
    from uav_airsim_ttc_predictor.demo import run_demo
    print("演示统计：", run_demo(weights=WEIGHTS, out_gif=args.gif,
                                steps=args.steps, speed=args.speed))


def main():
    ap = argparse.ArgumentParser(description="AirSim 无人机 TTC 预测（神经网络）")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--collect", action="store_true")
    g.add_argument("--train", action="store_true")
    g.add_argument("--demo", action="store_true")

    ap.add_argument("--episodes", type=int, default=6)
    ap.add_argument("--steps", type=int, default=80)
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--lr", type=float, default=5e-4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--gif", default="demo.gif")
    ap.add_argument("--speed", type=float, default=1.6)
    args = ap.parse_args()

    if args.collect:
        cmd_collect(args)
    elif args.train:
        cmd_train(args)
    elif args.demo:
        cmd_demo(args)


if __name__ == "__main__":
    main()