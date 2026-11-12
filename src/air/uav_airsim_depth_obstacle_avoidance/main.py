#!/usr/bin/env python3
"""统一入口。

    python3 main.py --collect                采集专家样本
    python3 main.py --train                  训练网络
    python3 main.py --demo --gif demo.gif    跑一次避障并录 gif
    python3 main.py --eval                   对比 神经网络 / 专家 / 直线
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

WEIGHTS = "models_trained/depth_avoid.npz"
DATA = "models_trained/depth_avoid_data.npz"


def cmd_collect(args):
    from uav_airsim_depth_obstacle_avoidance.dataset import collect
    collect(out_path=DATA, episodes=args.episodes, steps=args.steps, seed=args.seed)


def cmd_train(args):
    from uav_airsim_depth_obstacle_avoidance.cnn import DepthPolicyNet
    from uav_airsim_depth_obstacle_avoidance.dataset import load
    from uav_airsim_depth_obstacle_avoidance.train import train

    X, Y = load(DATA)
    print("载入 %d 条样本，深度 %s 动作 %s" % (len(X), X.shape, Y.shape))
    net, metrics = train(X, Y, epochs=args.epochs, lr=args.lr, out_path=WEIGHTS)
    print("完成：", metrics)


def cmd_demo(args):
    from uav_airsim_depth_obstacle_avoidance.demo import run_demo
    run_demo(weights=WEIGHTS, out_gif=args.gif, steps=args.steps)


def cmd_eval(args):
    from uav_airsim_depth_obstacle_avoidance.demo import run_eval
    run_eval(weights=WEIGHTS, episodes=args.episodes, steps=args.steps)


def main():
    ap = argparse.ArgumentParser(description="AirSim 深度避障（神经网络）")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--collect", action="store_true")
    g.add_argument("--train", action="store_true")
    g.add_argument("--demo", action="store_true")
    g.add_argument("--eval", action="store_true")

    ap.add_argument("--episodes", type=int, default=6)
    ap.add_argument("--steps", type=int, default=120)
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--gif", default="demo.gif")
    args = ap.parse_args()

    if args.collect:
        cmd_collect(args)
    elif args.train:
        cmd_train(args)
    elif args.demo:
        cmd_demo(args)
    elif args.eval:
        cmd_eval(args)


if __name__ == "__main__":
    main()
