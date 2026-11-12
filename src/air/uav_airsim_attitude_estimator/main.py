#!/usr/bin/env python3
"""统一入口。

    python3 main.py --collect    采集 IMU 窗与真值姿态
    python3 main.py --train      训练姿态估计 MLP
    python3 main.py --demo       实飞演示并录 gif
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

WEIGHTS = "models_trained/attitude.npz"
DATA = "models_trained/attitude_data.npz"


def main():
    ap = argparse.ArgumentParser(description="AirSim IMU 姿态估计（神经网络）")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--collect", action="store_true")
    g.add_argument("--train", action="store_true")
    g.add_argument("--demo", action="store_true")
    ap.add_argument("--episodes", type=int, default=6)
    ap.add_argument("--steps", type=int, default=110)
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--gif", default="demo.gif")
    args = ap.parse_args()

    if args.collect:
        from uav_airsim_attitude_estimator.dataset import collect
        collect(out_path=DATA, episodes=args.episodes, steps=args.steps, seed=args.seed)
    elif args.train:
        from uav_airsim_attitude_estimator.dataset import load
        from uav_airsim_attitude_estimator.train import train
        X, Y = load(DATA)
        print("载入 %d 条样本 %s 标签 %s" % (len(X), X.shape, Y.shape))
        _, m = train(X, Y, epochs=args.epochs, lr=args.lr, out_path=WEIGHTS)
        print("完成：", m)
    else:
        from uav_airsim_attitude_estimator.demo import run_demo
        print("演示统计：", run_demo(weights=WEIGHTS, out_gif=args.gif))


if __name__ == "__main__":
    main()