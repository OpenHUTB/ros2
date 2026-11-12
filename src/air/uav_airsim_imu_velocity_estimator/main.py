#!/usr/bin/env python3
"""统一入口。

    python3 main.py --collect                采集 IMU-速度样本
    python3 main.py --train                  训练 MLP
    python3 main.py --demo --gif demo.gif    实飞估计并录 gif
    python3 main.py --eval                   三轴误差评估
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

WEIGHTS = "models_trained/imu_vel.npz"
DATA = "models_trained/imu_vel_data.npz"


def cmd_collect(args):
    from uav_airsim_imu_velocity_estimator.dataset import collect
    collect(out_path=DATA, episodes=args.episodes, steps=args.steps, seed=args.seed)


def cmd_train(args):
    from uav_airsim_imu_velocity_estimator.dataset import load
    from uav_airsim_imu_velocity_estimator.train import train

    X, Y, mean, std = load(DATA)
    print("载入 %d 条样本，输入 %s 标签 %s" % (len(X), X.shape, Y.shape))
    net, metrics = train(X, Y, epochs=args.epochs, lr=args.lr, out_path=WEIGHTS)
    print("完成：", metrics)


def cmd_demo(args):
    from uav_airsim_imu_velocity_estimator.demo import run_demo
    stats, _, _ = run_demo(weights=WEIGHTS, out_gif=args.gif, steps=args.steps)
    print("演示统计：", stats)


def cmd_eval(args):
    from uav_airsim_imu_velocity_estimator.demo import run_eval
    print("评估：", run_eval(weights=WEIGHTS, steps=args.steps))


def main():
    ap = argparse.ArgumentParser(description="AirSim IMU 速度估计（神经网络）")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--collect", action="store_true")
    g.add_argument("--train", action="store_true")
    g.add_argument("--demo", action="store_true")
    g.add_argument("--eval", action="store_true")

    ap.add_argument("--episodes", type=int, default=6)
    ap.add_argument("--steps", type=int, default=120)
    ap.add_argument("--epochs", type=int, default=100)
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
