#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""环岛汇入端到端驾驶：相机图像 -> 方向/油门/制动。

对应 Issue #929：训练并评估图像到车辆控制模型。

四种模式（纯 NumPy，无需 CARLA / ROS / TensorFlow）：
  --mode train   合成数据集 + 训练 MLP，保存模型
  --mode eval    在闭环仿真里对比规则基线 vs NN，输出碰撞率/完成率/控制误差
  --mode demo    一键跑通 train+eval 并打印报告（取证用）
  --mode collect 本模块为合成数据，collect 与 train 等价（保留接口）

用法：
  python3 main.py --mode demo --save_dir results
  python3 main.py --mode train --epochs 300 --model_path models/e2e.npz
  python3 main.py --mode eval  --model_path models/e2e.npz
"""

from __future__ import annotations

import argparse
import json
import os

import numpy as np

from .dataset import make_dataset, split_dataset
from .end_to_end import ImageToControl
from .evaluate import evaluate, test_control_error


def do_train(args):
    print(f"[train] 合成数据集 n={args.samples} ...")
    images, labels = make_dataset(n=args.samples, seed=args.seed)
    train_img, train_lab, test_img, test_lab = split_dataset(images, labels, seed=args.seed)
    print(f"[train] 训练集 {len(train_img)}，测试集 {len(test_img)}（8:2 拆分）")

    model = ImageToControl(seed=args.seed)
    losses = model.train(train_img, train_lab, epochs=args.epochs, learning_rate=args.lr)
    print(f"[train] 初始 loss={losses[0]:.4f} -> 最终 loss={losses[-1]:.4f}")

    os.makedirs(os.path.dirname(args.model_path) or ".", exist_ok=True)
    model.save(args.model_path)
    print(f"[train] 模型已保存到 {args.model_path}")

    # 同时打印测试集 MAE
    test_pred = model.predict(test_img)
    mae = np.mean(np.abs(test_pred - test_lab))
    print(f"[train] 测试集动作 MAE = {mae:.4f}")
    return model


def do_eval(args):
    model = ImageToControl.load(args.model_path)
    print("[eval] 闭环对比（规则基线 vs NN，固定车流 24 episode）...")
    result = evaluate(model=model, n_episodes=24, seed=7)
    err = test_control_error(model)
    report = {"closed_loop": result, "test_set_error": err}
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
        with open(os.path.join(args.save_dir, "eval_report.json"), "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"[eval] 报告已写入 {args.save_dir}/eval_report.json")
    return report


def build_parser():
    p = argparse.ArgumentParser(description="环岛汇入端到端驾驶（轻量离线版）")
    p.add_argument("--mode", choices=["train", "eval", "demo", "collect"], default="demo")
    p.add_argument("--epochs", type=int, default=500)
    p.add_argument("--lr", type=float, default=0.02)
    p.add_argument("--samples", type=int, default=800)
    p.add_argument("--seed", type=int, default=11)
    p.add_argument("--model_path", default="models/e2e_roundabout.npz")
    p.add_argument("--save_dir", default="results")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.mode in ("train", "collect"):
        do_train(args)
    elif args.mode == "eval":
        do_eval(args)
    else:  # demo
        print("=" * 60)
        print("离线一键取证：训练 + 评估（无需 CARLA）")
        print("=" * 60)
        do_train(args)
        print()
        do_eval(args)


if __name__ == "__main__":
    main()
