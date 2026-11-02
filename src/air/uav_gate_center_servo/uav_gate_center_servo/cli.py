#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令行入口：自检 / 采集 / 训练 / 评测 / 出图。"""
from __future__ import annotations

import argparse
import json
import os
import sys

__all__ = ["main"]


def _root() -> str:
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _default(name: str) -> str:
    return os.path.join(_root(), "data", name)


def cmd_selftest(_args) -> int:
    from .selftest import run_selftest
    return run_selftest()


def cmd_collect(a) -> int:
    from .core.dataset import collect_dataset
    from .core.env import EnvConfig, GateServoEnv
    env = GateServoEnv(EnvConfig(scene_seed=int(a.scene_seed)))
    try:
        ds = collect_dataset(env, episodes=int(a.episodes), seed=int(a.seed),
                             verbose=True)
    finally:
        env.close()
    out = a.out or _default("gate_servo_dataset.npz")
    ds.save(out)
    print("数据集 %d 样本（train %d / val %d）→ %s"
          % (len(ds), len(ds.train_depth), len(ds.val_depth), out))
    return 0


def cmd_train(a) -> int:
    from .core.dataset import Dataset
    from .core.train import TrainConfig, train_mlp
    ds = Dataset.load(a.data or _default("gate_servo_dataset.npz"))
    model, hist = train_mlp(ds, TrainConfig(epochs=int(a.epochs),
                                            seed=int(a.seed)), verbose=True)
    out = a.out or _default("gate_servo.npz")
    hp = a.hist or _default("train_hist.json")
    model.save(out, {"train_samples": int(len(ds.train_depth)),
                     "val_samples": int(len(ds.val_depth)),
                     "best_epoch": int(hist["best_epoch"]),
                     "best_val_distance": float(hist["best_val_distance"])})
    with open(hp, "w", encoding="utf-8") as f:
        json.dump(hist, f, ensure_ascii=False, indent=2)
    print("模型 %s（%d 参数） 历史 %s" % (out, model.n_params(), hp))
    return 0


def cmd_evaluate(a) -> int:
    from .core.dataset import Dataset
    from .core.env import EnvConfig, GateServoEnv
    from .core.mlp import TinyMLP
    from .core.runner import evaluate_all
    ds = Dataset.load(a.data or _default("gate_servo_dataset.npz"))
    model = TinyMLP.load(a.model or _default("gate_servo.npz"))
    env = GateServoEnv(EnvConfig())
    try:
        out = evaluate_all(env, model, ds,
                           a.out or _default("results.json"),
                           n_episodes=int(a.episodes),
                           lateral_n=int(a.lateral_n))
    finally:
        env.close()
    print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0


def cmd_figures(a) -> int:
    from .core.dataset import Dataset
    from .core.env import EnvConfig, GateServoEnv
    from .core.mlp import TinyMLP
    from . import plotting
    ds = Dataset.load(a.data or _default("gate_servo_dataset.npz"))
    model = TinyMLP.load(a.model or _default("gate_servo.npz"))
    hp = a.hist or _default("train_hist.json")
    with open(hp, "r", encoding="utf-8") as f:
        hist = json.load(f)
    env = GateServoEnv(EnvConfig())
    try:
        plotting.make_all(env, model, ds, hist,
                          a.outdir or os.path.join(_root(), "..", "..", "..",
                                                   "docs", "img", "air",
                                                   "uav_gate_center_servo"))
    finally:
        env.close()
    return 0


def cmd_all(a) -> int:
    a.out = _default("gate_servo_dataset.npz")
    a.hist = _default("train_hist.json")
    sc = cmd_collect(a)
    if sc:
        return sc
    a.data = a.out
    a.out = _default("gate_servo.npz")
    a.hist = _default("train_hist.json")
    sc = cmd_train(a)
    if sc:
        return sc
    a.model = a.out
    a.out = _default("results.json")
    sc = cmd_evaluate(a)
    if sc:
        return sc
    a.hist = _default("train_hist.json")
    a.outdir = a.outdir or os.path.join(_root(), "..", "..", "..", "docs",
                                        "img", "air", "uav_gate_center_servo")
    return cmd_figures(a)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="uav_gate_center_servo")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--selftest", action="store_true", help="数值自检")
    g.add_argument("--collect", action="store_true", help="用 oracle 采集数据集")
    g.add_argument("--train", action="store_true", help="训练小 MLP")
    g.add_argument("--evaluate", action="store_true", help="闭环/回归评测")
    g.add_argument("--figures", action="store_true", help="生成文档配图")
    g.add_argument("--all", action="store_true", help="采集+训练+评测+出图")
    p.add_argument("--episodes", type=int, default=55)
    p.add_argument("--lateral-n", type=int, default=10)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--scene-seed", type=int, default=16)
    p.add_argument("--data", default="")
    p.add_argument("--model", default="")
    p.add_argument("--out", default="")
    p.add_argument("--hist", default="")
    p.add_argument("--outdir", default="")
    p.add_argument("--epochs", type=int, default=200)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.selftest:
        return cmd_selftest(args)
    if args.collect:
        return cmd_collect(args)
    if args.train:
        return cmd_train(args)
    if args.evaluate:
        return cmd_evaluate(args)
    if args.figures:
        return cmd_figures(args)
    if args.all:
        return cmd_all(args)
    build_parser().print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
