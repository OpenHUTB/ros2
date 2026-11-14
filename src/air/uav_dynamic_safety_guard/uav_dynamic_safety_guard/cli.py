#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统一入口：采集 / 训练 / 评测 / 出图 / 自检。

    python main.py --selftest
    python main.py --collect-imitation --out data/dynamic_imitation.npz
    python main.py --train-policy --data data/dynamic_imitation.npz \
                   --out data/dynamic_policy_seq.npz --temporal 1
    python main.py --train-policy --data data/dynamic_imitation.npz \
                   --out data/dynamic_policy_single.npz --temporal 0
    python main.py --evaluate --out results.json
    python main.py --figures  --results results.json --outdir ../docs/img/air/uav_dynamic_safety_guard
"""
from __future__ import annotations

import argparse
import json
import os
import time

import numpy as np

ORDER = ["none", "brake", "geometric", "neural_seq", "neural_single"]
LABELS = {"none": "无过滤", "brake": "刹车", "geometric": "几何教师",
          "neural_single": "单帧神经层", "neural_seq": "时序神经层"}


def _mkdir(path: str) -> None:
    d = os.path.dirname(os.path.abspath(path))
    if d:
        os.makedirs(d, exist_ok=True)


def _selftest(a) -> int:
    from .selftest import run
    ok, report = run()
    print(report)
    return 0 if ok else 1


def _collect_imitation(a) -> int:
    from .core.dataset import (CollectConfig, collect_imitation,
                               dataset_summary)
    from .core.filter import FilterConfig
    cfg = CollectConfig(seeds=tuple(range(int(a.seeds))),
                        per_seed=int(a.episodes), max_time=float(a.max_time),
                        speed=float(a.speed), max_frames=int(a.frames),
                        teacher=FilterConfig(margin=float(a.teacher_margin)),
                        verbose=True)
    d = collect_imitation(cfg)
    if int(a.mirror):
        from .core.dataset import mirror_dataset
        d = mirror_dataset(d)
        print("镜像增广后 %d 帧" % len(d["seed"]))
    _mkdir(a.out)
    np.savez_compressed(a.out, **d)
    print("示范数据 → %s（%s）" % (a.out, dataset_summary(d)))
    return 0


def _train_policy(a) -> int:
    from .core.network import GuardNet, NetConfig, SEQ_IN, SINGLE_IN
    from .core.training import TrainConfig, build_xy, offline_metrics, train_policy
    d = dict(np.load(a.data, allow_pickle=False))
    temporal = bool(int(a.temporal))
    in_dim = SEQ_IN if temporal else SINGLE_IN
    net = GuardNet(NetConfig(in_dim=in_dim, hidden=int(a.hidden),
                             seed=int(a.seed),
                             tag="seq" if temporal else "single"))
    print(net.summary())
    X, Y = build_xy(d, temporal=temporal)
    t0 = time.time()
    hist = train_policy(net, X, Y, d["episode"],
                        TrainConfig(epochs=int(a.epochs), lr=float(a.lr),
                                    seed=int(a.seed), temporal=temporal,
                                    verbose=True))
    print("训练 %.1fs，最优验证 MSE %.4f" % (time.time() - t0,
                                             hist["best_val"][0]))
    print("离线指标：", offline_metrics(net, X, Y))
    _mkdir(a.out)
    net.save(a.out, meta={"stage": "policy", "temporal": temporal,
                          "frames": int(len(d["seed"])),
                          "epochs": int(a.epochs)})
    with open(a.out + ".json", "w", encoding="utf-8") as f:
        json.dump({k: [float(x) for x in v] for k, v in hist.items()}, f,
                  ensure_ascii=False, indent=1)
    print("模型 → %s" % a.out)
    return 0


def _build_filters(env, a, want_models: bool = True):
    from .core.filter import (BrakeFilter, FilterConfig, GeometricGuard,
                              NeuralGuard, NoFilter)
    from .core.network import GuardNet
    cfgs = FilterConfig()
    filters = [NoFilter(), BrakeFilter(), GeometricGuard(env, cfgs)]
    if want_models:
        if os.path.exists(a.model):
            filters.append(NeuralGuard(GuardNet.load(a.model), cfgs, "seq"))
        if os.path.exists(a.model_single):
            filters.append(NeuralGuard(GuardNet.load(a.model_single), cfgs,
                                       "single"))
    return filters


def _eval_block(env, a, specs, noise=None) -> dict:
    from .core.experiment import benchmark, format_table, summarize
    filters = _build_filters(env, a)
    rows = benchmark(env, filters, specs, noise=noise)
    summary = summarize(rows)
    order = [f.name for f in filters]
    print(format_table(summary, order))
    return {"rows": rows, "summary": summary, "order": order}


def _evaluate(a) -> int:
    from .core.env import DynamicEnv, EnvConfig, make_episodes
    cfg = EnvConfig(max_time=float(a.max_time), speed=float(a.speed))
    env = DynamicEnv(cfg)
    seeds = list(range(int(a.eval_seed), int(a.eval_seed) + int(a.eval_episodes)))
    dyn = make_episodes(seeds, 1, cfg=cfg, n_moving=int(a.moving))
    stat = make_episodes(seeds, 1, cfg=cfg, n_moving=0)
    out = {"config": {"eval_seed": int(a.eval_seed), "episodes": len(seeds),
                      "max_time": float(a.max_time), "moving": int(a.moving),
                      "models": {"seq": a.model, "single": a.model_single}},
           "generated": time.strftime("%Y-%m-%d %H:%M:%S")}
    print("== 动态场景（%d 个运动障碍）==" % int(a.moving))
    out["dynamic"] = _eval_block(env, a, dyn)
    print("== 静态场景（消融对照）==")
    out["static"] = _eval_block(env, a, stat)
    if float(a.noise_sigma) > 0 or float(a.noise_drop) > 0:
        noise = {"sigma": float(a.noise_sigma), "drop": float(a.noise_drop)}
        print("== 动态场景 + 雷达噪声 %.2f m / 丢点 %.0f%% =="
              % (noise["sigma"], 100 * noise["drop"]))
        out["noise"] = _eval_block(env, a, dyn, noise=noise)
    _mkdir(a.out)
    with open(a.out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1, default=float)
    print("评测结果 → %s" % a.out)
    return 0


def _figures(a) -> int:
    from .plotting import make_all
    with open(a.results, "r", encoding="utf-8") as f:
        results = json.load(f)
    outdir = a.outdir or os.path.join("..", "docs", "img", "air",
                                      "uav_dynamic_safety_guard")
    os.makedirs(outdir, exist_ok=True)
    paths = make_all(results, a.model, a.model_single, outdir,
                     eval_seed=int(a.eval_seed))
    for p in paths:
        print("图 → %s" % p)
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="uav_dynamic_safety_guard",
        description="动态障碍下的时序神经安全层：采集、训练、评测与出图")
    g = p.add_argument_group("动作")
    g.add_argument("--selftest", action="store_true", help="离线自检（只需 numpy）")
    g.add_argument("--collect-imitation", dest="collect_imitation",
                   action="store_true", help="几何教师闭环示范数据")
    g.add_argument("--train-policy", dest="train_policy", action="store_true",
                   help="行为克隆训练安全层")
    g.add_argument("--evaluate", action="store_true", help="闭环评测")
    g.add_argument("--figures", action="store_true", help="从评测结果出图")

    d = p.add_argument_group("数据与模型")
    d.add_argument("--data", default="data/dynamic_imitation.npz")
    d.add_argument("--model", default="data/dynamic_policy_seq.npz")
    d.add_argument("--model-single", dest="model_single",
                   default="data/dynamic_policy_single.npz")
    d.add_argument("--out", "-o", default="results.json")
    d.add_argument("--results", default="results.json")
    d.add_argument("--outdir", default=None)

    c = p.add_argument_group("采集与训练参数")
    c.add_argument("--seeds", type=int, default=16, help="训练场景数 (0..n-1)")
    c.add_argument("--episodes", type=int, default=4, help="每个场景的回合数")
    c.add_argument("--frames", type=int, default=40000)
    c.add_argument("--mirror", type=int, default=1, help="1 开启左右镜像增广")
    c.add_argument("--teacher-margin", dest="teacher_margin", type=float,
                   default=1.0, help="示范时几何教师的间距余量 (m)")
    c.add_argument("--temporal", type=int, default=1, help="1 时序 332 维 / 0 单帧 116 维")
    c.add_argument("--hidden", type=int, default=192)
    c.add_argument("--epochs", type=int, default=120)
    c.add_argument("--lr", type=float, default=2e-3)
    c.add_argument("--seed", type=int, default=0)

    e = p.add_argument_group("评测参数")
    e.add_argument("--eval-seed", dest="eval_seed", type=int, default=100)
    e.add_argument("--eval-episodes", dest="eval_episodes", type=int, default=12)
    e.add_argument("--max-time", dest="max_time", type=float, default=20.0)
    e.add_argument("--speed", type=float, default=1.8)
    e.add_argument("--moving", type=int, default=5)
    e.add_argument("--noise-sigma", dest="noise_sigma", type=float, default=0.08)
    e.add_argument("--noise-drop", dest="noise_drop", type=float, default=0.15)
    return p


def main(argv=None) -> int:
    p = build_parser()
    a = p.parse_args(argv)
    if a.selftest:
        return _selftest(a)
    if a.collect_imitation:
        return _collect_imitation(a)
    if a.train_policy:
        return _train_policy(a)
    if a.evaluate:
        return _evaluate(a)
    if a.figures:
        return _figures(a)
    p.print_help()
    return 2
