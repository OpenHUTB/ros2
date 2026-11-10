#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统一入口：采集 / 训练 / 评估 / 出图 / 自检。

    python main.py --selftest
    python main.py --collect            --out data/safety_dataset.npz
    python main.py --train              --data data/safety_dataset.npz --out data/safety_net.npz
    python main.py --collect-imitation  --out data/safety_imitation.npz
    python main.py --train-policy       --data data/safety_imitation.npz --out data/safety_policy.npz
    python main.py --evaluate           --model data/safety_policy.npz
    python main.py --figures            --model data/safety_policy.npz --outdir figs/
"""
from __future__ import annotations

import argparse
import json
import os
import time

import numpy as np


def _common(a) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)


def _selftest(a) -> int:
    from .selftest import run
    return 0 if run() else 1


def _collect(a) -> int:
    from .core.dataset import CollectConfig, collect
    d = collect(CollectConfig(seeds=range(a.seeds), per_seed=a.episodes,
                              max_frames=a.frames, max_time=a.max_time,
                              verbose=True))
    np.savez_compressed(a.out, **d)
    print("数据集 %d 帧 → %s" % (len(d["h"]), a.out))
    return 0


def _train(a) -> int:
    from .core.network import NetConfig, SafetyNet
    from .core.training import TrainConfig, evaluate_predictions, train
    d = dict(np.load(a.data, allow_pickle=False))
    net = SafetyNet(NetConfig(hidden=a.hidden, seed=a.seed))
    hist = train(net, d, TrainConfig(epochs=a.epochs, verbose=True))
    print("离线指标：", evaluate_predictions(net, d))
    net.save(a.out, meta={"stage": "critic"})
    with open(a.out + ".json", "w", encoding="utf-8") as f:
        json.dump(hist, f, ensure_ascii=False, indent=1, default=float)
    print("模型 → %s" % a.out)
    return 0


def _collect_imitation(a) -> int:
    from .core.dataset import collect_imitation, mirror_imitation
    d = collect_imitation(range(a.seeds), per_seed=a.episodes,
                          max_time=a.max_time, n_obstacles=a.obstacles,
                          speed=a.speed, verbose=True)
    d = mirror_imitation(d)
    np.savez_compressed(a.out, **d)
    print("示范 %d 帧 → %s" % (len(d["X"]), a.out))
    return 0


def _train_policy(a) -> int:
    from .core.network import PolicyNet
    from .core.training import PolicyTrainConfig, train_policy
    d = dict(np.load(a.data, allow_pickle=False))
    net = PolicyNet(hidden=a.hidden, seed=a.seed)
    hist = train_policy(net, d, PolicyTrainConfig(epochs=a.epochs,
                                                   near_weight=a.near_weight,
                                                   verbose=True))
    net.save(a.out, meta={"stage": "imitation"})
    with open(a.out + ".json", "w", encoding="utf-8") as f:
        json.dump(hist, f, ensure_ascii=False, indent=1, default=float)
    print("策略 → %s" % a.out)
    return 0


def _filters(a, env):
    from .core.filter import (BrakeFilter, FilterConfig, GeometricFilter,
                              LearnedFilter, NoFilter)
    from .core.network import PolicyNet
    out = [NoFilter(), BrakeFilter(1.2), GeometricFilter(env, FilterConfig())]
    if a.model:
        out.append(LearnedFilter(PolicyNet.load(a.model), FilterConfig()))
    return out


def _evaluate(a) -> int:
    from .core.env import EnvConfig, SafetyEnv, make_episodes
    from .core.experiment import benchmark, run_episode, summarize
    env = SafetyEnv(EnvConfig(max_time=a.max_time, n_obstacles=a.obstacles,
                              speed=a.speed))
    specs = make_episodes(range(a.eval_seed, a.eval_seed + a.episodes),
                          per_seed=1, n_obstacles=a.obstacles)
    fs = _filters(a, env)
    rows = benchmark(env, fs, specs)
    summary = summarize(rows)
    print(json.dumps(summary, ensure_ascii=False, indent=1, default=float))
    noise = benchmark(env, fs[-1:], specs,
                      noise={"sigma": a.noise_sigma, "drop": a.noise_drop})
    ok = {r["episode"] for r in rows
          if r["filter"] == "learned" and not r["collision"]}
    bad = [r["episode"] for r in rows
           if r["filter"] == "none" and r["collision"] and r["episode"] in ok]
    tl, tl_none, scene = None, None, None
    if a.model and bad:
        spec = next((s for s in specs if s.key() == bad[0]), specs[0])
        env.reseed_scene(int(spec.seed))
        tl_none = run_episode(env, fs[0], spec, record=True)
        tl = run_episode(env, fs[-1], spec, record=True)
        scene = [{"pos": list(o.pos), "kind": o.kind,
                  "size": list(o.size)} for o in env.scene.objects]
    out = {"summary": summary, "rows": rows, "noise": noise,
           "timeline": tl, "timeline_none": tl_none, "scene": scene}
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=1,
                      default=lambda o: o.tolist()
                      if isinstance(o, np.ndarray) else float(o))
        print("结果 → %s" % a.out)
    return 0


def _figures(a) -> int:
    from .plotting import make_all
    with open(a.results, encoding="utf-8") as f:
        res = json.load(f)
    make_all(a.model, res, a.outdir)
    print("配图 → %s" % a.outdir)
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--collect", action="store_true")
    ap.add_argument("--train", action="store_true")
    ap.add_argument("--collect-imitation", action="store_true")
    ap.add_argument("--train-policy", action="store_true")
    ap.add_argument("--evaluate", action="store_true")
    ap.add_argument("--figures", action="store_true")
    ap.add_argument("--data", default="data/safety_dataset.npz")
    ap.add_argument("--model", default="data/safety_policy.npz")
    ap.add_argument("--results", default="results.json")
    ap.add_argument("--out", default="")
    ap.add_argument("--outdir", default="docs/img/air/uav_neural_safety_filter")
    ap.add_argument("--seeds", type=int, default=16)
    ap.add_argument("--episodes", type=int, default=12)
    ap.add_argument("--frames", type=int, default=160)
    ap.add_argument("--max-time", type=float, default=15.0)
    ap.add_argument("--obstacles", type=int, default=20)
    ap.add_argument("--speed", type=float, default=2.5)
    ap.add_argument("--hidden", type=int, default=128)
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--near-weight", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--eval-seed", type=int, default=100)
    ap.add_argument("--noise-sigma", type=float, default=0.08)
    ap.add_argument("--noise-drop", type=float, default=0.15)
    a = ap.parse_args(argv)
    if a.selftest:
        return _selftest(a)
    if a.collect:
        a.out = a.out or "data/safety_dataset.npz"
        return _collect(a)
    if a.train:
        a.out = a.out or "data/safety_net.npz"
        return _train(a)
    if a.collect_imitation:
        a.out = a.out or "data/safety_imitation.npz"
        return _collect_imitation(a)
    if a.train_policy:
        a.out = a.out or "data/safety_policy.npz"
        return _train_policy(a)
    if a.evaluate:
        a.out = a.out or "results.json"
        return _evaluate(a)
    if a.figures:
        return _figures(a)
    ap.print_help()
    return 0
