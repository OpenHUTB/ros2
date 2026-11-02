#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""闭环评测：oracle/MLP、延迟、横向偏移扫描。"""
from __future__ import annotations

import json
import time
from typing import Dict, Optional

import numpy as np

from .geometry import CAM_OFFSET_BODY, quat_to_mat
from .mlp import TinyMLP
from .train import evaluate_regression

__all__ = ["oracle_policy", "mlp_policy", "run_episode",
           "evaluate_closed_loop", "measure_latency",
           "evaluate_pass_vs_lateral", "evaluate_all"]


def oracle_policy(env):
    return lambda obs: env.oracle_command()


def mlp_policy(model: TinyMLP, env):
    def policy(obs):
        R = quat_to_mat(obs["quat"])
        pred = model.predict(obs["depth"])
        return env.servo.command(pred, R @ CAM_OFFSET_BODY,
                                 float(obs["valid_frac"]), True)
    return policy


def run_episode(env, policy, seed: int, **reset_kwargs) -> Dict:
    obs = env.reset(seed=seed, **reset_kwargs)
    pos = [obs["pos"].copy()]
    cmds = []
    done = False
    info = {}
    while not done:
        a = policy(obs)
        cmds.append(np.asarray(a, float).copy())
        obs, done, info = env.step(a)
        pos.append(obs["pos"].copy())
    out = dict(info)
    out["pos"] = np.asarray(pos)
    out["cmds"] = np.asarray(cmds)
    return out


def evaluate_closed_loop(env, policy, n_episodes: int = 50,
                         seed: int = 0) -> Dict[str, float]:
    reasons = {"pass": 0, "miss": 0, "timeout": 0}
    times = []
    for i in range(int(n_episodes)):
        r = run_episode(env, policy, seed + i)
        reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
        times.append(float(r["time"]))
    n = max(len(times), 1)
    return {
        "episodes": int(n_episodes),
        "pass_rate": reasons["pass"] / n,
        "miss_rate": reasons["miss"] / n,
        "timeout_rate": reasons["timeout"] / n,
        "time_mean": float(np.mean(times)),
        "time_median": float(np.median(times)),
        "time_max": float(np.max(times)),
    }


def measure_latency(model: TinyMLP, sample: np.ndarray, n: int = 1000) -> Dict[str, float]:
    d = np.asarray(sample, np.float32)
    for _ in range(100):
        model.predict(d)
    ts = []
    for _ in range(int(n)):
        t0 = time.perf_counter()
        model.predict(d)
        ts.append((time.perf_counter() - t0) * 1e3)
    return {"latency_mean_ms": float(np.mean(ts)),
            "latency_p95_ms": float(np.percentile(ts, 95))}


def evaluate_pass_vs_lateral(env, policy, offsets=(-1.5, -1.0, -0.5, 0.0,
                                                   0.5, 1.0, 1.5),
                             n_per: int = 10, seed: int = 0) -> Dict[str, float]:
    out = {}
    for off in offsets:
        ok = 0
        for i in range(int(n_per)):
            r = run_episode(env, policy, seed + i, lateral=float(off))
            ok += int(r["reason"] == "pass")
        out["%.1f" % off] = ok / max(n_per, 1)
    return out


def evaluate_all(env, model: TinyMLP, ds, out_path: Optional[str] = None,
                 n_episodes: int = 50, lateral_n: int = 10) -> Dict:
    reg = evaluate_regression(model, ds.val_depth, ds.val_label)
    oracle = evaluate_closed_loop(env, oracle_policy(env), n_episodes, seed=5000)
    mlp = evaluate_closed_loop(env, mlp_policy(model, env), n_episodes, seed=5000)
    lat = measure_latency(model, ds.val_depth[0])
    lateral = evaluate_pass_vs_lateral(env, mlp_policy(model, env),
                                       n_per=lateral_n, seed=7000)
    out = {
        "regression": reg,
        "closed_loop": {"oracle": oracle, "mlp": mlp},
        "latency": lat,
        "pass_vs_lateral": lateral,
        "model_params": model.n_params(),
        "val_samples": int(len(ds.val_depth)),
    }
    if out_path:
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2)
    return out
