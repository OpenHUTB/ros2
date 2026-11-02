#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""闭环评测：标称控制器 + 不同安全层。"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

from .env import FRAME_DT, EpisodeSpec, SafetyEnv
from .ranges import build_features, corrupt_grid

__all__ = ["run_episode", "benchmark", "summarize", "format_table"]


def _corrupted(obs: Dict[str, np.ndarray], rng, sigma: float,
               drop: float) -> Dict[str, np.ndarray]:
    g = corrupt_grid(obs["ranges"], rng, sigma=sigma, drop=drop)
    feat = build_features(g, obs["vel"], obs["goal_dir"], float(obs["dist"]),
                          float(obs["pos"][2]))
    out = dict(obs)
    out["ranges"] = g
    out["feat"] = feat.astype(np.float32)
    return out


def run_episode(env: SafetyEnv, filt, spec, noise: Optional[dict] = None,
                record: bool = False) -> Dict[str, object]:
    obs = env.reset(spec)
    filt.reset(obs)
    d0 = float(np.linalg.norm(np.asarray(spec.goal, float)
                              - np.asarray(spec.start, float)))
    rng = np.random.default_rng(7000 + int(spec.seed))
    n = int(round(env.cfg.max_time / FRAME_DT))
    hs: List[float] = []
    lat: List[float] = []
    dev: List[float] = []
    tl = {"t": [], "h": [], "h_pred": [], "q_sel": [], "q_nom": [],
          "speed": [], "u": [], "u_nom": [], "pos": []}
    done = False
    info = {"collision": False, "success": False, "time": 0.0, "path": 0.0}
    for k in range(n):
        o = _corrupted(obs, rng, noise["sigma"], noise["drop"]) \
            if noise else obs
        u_nom = env.nominal_action()
        t0 = time.perf_counter()
        u, diag = filt.act(o, u_nom)
        lat.append((time.perf_counter() - t0) * 1e3)
        dev.append(float(np.linalg.norm(np.asarray(u) - u_nom)))
        obs, info, done = env.step(u)
        hs.append(float(info["h"]))
        if record:
            tl["t"].append(k * FRAME_DT)
            tl["h"].append(float(obs["h"]))
            tl["h_pred"].append(float(diag.get("h", np.nan)))
            tl["q_sel"].append(float(diag.get("q_sel", np.nan)))
            tl["q_nom"].append(float(diag.get("q_nom", np.nan)))
            tl["speed"].append(float(info["speed"]))
            tl["u"].append(np.asarray(u, float).copy())
            tl["u_nom"].append(np.asarray(u_nom, float).copy())
            tl["pos"].append(np.asarray(obs["pos"], float).copy())
        if done:
            break
    out = {"filter": filt.name, "episode": spec.key(), "seed": int(spec.seed),
           "collision": bool(info["collision"]),
           "success": bool(info["success"]), "time": float(info["time"]),
           "progress": float(np.clip((d0 - info["dist"]) / max(d0, 1e-6), 0, 1)),
           "path": float(info["path"]), "steps": k + 1,
           "h_min": float(np.min(hs)) if hs else float("nan"),
           "h_mean": float(np.mean(hs)) if hs else float("nan"),
           "latency_ms": float(np.mean(lat)) if lat else float("nan"),
           "dev_mean": float(np.mean(dev)) if dev else 0.0,
           "intervene": float(np.mean(np.asarray(dev) > 0.1)) if dev else 0.0}
    if record:
        out["timeline"] = {k: np.asarray(v) for k, v in tl.items()}
    return out


def benchmark(env: SafetyEnv, filters: Sequence, specs: Sequence[EpisodeSpec],
              noise: Optional[dict] = None) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    for spec in specs:
        env.reseed_scene(int(spec.seed))
        for filt in filters:
            r = run_episode(env, filt, spec, noise=noise)
            rows.append(r)
    return rows


def summarize(rows: Sequence[Dict[str, object]]) -> Dict[str, Dict[str, float]]:
    keys = ("collision", "success", "time", "progress", "path", "h_min",
            "h_mean", "latency_ms", "intervene")
    acc: Dict[str, Dict[str, list]] = {}
    for r in rows:
        d = acc.setdefault(r["filter"], {k: [] for k in keys})
        for k in keys:
            d[k].append(float(r[k]))
    out: Dict[str, Dict[str, float]] = {}
    for name, d in acc.items():
        out[name] = {k: float(np.mean(v)) for k, v in d.items()}
        out[name]["n"] = float(len(d["collision"]))
    return out


def format_table(summary: Dict[str, Dict[str, float]],
                 order: Sequence[str]) -> str:
    head = "%-10s %8s %8s %8s %8s %8s %8s" % (
        "过滤器", "碰撞率", "成功率", "推进率", "最小间距", "路径(m)", "延迟(ms)")
    lines = [head, "-" * 72]
    for name in order:
        if name not in summary:
            continue
        d = summary[name]
        lines.append("%-10s %7.0f%% %7.0f%% %7.0f%% %8.2f %8.1f %8.3f"
                     % (name, 100 * d["collision"], 100 * d["success"],
                        100 * d["progress"], d["h_min"], d["path"],
                        d["latency_ms"]))
    return "\n".join(lines)
