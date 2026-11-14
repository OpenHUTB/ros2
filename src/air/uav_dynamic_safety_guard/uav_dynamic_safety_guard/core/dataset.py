#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""示范数据采集：几何教师在动态障碍场里闭环飞行，逐帧记录 (观测, 标称, 修正)。

存的是原始量（三帧栅格、速度、目标方向、指令），特征在网络训练时现拼，
这样单帧/时序两种特征、镜像增广都来自同一份数据。
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from .env import FRAME_DT, DynamicEnv, EnvConfig, make_episodes
from .filter import FilterConfig, GeometricGuard
from .samples import DTYPES, KEYS, dataset_summary, empty, mirror_dataset, \
    slice_features

__all__ = ["CollectConfig", "collect_imitation", "mirror_dataset",
           "slice_features", "dataset_summary", "KEYS", "DTYPES"]


@dataclass
class CollectConfig:
    seeds: Sequence[int] = field(default_factory=lambda: tuple(range(16)))
    per_seed: int = 8
    max_time: float = 20.0
    n_static: int = 18
    n_moving: int = 5
    speed: float = 1.8
    max_frames: int = 40000
    teacher: FilterConfig = field(default_factory=FilterConfig)
    verbose: bool = True

    def env(self) -> EnvConfig:
        return EnvConfig(n_static=int(self.n_static),
                         n_moving=int(self.n_moving),
                         max_time=float(self.max_time),
                         speed=float(self.speed))


def collect_imitation(cfg: Optional[CollectConfig] = None
                      ) -> Dict[str, np.ndarray]:
    """闭环跑几何教师，记录示范帧（``u_tgt`` 为教师修正后的指令）。"""
    cfg = cfg or CollectConfig()
    env = DynamicEnv(cfg.env())
    guard = GeometricGuard(env, cfg.teacher)
    specs = make_episodes(cfg.seeds, cfg.per_seed, cfg=cfg.env())
    rows: Dict[str, List[np.ndarray]] = {k: [] for k in KEYS}
    frames = 0
    t0 = time.time()
    steps = int(round(cfg.max_time / FRAME_DT))
    for i, spec in enumerate(specs):
        obs = env.reset(spec)
        guard.reset(obs)
        for _ in range(steps):
            u_nom = env.nominal_action()
            u, diag = guard.act(obs, u_nom)
            if float(obs["h"]) > 0.0:
                rows["hist"].append(np.asarray(obs["hist"], np.float32).copy())
                rows["vel"].append(np.asarray(obs["vel"], np.float32).copy())
                rows["goal_dir"].append(np.asarray(obs["goal_dir"],
                                                   np.float32).copy())
                rows["dist"].append(np.float32(obs["dist"]))
                rows["z"].append(np.float32(obs["pos"][2]))
                rows["u_nom"].append(np.asarray(u_nom, np.float32))
                rows["u_tgt"].append(np.asarray(u, np.float32))
                rows["h"].append(np.float32(obs["h"]))
                rows["q_tgt"].append(np.float32(diag.get("q_sel", np.nan)))
                rows["seed"].append(np.int32(spec.seed))
                rows["episode"].append(np.int32(i))
                frames += 1
            obs, _info, done = env.step(u)
            if done or frames >= cfg.max_frames:
                break
        if cfg.verbose:
            print("  [collect] %2d/%d %s frames=%d (%.1fs)"
                  % (i + 1, len(specs), spec.key(), frames, time.time() - t0),
                  flush=True)
        if frames >= cfg.max_frames:
            break
    env.close()
    if frames == 0:
        return empty(0)
    return {k: np.asarray(rows[k], dtype=DTYPES[k]) for k in KEYS}
