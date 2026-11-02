#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""安全飞行数据集：随机指令下的 (观测, 动作, 当前间距, 一步后间距)。"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

from .env import FRAME_DT, SafetyEnv, EnvConfig, make_episodes
from .geometry import rollout_clearance_many
from .ranges import (MAX_RANGE, N_AZ, N_EL, build_features, direction_range,
                     lidar_lower_bound)

__all__ = ["CollectConfig", "collect", "collect_imitation", "mirror_imitation",
           "dataset_features", "dataset_actions", "dataset_q_inputs",
           "dataset_q_labels", "dataset_ranges"]


@dataclass
class CollectConfig:
    seeds: Sequence[int] = tuple(range(10))
    per_seed: int = 12
    max_frames: int = 160
    max_time: float = 8.0
    near_frac: float = 0.25
    seed: int = 0
    verbose: bool = True


def _sample_action(rng: np.random.Generator, u_nom: np.ndarray,
                   mode: int) -> np.ndarray:
    if mode == 0:
        return np.asarray(u_nom, float).copy()
    if mode == 2:
        return np.array([rng.uniform(-1.0, 3.0), rng.uniform(-2.0, 2.0),
                         rng.uniform(-1.5, 1.5)])
    if rng.random() < 0.08:
        return np.zeros(3)
    ang = np.deg2rad(rng.uniform(-60.0, 60.0))
    sp = rng.uniform(0.3, 3.0)
    return np.array([sp * math.cos(ang), sp * math.sin(ang),
                     rng.uniform(-1.0, 1.0)])


def collect(cfg: Optional[CollectConfig] = None) -> Dict[str, np.ndarray]:
    cfg = cfg or CollectConfig()
    env = SafetyEnv(EnvConfig(max_time=cfg.max_time))
    rng = np.random.default_rng(cfg.seed)
    cols: Dict[str, List[np.ndarray]] = {k: [] for k in
                                         ("ranges", "vel", "goal_dir", "dist",
                                          "z", "action", "h", "q",
                                          "ep", "seed")}
    ep_id = 0
    for seed in cfg.seeds:
        env.reseed_scene(int(seed))
        specs = make_episodes([int(seed)], per_seed=cfg.per_seed,
                              near_frac=cfg.near_frac)
        n0 = ep_id
        for spec in specs:
            mode = int(rng.choice([1, 2], p=[0.7, 0.3]))
            obs = env.reset(spec)
            for _ in range(cfg.max_frames):
                u = _sample_action(rng, env.nominal_action(), mode)
                c, s = math.cos(float(obs["yaw"])), math.sin(float(obs["yaw"]))
                Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
                v_w = Rz @ obs["vel"]
                h = rollout_clearance_many(
                    env.scene, obs["pos"], v_w[None], float(obs["yaw"]),
                    obs["vel"][None])[0]
                q = rollout_clearance_many(
                    env.scene, obs["pos"], v_w[None], float(obs["yaw"]),
                    u[None])[0]
                obs2, info, done = env.step(u)
                cols["ranges"].append(np.rint(obs["ranges"] / MAX_RANGE
                                              * 255.0).astype(np.uint8))
                cols["vel"].append(obs["vel"].astype(np.float32))
                cols["goal_dir"].append(obs["goal_dir"].astype(np.float32))
                cols["dist"].append(np.float32(obs["dist"]))
                cols["z"].append(np.float32(obs["pos"][2]))
                cols["action"].append(np.asarray(u, np.float32))
                cols["h"].append(np.float32(h))
                cols["q"].append(np.float32(q))
                cols["ep"].append(np.int32(ep_id))
                cols["seed"].append(np.int32(seed))
                obs = obs2
                if done:
                    break
            ep_id += 1
        if cfg.verbose:
            print("  seed %2d：%2d 集，累计 %d 帧"
                  % (seed, ep_id - n0, len(cols["h"])))
    env.close()
    out = {k: np.asarray(v) for k, v in cols.items()}
    out["ranges"] = out["ranges"].reshape(-1, N_EL, N_AZ)
    out["meta"] = np.asarray("%d seeds, %d episodes" % (len(cfg.seeds), ep_id))
    return out


def collect_imitation(seeds: Sequence[int], per_seed: int = 5,
                      max_time: float = 15.0, n_obstacles: int = 20,
                      speed: float = 2.5,
                      verbose: bool = True) -> Dict[str, np.ndarray]:
    """几何安全层闭环飞行，记录 (观测, 标称指令) → 修正指令。"""
    from .filter import FilterConfig, GeometricFilter
    env = SafetyEnv(EnvConfig(max_time=max_time, n_obstacles=n_obstacles,
                              speed=speed))
    X: List[np.ndarray] = []
    Y: List[np.ndarray] = []
    ep_ids: List[int] = []
    ep = 0
    for seed in seeds:
        env.reseed_scene(int(seed))
        specs = make_episodes([int(seed)], per_seed=per_seed,
                              n_obstacles=n_obstacles)
        for spec in specs:
            obs = env.reset(spec)
            filt = GeometricFilter(env, FilterConfig(progress=0.0))
            filt.reset(obs)
            done = False
            while not done:
                u_nom = env.nominal_action()
                u, _ = filt.act(obs, u_nom)
                X.append(np.concatenate([obs["feat"], u_nom]).astype(np.float32))
                Y.append(np.asarray(u, np.float32))
                ep_ids.append(ep)
                obs, _info, done = env.step(u)
            ep += 1
        if verbose:
            print("  seed %2d：累计 %d 帧" % (seed, len(X)))
    env.close()
    return {"X": np.asarray(X), "Y": np.asarray(Y),
            "ep": np.asarray(ep_ids, np.int32)}


def mirror_imitation(d: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
    """左右镜像增广：距离栅格按方位翻转，横向速度/目标/指令同时取反。"""
    X = d["X"].copy()
    Y = d["Y"].copy()
    g = X[:, :N_EL * N_AZ].reshape(-1, N_EL, N_AZ)
    X[:, :N_EL * N_AZ] = np.roll(g[:, :, ::-1], 1, axis=2).reshape(len(X), -1)
    X[:, N_EL * N_AZ + 1] *= -1          # vel_y
    X[:, N_EL * N_AZ + 4] *= -1          # goal_dir_y
    X[:, -2] *= -1                       # nominal vy
    Y[:, 1] *= -1
    ep = d["ep"] + int(d["ep"].max()) + 1
    return {"X": np.concatenate([d["X"], X]),
            "Y": np.concatenate([d["Y"], Y]),
            "ep": np.concatenate([d["ep"], ep])}


def dataset_ranges(d: Dict[str, np.ndarray]) -> np.ndarray:
    return d["ranges"].astype(np.float32) / 255.0 * MAX_RANGE


def dataset_features(d: Dict[str, np.ndarray]) -> np.ndarray:
    g = dataset_ranges(d)
    return np.stack([build_features(g[i], d["vel"][i], d["goal_dir"][i],
                                    float(d["dist"][i]), float(d["z"][i]))
                     for i in range(len(g))]).astype(np.float32)


def dataset_actions(d: Dict[str, np.ndarray]) -> np.ndarray:
    return d["action"].astype(np.float32)


def dataset_q_inputs(d: Dict[str, np.ndarray]) -> np.ndarray:
    """q 头的输入：指令 3 维 + 指令方向的激光距离 2 维。"""
    g = dataset_ranges(d)
    out = np.zeros((len(g), 5), np.float32)
    for i in range(len(g)):
        out[i, :3] = d["action"][i]
        out[i, 3:] = direction_range(g[i], d["action"][i])
    return out


def dataset_q_labels(d: Dict[str, np.ndarray], horizon: float = 0.6
                     ) -> np.ndarray:
    """q 头拟合的残差：精确扫掠间距 − 激光下界。"""
    g = dataset_ranges(d)
    q = d["q"].astype(np.float64)
    for i in range(len(g)):
        q[i] -= lidar_lower_bound(g[i], d["action"][i], horizon)
    return q
