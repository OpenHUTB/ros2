"""训练：专家演示 → 模仿学习 → DAgger 纠偏。

专家的输入是**真值位姿**（纯追踪），但网络只在**雷达特征**上学习——这一点是有意的：
策略永远拿不到真值，训练时也不给。
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from . import track
from .env import LatencyEnv
from .nav import (DelayLine, V_REF, features, pure_pursuit, query_pose, run_episode,
                  scan_at)
from .net import LatencyNet


def collect_expert(env: LatencyEnv, episodes: int = 4, rate_hz: float = 20.0,
                   seed0: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """在真值位姿上跑纯追踪，记录 (雷达特征, 专家动作)。"""
    xs, ys = [], []
    rng = np.random.default_rng(seed0 + 11)
    for k in range(episodes):
        env.reset(seed=seed0 + k)
        idx = env.nearest_index(env.pose()[:2])
        w_prev = 0.0
        for _ in range(int(round(20.0 / (1.0 / rate_hz)))):
            pose = env.pose()
            v, _ = env.velocity()
            scan = scan_at(env, pose, rng)
            v_cmd, w_cmd = pure_pursuit(pose, env.path, idx, v_ref=V_REF)
            xs.append(features(scan, v, w_prev))
            ys.append((v_cmd, w_cmd))
            env.step_period(v_cmd, w_cmd, 10)
            idx = env.nearest_index(env.pose()[:2], idx)
            w_prev = w_cmd
    return np.asarray(xs, dtype=np.float32), np.asarray(ys, dtype=np.float32)


def collect_dagger(env: LatencyEnv, net: LatencyNet, comp: str = "model",
                   delay_ms: float = 200.0, rate_hz: float = 20.0, episodes: int = 2,
                   seed0: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """策略自己跑，专家在旁边贴标签（DAgger 的核心）。"""
    xs, ys = [], []
    rng = np.random.default_rng(seed0 + 29)
    dt = 1.0 / rate_hz
    for k in range(episodes):
        env.reset(seed=100 + seed0 + k)
        idx = env.nearest_index(env.pose()[:2])
        ticks = int(round(delay_ms / 1000.0 * rate_hz))
        delay = DelayLine(ticks)
        w_prev = 0.0
        for _ in range(int(round(30.0 / dt))):
            pose = env.pose()
            v, w = env.velocity()
            q = query_pose(env, pose, v, w, delay.pending(), dt, comp)
            feat = features(scan_at(env, q, rng), v, w_prev)
            v_lab, w_lab = pure_pursuit(pose, env.path, idx, v_ref=V_REF)
            v_cmd, w_cmd = net.act(feat)
            xs.append(feat)
            ys.append((v_lab, w_lab))
            v_app, w_app = delay.push((v_cmd, w_cmd))
            env.step_period(v_app, w_app, max(1, int(round(dt / 0.005))))
            idx = env.nearest_index(env.pose()[:2], idx)
            w_prev = w_cmd
        if env.progress() > 0.6:
            break
    return np.asarray(xs, dtype=np.float32), np.asarray(ys, dtype=np.float32)


def train(episodes: int = 4, dagger_rounds: int = 1, epochs: int = 120,
          seed: int = 0, save_to: str | Path | None = None,
          shapes: tuple[str, ...] = track.SHAPES) -> tuple[LatencyNet, dict]:
    """训练一个网络：三条赛道各采若干回合，再补 DAgger。

    注意：每个赛道单独建 env，训练结束后不保留任何 env 状态（不会污染调用方）。
    """
    net = LatencyNet(seed=seed)
    xs, ys = [], []
    for sh in shapes:
        env = LatencyEnv(sh, seed=seed)
        x, y = collect_expert(env, episodes=episodes, seed0=seed)
        xs.append(x)
        ys.append(y)
    x = np.vstack(xs)
    y = np.vstack(ys)
    hist = net.fit(x, y, epochs=epochs, seed=seed)

    dagger_samples = 0
    for r in range(dagger_rounds):
        env = LatencyEnv(shapes[r % len(shapes)], seed=seed)
        xd, yd = collect_dagger(env, net, comp="model", delay_ms=300.0, seed0=seed + r)
        dagger_samples += len(xd)
        x = np.vstack([x, xd, xd])          # 新数据过采样，避免被两千条老数据淹没
        y = np.vstack([y, yd, yd])
        hist = net.fit(x, y, epochs=40, lr=1e-3, seed=seed + 100 + r)

    if save_to is not None:
        net.save(save_to)
    info = {
        "samples": int(len(x)),
        "expert_samples": int(len(x) - dagger_samples * 2),
        "dagger_samples": int(dagger_samples),
        "params": net.n_params(),
        "final_loss": float(hist[-1]),
        "history": [float(h) for h in hist],
    }
    return net, info


def evaluate(net: LatencyNet, shapes: tuple[str, ...] = track.SHAPES,
             delay_ms: float = 0.0, comp: str = "none", rate_hz: float = 20.0,
             seeds: tuple[int, ...] = (0,)) -> dict:
    """在若干赛道/种子上跑闭环，返回汇总指标。"""
    rows = []
    for sh in shapes:
        env = LatencyEnv(sh, seed=0)
        for s in seeds:
            rows.append(run_episode(env, policy=net, comp=comp, delay_ms=delay_ms,
                                    rate_hz=rate_hz, seed=s))
    return {
        "rms": float(np.mean([r["rms"] for r in rows])),
        "max": float(np.mean([r["max"] for r in rows])),
        "success": float(np.mean([r["completed"] for r in rows])),
        "collide_frames": int(np.sum([r["collide_frames"] for r in rows])),
        "min_clear": float(np.min([r["min_clear"] for r in rows])),
        "cpu_ms": float(np.mean([r["mean_cpu_ms"] for r in rows])),
        "runs": rows,
    }
