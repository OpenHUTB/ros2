#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PPO：裁剪式策略梯度 + GAE。

实现要点：

* **tanh 的雅可比**：动作是 ``a = tanh(raw)``，对数概率减去 ``Σ log(1−a²)``；
  漏掉这一项时策略往边界顶，动作长期贴着 ±1。
* **优势按 mini-batch 归一化**，不是整批归一化——整批归一化会把"这一批整体
  都好"的基线信息也减掉，价值头学不到东西。
* **GRU 的隐状态用采样时存下来的**，梯度不跨时间回传（截断 BPTT）。
  与 VIO 前端同一处理：携带长期信息的是隐状态本身，不是它的梯度。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .nn import Adam
from .policy import LOG_STD_MAX, LOG_STD_MIN, Policy

__all__ = ["PPOConfig", "rollout", "compute_gae", "ppo_update", "evaluate_policy"]


@dataclass
class PPOConfig:
    rollout_steps: int = 1024
    epochs: int = 4
    batch: int = 64
    lr: float = 3e-4
    gamma: float = 0.99
    lam: float = 0.95
    clip: float = 0.2
    value_coef: float = 0.5
    entropy_coef: float = 0.005
    grad_clip: float = 0.5
    weight_decay: float = 0.0
    seed: int = 0


# --------------------------------------------------------------------------- #
def rollout(policy: Policy, env, steps: int, rng: np.random.Generator,
            max_steps_per_ep: int = 800, jitter: float = 0.6) -> List[Dict]:
    """采样一段轨迹。每条 episode 结束时把隐状态清零。"""
    obs = env.reset(seed=int(rng.integers(0, 1 << 30)), jitter=jitter)
    h = policy.init_hidden(1)
    data: List[Dict] = []
    ep_steps = 0
    while len(data) < steps:
        act, logp, value, h_new, raw = policy.act(obs, h, rng=rng)
        obs2, r, done, info = env.step(act[0])
        data.append({"obs": obs, "h": h.copy(), "act": act[0].copy(),
                     "raw": raw[0].copy(), "logp": float(logp[0]),
                     "value": float(value[0]), "reward": float(r),
                     "done": float(done)})
        obs = obs2
        h = h_new
        ep_steps += 1
        if done or ep_steps >= max_steps_per_ep:
            # 每轮结束的统计挂在那一轮的最后一帧上：采样循环可能横跨多轮，
            # 只看 data[-1] 会漏掉前面的轮次。
            data[-1]["info"] = {"passed": float(info["passed"]),
                                "completed": float(info["completed"]),
                                "time": float(info["time"])}
            obs = env.reset(seed=int(rng.integers(0, 1 << 30)), jitter=jitter)
            h = policy.init_hidden(1)
            ep_steps = 0
    return data


def compute_gae(data: Sequence[Dict], last_value: float = 0.0,
                gamma: float = 0.99, lam: float = 0.95) -> Tuple[np.ndarray, np.ndarray]:
    n = len(data)
    adv = np.zeros(n)
    ret = np.zeros(n)
    gae = 0.0
    for t in reversed(range(n)):
        next_v = last_value if t == n - 1 else data[t + 1]["value"]
        nonterm = 1.0 - data[t]["done"]
        delta = data[t]["reward"] + gamma * next_v * nonterm - data[t]["value"]
        gae = delta + gamma * lam * nonterm * gae
        adv[t] = gae
        ret[t] = gae + data[t]["value"]
    return adv, ret


def _batch_obs(sel: Sequence[Dict]):
    return {"depth": np.stack([d["obs"]["depth"] for d in sel]).astype(np.float32),
            "vel": np.stack([d["obs"]["vel"] for d in sel]).astype(np.float32)}


def ppo_update(policy: Policy, data: Sequence[Dict], cfg: PPOConfig,
               opt: Optional[Adam] = None,
               adv: Optional[np.ndarray] = None) -> Dict[str, float]:
    """一次 PPO 更新。``adv`` 可外部传入（离线复用同一批数据时用）。"""
    if adv is None:
        adv, ret = compute_gae(data, gamma=cfg.gamma, lam=cfg.lam)
    else:
        _, ret = compute_gae(data, gamma=cfg.gamma, lam=cfg.lam)
    opt = opt or Adam(policy.layers, lr=cfg.lr, weight_decay=cfg.weight_decay,
                      clip=cfg.grad_clip)
    n = len(data)
    idx = np.arange(n)
    rng = np.random.default_rng(cfg.seed)
    stats = {"pi": 0.0, "v": 0.0, "ent": 0.0, "kl": 0.0, "clip_frac": 0.0}
    nb = 0
    for _ in range(cfg.epochs):
        rng.shuffle(idx)
        for b in range(0, n, cfg.batch):
            sel = idx[b:b + cfg.batch]
            if len(sel) < 2:
                continue
            adv_b = adv[sel]
            adv_b = (adv_b - adv_b.mean()) / (adv_b.std() + 1e-8)
            obs = _batch_obs([data[i] for i in sel])
            h = np.concatenate([data[i]["h"] for i in sel], axis=0)
            act = np.stack([data[i]["act"] for i in sel])
            raw_old = np.stack([data[i]["raw"] for i in sel])
            logp_old = np.array([data[i]["logp"] for i in sel])
            ret_b = ret[sel]

            mean, value, _ = policy.forward(obs, h)
            std = policy.std[None, :]
            z = (raw_old - mean) / std
            logp = (-0.5 * z ** 2 - np.log(std) - 0.5 * math.log(2 * math.pi)
                    ).sum(axis=1)
            logp = logp - np.sum(np.log(1.0 - act ** 2 + 1e-6), axis=1)
            ratio = np.exp(np.clip(logp - logp_old, -6.0, 6.0))

            unclipped = ratio * adv_b
            clipped = np.clip(ratio, 1.0 - cfg.clip, 1.0 + cfg.clip) * adv_b
            use_unclipped = unclipped <= clipped
            n_clip = int((~use_unclipped).sum())

            # dL/dlogp：未裁剪分支取 -ratio*A；裁剪分支梯度为 0
            d_logp = np.where(use_unclipped, -ratio * adv_b, 0.0) / len(sel)
            # ∂logp/∂μ = +(raw−μ)/σ²（高斯对数密度的得分函数，符号为正）。
            # 写成负号会让更新把高回报动作的概率往下压——测试里奖励偏向 +1
            # 的动作，10 轮 PPO 后策略输出收敛到 −1，方向完全反了。
            d_mean = d_logp[:, None] * ((raw_old - mean) / std ** 2)
            d_logstd = np.sum(d_logp[:, None] * (z ** 2 - 1.0), axis=0)
            # 熵项：H = Σ(log σ + 常数)，只对 log σ 有梯度
            d_logstd += -cfg.entropy_coef * np.ones(policy.cfg.action_dim)
            d_value = cfg.value_coef * (value - ret_b) / len(sel)

            policy.zero_grad()
            policy.backward(d_mean, d_value, d_logstd)
            opt.step()
            # log σ 也得更新（它是独立参数，不在任何层里）
            policy.log_std -= cfg.lr * d_logstd
            np.clip(policy.log_std, LOG_STD_MIN, LOG_STD_MAX, out=policy.log_std)

            with np.errstate(divide="ignore", invalid="ignore"):
                approx_kl = float(np.mean(logp_old - logp))
            stats["pi"] += float(np.mean(-np.minimum(unclipped, clipped)))
            stats["v"] += float(np.mean((value - ret_b) ** 2))
            stats["ent"] += float(policy.cfg.action_dim * 0.5
                                  * math.log(2 * math.pi * math.e))
            stats["ent"] += float(np.sum(policy.log_std))
            stats["kl"] += approx_kl
            stats["clip_frac"] += n_clip / len(sel)
            nb += 1
    for k in ("pi", "v", "ent", "kl", "clip_frac"):
        stats[k] /= max(nb, 1)
    return stats


# --------------------------------------------------------------------------- #
def evaluate_policy(policy: Policy, env, n_episode: int = 5, seed: int = 0,
                    deterministic: bool = True, jitter: float = 0.6,
                    max_steps: int = 800) -> Dict[str, float]:
    """评估：通过门数、完成率、通过时间、碰撞数、推理延迟。"""
    import time
    rng = np.random.default_rng(seed)
    passed, times, coll, lat = [], [], [], []
    completed = 0
    for _ in range(n_episode):
        obs = env.reset(seed=int(rng.integers(0, 1 << 30)), jitter=jitter)
        h = policy.init_hidden(1)
        steps = 0
        while steps < max_steps:
            t0 = time.perf_counter()
            act, _, _, h, _ = policy.act(obs, h, deterministic=deterministic,
                                         rng=rng)
            lat.append((time.perf_counter() - t0) * 1000.0)
            obs, _, done, info = env.step(act[0])
            steps += 1
            if done:
                break
        passed.append(info["passed"])
        coll.append(info["collisions"])
        if info["completed"]:
            completed += 1
            times.append(info["time"])
    return {"passed": float(np.mean(passed)),
            "completed": completed / max(n_episode, 1),
            "time": float(np.mean(times)) if times else float("nan"),
            "collisions": float(np.mean(coll)),
            "latency_ms": float(np.mean(lat)) if lat else float("nan")}
