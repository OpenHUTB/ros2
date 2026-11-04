#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DAgger：策略跑、专家纠错、聚合重训。

纯行为克隆的问题在闭环里会放大：策略一旦偏离专家轨迹，就进入了训练集里
没见过的状态，误差随步数累积。DAgger 的做法是**在策略自己的状态分布上**
持续向专家要标签：

    π₀ ← 专家示范
    for i in 1..N:
        用 πᵢ₋₁ 跑，并在每一步同时问专家该怎么做
        把新数据并进数据集
        πᵢ ← 在聚合数据上重训

聚合数据按"轮次越新权重越高"采样：早期专家轨迹占的样本量很大，不降权的话
后面几轮纠错样本会被淹没。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from .bc import BCConfig, bc_metrics, norm_scale, train_bc
from .expert import GateExpert
from .gate_env import GateRacingEnv
from .policy import Policy

__all__ = ["DaggerConfig", "dagger_round", "run_dagger", "sample_aggregate"]


@dataclass
class DaggerConfig:
    rounds: int = 3
    expert_episodes: int = 4
    policy_episodes: int = 3
    max_steps: int = 800
    jitter: float = 0.6
    bc_epochs: int = 8
    bc_epochs_first: int = 12
    batch: int = 32
    lr: float = 1e-3
    loss: str = "mse"          # 见 bc.BCConfig：Huber 会把横向输出压向中位数
    mirror: bool = True        # 水平镜像增广，数据翻倍
    #: 旧轮次样本的采样权重衰减（每多一轮乘一次）
    decay: float = 0.6
    val_frac: float = 0.1
    max_samples: int = 40000
    #: 采集策略轨迹时的动作噪声（归一化尺度）。0 = 用确定性动作。
    #: 初版直接采样（σ=0.6 归一化 ≈ 2.4 m/s 抖动），无人机出门就被甩飞，
    #: 采到的全是失控状态，三轮下来闭环从 4.2/8 退化到 0/8。DAgger 要的是
    #: **策略自己的状态分布**，不是随机探索分布——探索留给后面的 PPO。
    policy_noise: float = 0.0
    eval_episodes: int = 3
    #: 策略跑出这个包络就截断本轮采集。失控状态下的专家标签残差很大，MSE 又是
    #: 平方加权，几千条这种样本会把专家数据**反向压过去**——实测不截断时三轮
    #: DAgger 把闭环从 1.7/8 拖到 0/8，连专家分布上的验证 MSE 都从 0.0006
    #: 涨到 0.0084。专家自己全程倾角 < 45°、高度 0.6~3.2 m、|y| < 2 m。
    drift_tilt: float = math.radians(50.0)
    drift_z: float = 0.4
    drift_y: float = 4.0
    seed: int = 0


def _rollout_policy_collect_expert(env: GateRacingEnv, policy: Policy,
                                   expert: GateExpert, n_ep: int, cfg: DaggerConfig,
                                   rng) -> List[Dict[str, np.ndarray]]:
    """策略跑，同时在每一步记录专家的动作作为标签。"""
    out: List[Dict[str, np.ndarray]] = []
    for _ in range(n_ep):
        obs = env.reset(seed=int(rng.integers(0, 1 << 30)), jitter=cfg.jitter)
        h = policy.init_hidden(1)
        steps = 0
        while steps < cfg.max_steps:
            a_exp = expert.act(env)
            out.append({"depth": obs["depth"], "vel": obs["vel"], "act": a_exp})
            a_pol, _, _, h, _ = policy.act(
                obs, h, deterministic=(cfg.policy_noise <= 0.0), rng=rng)
            if cfg.policy_noise > 0.0:
                mode, _, _, _, _ = policy.act(obs, h, deterministic=True)
                a_pol = mode + cfg.policy_noise * (a_pol - mode)
            obs, _, done, info = env.step(a_pol[0])
            steps += 1
            if done:
                break
            st = env.quad.state()
            if (math.hypot(st.roll, st.pitch) > cfg.drift_tilt
                    or st.pos[2] < cfg.drift_z or abs(st.pos[1]) > cfg.drift_y):
                break
    return out


def sample_aggregate(rounds: Sequence[Sequence[Dict[str, np.ndarray]]],
                     decay: float, max_samples: int, rng) -> List[Dict]:
    """按轮次加权抽样，把各轮数据合成一个训练集。"""
    weights = [decay ** (len(rounds) - 1 - i) for i in range(len(rounds))]
    sizes = [max(len(r), 1) for r in rounds]
    total = sum(w * s for w, s in zip(weights, sizes))
    out: List[Dict] = []
    for r, w, s in zip(rounds, weights, sizes):
        k = int(round(max_samples * (w * s / total)))
        k = max(min(k, len(r)), 0)
        if k == 0:
            continue
        idx = rng.choice(len(r), size=k, replace=k > len(r))
        out += [r[i] for i in idx]
    return out


def dagger_round(env: GateRacingEnv, policy: Policy,
                 rounds_data: List[List[Dict]], cfg: DaggerConfig,
                 rng, verbose: bool = True) -> Dict[str, float]:
    """执行一轮：策略示教采集 → 聚合 → 重训。"""
    expert = GateExpert()
    new = _rollout_policy_collect_expert(env, policy, expert, cfg.policy_episodes,
                                         cfg, rng)
    rounds_data.append(new)
    agg = sample_aggregate(rounds_data, cfg.decay, cfg.max_samples, rng)
    val = agg[:max(int(len(agg) * cfg.val_frac), 1)]
    bcfg = BCConfig(epochs=cfg.bc_epochs, batch=cfg.batch, lr=cfg.lr,
                    seed=cfg.seed, val_frac=cfg.val_frac, loss=cfg.loss,
                    mirror=cfg.mirror)
    hist = train_bc(policy, agg, norm_scale(env.cfg), bcfg, verbose=verbose)
    m = bc_metrics(policy, val, norm_scale(env.cfg), cfg=bcfg)
    return {"new_samples": float(len(new)), "agg_samples": float(len(agg)),
            "loss": hist["train"][-1], "val_mse": m["mse"]}


def run_dagger(env: GateRacingEnv, policy: Policy,
               cfg: Optional[DaggerConfig] = None, verbose: bool = True
               ) -> Dict[str, List[float]]:
    cfg = cfg or DaggerConfig()
    rng = np.random.default_rng(cfg.seed)
    expert = GateExpert()

    if verbose:
        print("第 0 轮：专家示范")
    from .expert import rollout_expert
    first = rollout_expert(env, n_episode=cfg.expert_episodes,
                           max_steps=cfg.max_steps, jitter=cfg.jitter,
                           seed=cfg.seed, verbose=verbose)
    rounds_data: List[List[Dict]] = [first]
    hist: Dict[str, List[float]] = {"samples": [float(len(first))],
                                    "val_mse": [], "loss": [],
                                    "passed": [], "completed": []}
    bcfg = BCConfig(epochs=cfg.bc_epochs_first, batch=cfg.batch, lr=cfg.lr,
                    seed=cfg.seed, val_frac=cfg.val_frac, loss=cfg.loss,
                    mirror=cfg.mirror)
    h0 = train_bc(policy, first, norm_scale(env.cfg), bcfg, verbose=verbose)
    hist["val_mse"].append(h0["val_mse"][-1])
    hist["loss"].append(h0["train"][-1])

    def _eval(tag: str) -> None:
        """每轮做一次闭环评测。

        只看开环 MSE 会误判：训练/验证集都是专家数据时 MSE 一路降，但闭环早就
        退化了（实测三轮后 MSE 0.0094 看着不错，闭环 0/8）。通过门数是唯一
        能反映"飞得怎么样"的信号。
        """
        from .ppo import evaluate_policy
        m = evaluate_policy(policy, env, n_episode=cfg.eval_episodes,
                            seed=cfg.seed + 3, deterministic=True,
                            jitter=cfg.jitter, max_steps=cfg.max_steps)
        hist["passed"].append(float(m["passed"]))
        hist["completed"].append(float(m["completed"]))
        if verbose:
            print("  [闭环] %s：通过 %.1f/%d，完成率 %.0f%%，擦框 %.1f"
                  % (tag, m["passed"], len(env.track), 100 * m["completed"],
                     m["collisions"]))

    _eval("第 0 轮后")
    for i in range(cfg.rounds):
        if verbose:
            print("第 %d 轮 DAgger" % (i + 1))
        st = dagger_round(env, policy, rounds_data, cfg, rng, verbose=verbose)
        hist["samples"].append(st["agg_samples"])
        hist["val_mse"].append(st["val_mse"])
        hist["loss"].append(st["loss"])
        if verbose:
            print("  本轮新增 %d 条，聚合 %d 条，验证 MSE %.5f"
                  % (st["new_samples"], st["agg_samples"], st["val_mse"]))
        _eval("第 %d 轮后" % (i + 1))
    return hist
