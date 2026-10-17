#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""专家示范：朝下一个门开口中心飞的比例控制律。

赛道是一条沿 +x 排开的门序列，"走哪条路"没有搜索空间，所以专家不需要 A*——
真正需要决策的是**怎么在门之间保持平飞并压住速度**。这里用航向系比例控制：

    v_cmd = clip(K ⊙ R_z(yaw)ᵀ(T_aim − p), ±v_max)

瞄准点取门中心**再往前一点**（``look_ahead``），否则无人机到门口会减速，
示范里全是"贴着门悬停再挪"的动作，策略学出来也会磨蹭。

专家只吃**真值位姿**（不是深度图）：深度图给策略学，专家负责给出"该怎么动"，
两者共用同一套动作语义（``GateRacingEnv`` 的航向系速度指令），DAgger 才能对齐。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

__all__ = ["ExpertConfig", "GateExpert", "rollout_expert"]


@dataclass
class ExpertConfig:
    k_xy: float = 1.6          # 位置误差 → 速度 的比例
    k_z: float = 1.8
    #: 瞄准点越过门中心的距离。相邻门高差最大 2.2 m、间距 5 m，1.5 m 时无人机
    #: 进门前才开始补偿高度，实测平均擦框 1.25 次/轮；2.5 m 时 0 次。
    look_ahead: float = 2.5
    v_max: float = 3.0
    #: 门的开口中心在 z ∈ [0.8, 3.2] 之间跳，相邻门高差可达 1.2 m，
    #: 5 m 的间距里只有 1.7 s 去补偿；垂直速度上限太小会一路擦框。
    v_z_max: float = 1.8
    settle: float = 1.0        # 距离门多近开始把横向误差压小


class GateExpert:
    def __init__(self, cfg: Optional[ExpertConfig] = None):
        self.cfg = cfg or ExpertConfig()

    def act(self, env) -> np.ndarray:
        cfg = self.cfg
        st = env.quad.state()
        p = st.pos
        target = env.expert_target().copy()
        target[0] += cfg.look_ahead
        # 误差在**航向系**里算，不用实时姿态。机身为了前进而俯仰时，实时姿态
        # 会把一部分前向误差旋到竖直轴上（实测俯仰 20° 就能把 ±0.5 m 的高度差
        # 放大成 −1.8 m/s 的下沉指令），下沉又加剧俯仰，几帧就翻过去了。
        yaw = getattr(env, "ref_yaw", None)
        if yaw is None:
            yaw = st.yaw
        e = target - p
        c, s = math.cos(yaw), math.sin(yaw)
        e_b = np.array([c * e[0] + s * e[1], -s * e[0] + c * e[1], e[2]])
        # 快到门时提高纵向增益、压低横向，避免在门口画弧
        near = np.linalg.norm(target[:2] - p[:2]) < cfg.settle
        v = np.array([cfg.k_xy * e_b[0], cfg.k_xy * e_b[1], cfg.k_z * e_b[2]])
        if near:
            v[:2] *= 0.8
        v[0] = np.clip(v[0], 0.35 * cfg.v_max, cfg.v_max)   # 只许前进
        v[1] = np.clip(v[1], -cfg.v_max, cfg.v_max)
        v[2] = np.clip(v[2], -cfg.v_z_max, cfg.v_z_max)
        return v.astype(np.float32)


def rollout_expert(env, n_episode: int = 4, max_steps: int = 800,
                   jitter: float = 0.6, seed: int = 0,
                   verbose: bool = False) -> List[Dict[str, np.ndarray]]:
    """跑若干轮专家示范，返回逐帧 (obs, action, reward, done) 记录。

    ``jitter`` 是每轮起点的随机偏移：DAgger 的第一轮若全从同一点起飞，
    策略学到的状态分布会窄得可怜。
    """
    ex = GateExpert()
    rng = np.random.default_rng(seed)
    out: List[Dict[str, np.ndarray]] = []
    for ep in range(n_episode):
        obs = env.reset(seed=int(rng.integers(0, 1 << 30)), jitter=jitter)
        steps = 0
        while steps < max_steps:
            a = ex.act(env)
            obs2, r, done, info = env.step(a)
            out.append({"depth": obs["depth"], "vel": obs["vel"], "act": a,
                        "reward": np.float32(r), "done": np.float32(done),
                        "episode": np.int32(ep)})
            obs = obs2
            steps += 1
            if done:
                break
        if verbose:
            print("  专家第 %d 轮：通过 %d/%d 门，碰撞 %d，用时 %.1f s"
                  % (ep, info["passed"], len(env.track), info["collisions"],
                     info["time"]))
    return out


def expert_metrics(env, n_episode: int = 3, seed: int = 0) -> Dict[str, float]:
    """专家自身的成绩——作为策略的上界参照。"""
    ex = GateExpert()
    rng = np.random.default_rng(seed)
    passed, times, coll = [], [], []
    for _ in range(n_episode):
        obs = env.reset(seed=int(rng.integers(0, 1 << 30)), jitter=0.6)
        done = False
        while not done:
            _, _, done, info = env.step(ex.act(env))
        passed.append(info["passed"])
        times.append(info["time"] if info["completed"] else float("nan"))
        coll.append(info["collisions"])
    t = [v for v in times if v == v]
    return {"passed": float(np.mean(passed)),
            "time": float(np.mean(t)) if t else float("nan"),
            "collisions": float(np.mean(coll)),
            "completed": float(len(t)) / n_episode}
