#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""示范引导：先用规则专家做行为克隆，再用 DAgger 在"学生自己走到的状态"上纠偏.

    python3 scripts/bc_train.py --episodes 200 --dagger-rounds 3 --out models/bc_policy.npz

DAgger 的价值：单纯 BC 只在专家轨迹上训练，学生一旦偏离就再也没见过正确动作；
DAgger 让学生自己飞、在每个到达的状态上问专家"这里该怎么做"，把纠偏样本加进数据集。
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import torch
import torch.nn as nn

from baselines import expert_action
from policy import ACT_DIM
from traffic_env import TrafficNavEnv


class Mlp(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden), nn.Tanh(),
            nn.Linear(hidden, hidden), nn.Tanh(),
            nn.Linear(hidden, act_dim))

    def forward(self, x):
        return self.net(x)


def collect_expert(env, episodes, seed):
    obs_list, act_list = [], []
    for episode in range(episodes):
        obs, _ = env.reset(seed=seed * 100 + episode)
        done = False
        while not done:
            action = expert_action(env)
            obs_list.append(obs.copy())
            act_list.append(action.copy())
            obs, _reward, terminated, truncated, _info = env.step(action)
            done = terminated or truncated
    return obs_list, act_list


def rollout_student(env, model, episodes, seed):
    """让学生自己飞，在访问到的状态上贴专家标签（DAgger 的核心）."""
    obs_list, act_list = [], []
    success = 0
    for episode in range(episodes):
        obs, _ = env.reset(seed=seed * 100 + 7919 + episode)
        done = False
        while not done:
            obs_list.append(obs.copy())
            act_list.append(expert_action(env).copy())
            with torch.no_grad():
                action = model(torch.from_numpy(obs).float()).numpy()
            obs, _reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
        success += int(bool(info.get("success")))
    return obs_list, act_list, success


def train(model, obs, act, epochs, lr=1e-3, hold_weight=2.0):
    x = torch.from_numpy(obs)
    y = torch.from_numpy(act)
    # "悬停"样本在专家数据里是少数，但恰恰是学会等窗口的关键，给更高权重
    weight = 1.0 + (hold_weight - 1.0) * (np.abs(act).sum(axis=1) < 0.05)
    w = torch.from_numpy(weight.astype(np.float32))
    optimiser = torch.optim.Adam(model.parameters(), lr=lr)
    n = len(x)
    for epoch in range(epochs):
        perm = torch.randperm(n)
        for start in range(0, n, 256):
            idx = perm[start:start + 256]
            loss = (((model(x[idx]) - y[idx]) ** 2).mean(dim=1) * w[idx]).mean()
            optimiser.zero_grad()
            loss.backward()
            optimiser.step()
    with torch.no_grad():
        mse = float(((model(x) - y) ** 2).mean())
    return mse


def export(model, path, include_traffic=True):
    layers = [m for m in model.net if isinstance(m, nn.Linear)]
    data = {"n_layers": len(layers) - 1, "include_traffic": int(include_traffic)}
    for i, layer in enumerate(layers[:-1]):
        data["h%d_w" % i] = layer.weight.detach().numpy()
        data["h%d_b" % i] = layer.bias.detach().numpy()
    data["out_w"] = layers[-1].weight.detach().numpy()
    data["out_b"] = layers[-1].bias.detach().numpy()
    np.savez(path, **data)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--dagger-episodes", type=int, default=40)
    ap.add_argument("--dagger-rounds", type=int, default=3)
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default="models/bc_policy.npz")
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    forced = TrafficNavEnv(seed=args.seed + 1, force_crossing=True)
    forced.set_difficulty(1.0)
    mixed = TrafficNavEnv(seed=args.seed)
    mixed.set_difficulty(1.0)

    obs_a, act_a = collect_expert(mixed, int(args.episodes * 0.6), args.seed)
    obs_b, act_b = collect_expert(forced, int(args.episodes * 0.4), args.seed)
    obs = np.asarray(obs_a + obs_b, dtype=np.float32)
    act = np.asarray(act_a + act_b, dtype=np.float32)
    print("专家示范样本 %d（其中悬停动作 %d）"
          % (len(obs), int((np.abs(act).sum(axis=1) < 0.05).sum())))

    model = Mlp(obs.shape[1], ACT_DIM)
    print("BC mse %.5f" % train(model, obs, act, args.epochs))

    for round_index in range(args.dagger_rounds):
        obs_d, act_d, success = rollout_student(forced, model, args.dagger_episodes,
                                               args.seed + round_index)
        obs = np.concatenate([obs, np.asarray(obs_d, dtype=np.float32)])
        act = np.concatenate([act, np.asarray(act_d, dtype=np.float32)])
        mse = train(model, obs, act, args.epochs // 2)
        print("DAgger 第 %d 轮：新增 %d 样本，学生成功率 %.0f%%，mse %.5f"
              % (round_index + 1, len(obs_d), 100.0 * success / args.dagger_episodes, mse))

    export(model, args.out)
    print("已导出 %s" % args.out)


if __name__ == "__main__":
    main()
