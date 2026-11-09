#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PPO 训练入口：课程学习 -> 保存模型 -> 导出纯 numpy 权重（部署侧加载）."""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, EvalCallback
from stable_baselines3.common.vec_env import DummyVecEnv

from policy import extract_sb3_weights
from traffic_env import TrafficNavEnv


class CurriculumCallback(BaseCallback):
    """按训练进度把难度从 start 线性升到 1（车辆速度同步提高）."""

    def __init__(self, total_timesteps: int, start: float = 0.5, verbose: int = 0):
        super().__init__(verbose)
        self.total = max(int(total_timesteps), 1)
        self.start = float(start)

    def _on_step(self) -> bool:
        progress = min(1.0, self.num_timesteps / self.total)
        difficulty = self.start + (1.0 - self.start) * progress
        for env in self.training_env.envs:
            env.set_difficulty(difficulty)
        return True


def make_env(seed, include_traffic, difficulty, force_crossing, cars_per_lane=2):
    def _factory():
        env = TrafficNavEnv(include_traffic=include_traffic, seed=seed,
                            cars_per_lane=cars_per_lane,
                            force_crossing=force_crossing)
        env.max_cars_per_lane = cars_per_lane
        env.set_difficulty(difficulty)
        return env
    return _factory


def main():
    ap = argparse.ArgumentParser(description="训练动态交通避障 PPO 策略")
    ap.add_argument("--total", type=int, default=400000, help="总训练步数")
    ap.add_argument("--net", type=str, default="64,64", help="隐藏层结构")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", type=str, default="models")
    ap.add_argument("--name", type=str, default="ppo_scratch",
                    help="导出的 npz 名称（不含扩展名）")
    ap.add_argument("--no-traffic-features", action="store_true",
                    help="消融：观测里去掉车辆相对速度（22 维，仅靠激光直方图）")
    ap.add_argument("--force-crossing", action="store_true",
                    help="起点/终点固定在车道两侧，强制横穿车流")
    ap.add_argument("--difficulty", type=float, default=1.0,
                    help="训练难度（1.0 = 满速车辆）")
    ap.add_argument("--eval-freq", type=int, default=10000)
    ap.add_argument("--init-from", default="", help="用行为克隆权重初始化策略")
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--ent", type=float, default=0.01)
    args = ap.parse_args()

    include_traffic = not args.no_traffic_features
    net_arch = [int(x) for x in args.net.split(",")]

    env = DummyVecEnv([make_env(args.seed + i, include_traffic, args.difficulty,
                                args.force_crossing) for i in range(4)])
    eval_env = DummyVecEnv([make_env(args.seed + 10, include_traffic, args.difficulty,
                                     args.force_crossing)])

    os.makedirs(args.out, exist_ok=True)
    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path=os.path.join(args.out, "best_" + args.name),
        log_path=os.path.join(args.out, "eval_log_" + args.name),
        eval_freq=args.eval_freq,
        n_eval_episodes=10,
        deterministic=True,
    )
    curriculum = CurriculumCallback(args.total, start=0.6)

    model = PPO(
        "MlpPolicy", env, verbose=0,
        n_steps=512, batch_size=256, learning_rate=args.lr,
        gamma=0.99, gae_lambda=0.95, ent_coef=args.ent,
        vf_coef=0.5, max_grad_norm=0.5,
        policy_kwargs={"net_arch": net_arch},
        seed=args.seed,
    )
    if args.init_from:
        import torch
        data = np.load(args.init_from)
        layers = [m for m in model.policy.mlp_extractor.policy_net
                  if isinstance(m, torch.nn.Linear)]
        with torch.no_grad():
            for i, layer in enumerate(layers):
                layer.weight.copy_(torch.tensor(data["h%d_w" % i]))
                layer.bias.copy_(torch.tensor(data["h%d_b" % i]))
            model.policy.action_net.weight.copy_(torch.tensor(data["out_w"]))
            model.policy.action_net.bias.copy_(torch.tensor(data["out_b"]))
        print("已用 %s 初始化策略 (%d 层)" % (args.init_from, len(layers)))

    model.learn(total_timesteps=args.total, callback=[curriculum, eval_cb])

    best_path = os.path.join(args.out, "best_" + args.name, "best_model.zip")
    best = PPO.load(best_path) if os.path.exists(best_path) else model
    weights = extract_sb3_weights(best)
    weights["include_traffic"] = int(include_traffic)
    out = os.path.join(args.out, args.name + ".npz")
    np.savez(out, **weights)
    print("导出权重: %s (obs_dim=%d)" % (out, 34 if include_traffic else 22))


if __name__ == "__main__":
    main()
