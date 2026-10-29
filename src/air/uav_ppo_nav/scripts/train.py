#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PPO 导航训练入口：训练 -> 保存模型 -> 导出纯 numpy 权重（供部署侧加载）."""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.vec_env import DummyVecEnv

from env import NavEnv
from policy import extract_sb3_weights


def _make_env(seed, obstacles):
    def _f():
        return NavEnv(num_obstacles=obstacles, seed=seed)
    return _f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--total", type=int, default=300000, help="总训练步数")
    ap.add_argument("--net", type=str, default="64,64", help="隐藏层结构")
    ap.add_argument("--obstacles", type=int, default=6, help="障碍物数量")
    ap.add_argument("--out", type=str, default="models")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    net_arch = [int(x) for x in args.net.split(",")]
    env = DummyVecEnv([_make_env(args.seed, args.obstacles)])
    eval_env = DummyVecEnv([_make_env(args.seed + 1, args.obstacles)])

    os.makedirs(args.out, exist_ok=True)
    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path=args.out,
        log_path=os.path.join(args.out, "eval_log"),
        eval_freq=20000,
        n_eval_episodes=10,
        deterministic=True,
    )

    model = PPO(
        "MlpPolicy", env, verbose=1,
        n_steps=2048, batch_size=256, learning_rate=3e-4,
        gamma=0.99, gae_lambda=0.95, ent_coef=0.005,
        vf_coef=0.5, max_grad_norm=0.5,
        policy_kwargs={"net_arch": net_arch},
        seed=args.seed,
    )
    model.learn(total_timesteps=args.total, callback=eval_cb)

    # 保存最终模型
    model.save(os.path.join(args.out, "final_model"))

    # 导出权重（部署侧纯 numpy 推理用）
    best_path = os.path.join(args.out, "best_model.zip")
    best = PPO.load(best_path) if os.path.exists(best_path) else model
    w = extract_sb3_weights(best)
    np.savez(os.path.join(args.out, "policy_weights.npz"), **w)
    print("已导出权重: %s" % os.path.join(args.out, "policy_weights.npz"))


if __name__ == "__main__":
    main()
