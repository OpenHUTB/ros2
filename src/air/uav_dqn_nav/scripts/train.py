#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DQN 导航训练入口：训练 -> 保存模型 -> 导出纯 numpy 权重（供部署侧加载）.

用法（由 main.py --train 转发）：
    python3 train.py --total 200000 --obstacles 6 --seed 42
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

from dqn_agent import DQNAgent
from env import NavEnv
from policy import N_ACTIONS, OBS_DIM, act_index_to_velocity


def _evaluate(agent: DQNAgent, episodes: int, obstacles: int, seed: int):
    """用贪心策略评估成功率（确定性动作）."""
    env = NavEnv(num_obstacles=obstacles, seed=seed)
    succ = crash = timeout = 0
    rewards, steps_list = [], []
    for _ in range(episodes):
        obs, _ = env.reset()
        done = False
        ep_r, n = 0.0, 0
        while not done:
            act = agent.select_action(obs, training=False)
            obs, r, terminated, truncated, info = env.step(act_index_to_velocity(act))
            ep_r += r
            n += 1
            done = terminated or truncated
        rewards.append(ep_r)
        steps_list.append(n)
        if info["success"]:
            succ += 1
        elif info["crash_reason"]:
            crash += 1
        else:
            timeout += 1
    env.close()
    return succ, crash, timeout, float(np.mean(rewards)), float(np.mean(steps_list))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--total", type=int, default=200000, help="总训练步数")
    ap.add_argument("--batch", type=int, default=128, help="回放采样批量")
    ap.add_argument("--buffer", type=int, default=50000, help="经验回放容量")
    ap.add_argument("--gamma", type=float, default=0.99, help="折扣因子")
    ap.add_argument("--lr", type=float, default=1e-3, help="学习率")
    ap.add_argument("--tau", type=float, default=0.005, help="目标网络软更新系数")
    ap.add_argument("--update-freq", type=int, default=4, help="每 N 步训练一次")
    ap.add_argument("--epsilon-start", type=float, default=1.0)
    ap.add_argument("--epsilon-end", type=float, default=0.05)
    ap.add_argument("--epsilon-decay", type=int, default=100000,
                    help="ε 线性衰减步数")
    ap.add_argument("--obstacles", type=int, default=6, help="障碍物数量")
    ap.add_argument("--eval-freq", type=int, default=10000, help="评估间隔（步）")
    ap.add_argument("--eval-episodes", type=int, default=10, help="每次评估回合数")
    ap.add_argument("--out", type=str, default="models")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    agent = DQNAgent(
        obs_dim=OBS_DIM, n_actions=N_ACTIONS,
        lr=args.lr, gamma=args.gamma, tau=args.tau,
        update_freq=args.update_freq, batch_size=args.batch,
        buffer_capacity=args.buffer,
        epsilon_start=args.epsilon_start, epsilon_end=args.epsilon_end,
        epsilon_decay_steps=args.epsilon_decay,
        seed=args.seed,
    )

    env = NavEnv(num_obstacles=args.obstacles, seed=args.seed)
    best_succ = -1.0
    eval_log = []

    obs, _ = env.reset()
    ep_reward, ep_steps = 0.0, 0
    for step in range(1, args.total + 1):
        act = agent.select_action(obs, training=True)
        obs2, r, terminated, truncated, _info = env.step(act_index_to_velocity(act))
        agent.remember(obs, act, r, obs2, terminated)
        agent.maybe_train()

        ep_reward += r
        ep_steps += 1
        obs = obs2

        if terminated or truncated:
            # 回合结束：记录后重置（回合信息仅用于进度显示）
            obs, _ = env.reset()
            ep_reward, ep_steps = 0.0, 0

        if step % args.eval_freq == 0:
            succ, crash, timeout, avg_r, avg_steps = _evaluate(
                agent, args.eval_episodes, args.obstacles, args.seed + 1)
            eval_log.append((step, succ, avg_r))
            print("step=%8d | eps=%.3f | eval成功=%d/%d (%.1f%%) | "
                  "失败=%d 超时=%d | 平均奖励=%.2f | 平均步数=%.1f"
                  % (step, agent.epsilon(), succ, args.eval_episodes,
                     100.0 * succ / args.eval_episodes, crash, timeout,
                     avg_r, avg_steps), flush=True)

            if succ > best_succ:
                best_succ = succ
                agent.save(os.path.join(args.out, "best_model.pt"),
                           os.path.join(args.out, "policy_weights.npz"))
                print("  -> 新最佳模型已保存 (成功率 %.1f%%)"
                      % (100.0 * best_succ / args.eval_episodes), flush=True)

    env.close()

    # 最终保存
    agent.save(os.path.join(args.out, "final_model.pt"),
               os.path.join(args.out, "policy_weights.npz"))
    print("训练完成：best 成功率 = %d/%d，权重已导出 %s"
          % (best_succ, args.eval_episodes,
             os.path.join(args.out, "policy_weights.npz")))


if __name__ == "__main__":
    main()
