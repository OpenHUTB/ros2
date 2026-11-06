#!/usr/bin/env python3
"""PPO 训练脚本：G1 站立平衡的残差策略。

用法：
    python scripts/train_ppo.py --timesteps 3000000 --n-envs 8
    python scripts/train_ppo.py --timesteps 50000 --n-envs 4 --out /tmp/smoke

训练产物：
    <out>/ppo_g1.zip          SB3 存档（已被 .gitignore 忽略，仅续训 / 导出用）
    <out>/training_log.csv    每轮回报与指标（入库）

训练完记得导出 TorchScript 再入库：
    python scripts/export_policy.py --weights <out>/ppo_g1.zip --out weights/policy_g1.pt
"""

from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np
import gymnasium as gym
from gymnasium.wrappers import RecordEpisodeStatistics
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from humanoid_ppo_locomotion.g1_env import G1WalkEnv  # noqa: E402


def make_env(model_path: str, rank: int, seed: int = 0,
             vx_range=(-0.3, 1.0), wz_range=(-0.8, 0.8)):
    def _init():
        env = G1WalkEnv(model_path=model_path, cmd_vx_range=vx_range,
                        cmd_wz_range=wz_range)
        env = RecordEpisodeStatistics(env)   # 提供 info["episode"]，用于记录训练曲线
        env.reset(seed=seed + rank)
        return env

    return _init


class MetricsCallback(BaseCallback):
    """记录 rollout 的平均回报与平均速度。"""

    def __init__(self, log_path: str):
        super().__init__()
        self.log_path = log_path
        self.rows: list[dict] = []
        if os.path.exists(log_path):
            os.remove(log_path)

    def _on_step(self) -> bool:
        infos = self.locals.get("infos")
        if infos and "episode" in infos[0]:
            ep_rew = float(np.mean([i["episode"]["r"] for i in infos if "episode" in i]))
            ep_len = float(np.mean([i["episode"]["l"] for i in infos if "episode" in i]))
            self.rows.append(
                dict(timesteps=self.num_timesteps, ep_rew=ep_rew, ep_len=ep_len)
            )
            with open(self.log_path, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=["timesteps", "ep_rew", "ep_len"])
                w.writeheader()
                w.writerows(self.rows)
        return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=os.environ.get(
        "G1_MODEL", "models/g1/scene_29dof.xml"))
    ap.add_argument("--timesteps", type=int, default=3_000_000)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--out", default="saved_models/stage1")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--resume", default="")
    ap.add_argument("--vx-max", type=float, default=0.5, help="前向速度指令上限（stage1=0.5，stage2=1.0）")
    ap.add_argument("--wz-max", type=float, default=0.8, help="角速度指令上限")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    log_path = os.path.join(args.out, "training_log.csv")

    vx_range = (-0.3, args.vx_max)
    wz_range = (-args.wz_max, args.wz_max)
    print(f"[train] 指令范围 vx={vx_range} wz={wz_range}")
    env_fns = [make_env(args.model, i, args.seed, vx_range, wz_range)
               for i in range(args.n_envs)]
    venv = SubprocVecEnv(env_fns) if args.n_envs > 1 else DummyVecEnv(env_fns)

    if args.resume and os.path.exists(args.resume):
        model = PPO.load(args.resume, env=venv, device="cpu")
        print(f"[train] 从 {args.resume} 继续训练")
    else:
        model = PPO(
            "MlpPolicy",
            venv,
            learning_rate=3e-4,
            n_steps=512,
            batch_size=2048,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.005,
            vf_coef=0.5,
            max_grad_norm=1.0,
            policy_kwargs=dict(net_arch=dict(pi=[256, 128], vf=[256, 128]),
                               # 实测残差幅度 ±0.15 rad 以内基元都能稳住（20 s 跑满），
                               # ±0.30 rad 就只剩 2.4 s。1σ≈0.078 起步，让探索不致命。
                               log_std_init=-1.5),
            verbose=1,
            device="cpu",
            seed=args.seed,
        )

    model.learn(
        total_timesteps=args.timesteps,
        callback=MetricsCallback(log_path),
        progress_bar=False,
        reset_num_timesteps=not bool(args.resume),
    )
    out_path = os.path.join(args.out, "ppo_g1.zip")
    model.save(out_path)
    print(f"[train] 权重已保存：{out_path}")
    venv.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
