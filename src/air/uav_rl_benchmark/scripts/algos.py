#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL 算法工厂：同一环境、同一结构参数下构造 PPO / SAC / TD3.

超参选择原则（保证对比公平）
----------------------------
* **同一学习率** ``3e-4``、**同一折扣** ``gamma=0.99``、**同一批大小** ``256``；
* **同一网络结构**（由 ``arch`` 统一指定，默认 ``[64, 64]``）——
  SB3 的 SAC/TD3 默认是 ``[256, 256]``，若不统一，比出来的差异里混着"网络更大"这个因素；
* **同一经验回放/采样节奏**：on-policy 的 PPO 用 ``n_steps=2048``，
  off-policy 的 SAC/TD3 用 ``train_freq=1, gradient_steps=1``（每步一次梯度更新）；
* 其余保持 SB3 默认（SAC 自动熵调节、TD3 目标策略平滑等），因为这些正是各算法的特征。
"""
from __future__ import annotations

ALGOS = ("ppo", "sac", "td3")

DEFAULT_ARCH = (64, 64)

#: 统一超参
COMMON = dict(learning_rate=3e-4, gamma=0.99)


def make_model(name: str, env, arch=DEFAULT_ARCH, seed: int = 0, tensorboard_log=None):
    """构造指定算法的模型（惰性导入，便于无 torch 环境下只跑桩测试）."""
    name = name.lower()
    arch = [int(x) for x in arch]

    if name == "ppo":
        from stable_baselines3 import PPO  # noqa: WPS433
        return PPO("MlpPolicy", env, verbose=0, seed=seed,
                   n_steps=2048, batch_size=256,
                   gae_lambda=0.95, ent_coef=0.005, vf_coef=0.5, max_grad_norm=0.5,
                   policy_kwargs={"net_arch": arch},
                   tensorboard_log=tensorboard_log, **COMMON)

    if name == "sac":
        from stable_baselines3 import SAC  # noqa: WPS433
        return SAC("MlpPolicy", env, verbose=0, seed=seed,
                   buffer_size=200000, batch_size=256, tau=0.005,
                   train_freq=1, gradient_steps=1, learning_starts=1000,
                   policy_kwargs={"net_arch": arch},
                   tensorboard_log=tensorboard_log, **COMMON)

    if name == "td3":
        from stable_baselines3 import TD3  # noqa: WPS433
        return TD3("MlpPolicy", env, verbose=0, seed=seed,
                   buffer_size=200000, batch_size=256, tau=0.005,
                   train_freq=1, gradient_steps=1, learning_starts=1000,
                   policy_delay=2, target_policy_noise=0.2, target_noise_clip=0.5,
                   policy_kwargs={"net_arch": arch},
                   tensorboard_log=tensorboard_log, **COMMON)

    raise ValueError("未知算法 %r，可选：%s" % (name, ", ".join(ALGOS)))


def arch_str(arch) -> str:
    """[64, 64] -> '64x64'（用于文件名与表格）."""
    return "x".join(str(int(x)) for x in arch)
