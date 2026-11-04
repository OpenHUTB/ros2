#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DQN（Deep Q-Network）算法实现：经验回放 + 目标网络 + ε-greedy 探索.

算法要点（论文：Mnih et al., "Human-level control through deep reinforcement
learning", Nature 518:529-533, 2015）：
1. 经验回放（Replay Buffer）：打破样本相关性，提高样本利用率；
2. 目标网络（Target Network）：固定 Q_target 若干步，缓解自举发散；
3. 损失函数：L = E[(r + γ·max_a' Q_target(s', a') - Q(s, a))²]，仅对 Q 网络反传；
4. ε-greedy 探索：ε 从 ε_start 线性衰减到 ε_end。

本实现不依赖 stable_baselines3，仅依赖 torch，代码量小、便于课程原理讲解。
"""
from __future__ import annotations

import random
from collections import deque
from typing import Deque, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

from policy import N_ACTIONS, build_q_network, extract_torch_weights


class ReplayBuffer(object):
    """固定容量环形经验回放池：存 (s, a, r, s2, done)."""

    def __init__(self, capacity: int = 50000):
        self.buf: Deque[Tuple] = deque(maxlen=capacity)

    def push(self, s, a, r, s2, done):
        self.buf.append((s, a, r, s2, done))

    def sample(self, batch_size: int):
        batch = random.sample(self.buf, batch_size)
        s = torch.as_tensor(np.stack([b[0] for b in batch]), dtype=torch.float32)
        a = torch.as_tensor(np.array([b[1] for b in batch], dtype=np.int64))
        r = torch.as_tensor(np.array([b[2] for b in batch], dtype=np.float32))
        s2 = torch.as_tensor(np.stack([b[3] for b in batch]), dtype=torch.float32)
        done = torch.as_tensor(np.array([b[4] for b in batch], dtype=np.float32))
        return s, a, r, s2, done

    def __len__(self):
        return len(self.buf)


class DQNAgent(object):
    """DQN 智能体：Q 网络 + 目标网络 + 经验回放 + ε-greedy."""

    def __init__(
        self,
        obs_dim: int,
        n_actions: int = N_ACTIONS,
        lr: float = 1e-3,
        gamma: float = 0.99,
        tau: float = 0.005,          # 目标网络软更新系数
        update_freq: int = 4,        # 每 N 步训练一次
        batch_size: int = 128,
        buffer_capacity: int = 50000,
        epsilon_start: float = 1.0,
        epsilon_end: float = 0.05,
        epsilon_decay_steps: int = 100000,
        hidden=(64, 64),
        seed: int = 42,
    ):
        self.n_actions = n_actions
        self.gamma = gamma
        self.tau = tau
        self.update_freq = update_freq
        self.batch_size = batch_size
        self.epsilon_start = epsilon_start
        self.epsilon_end = epsilon_end
        self.epsilon_decay_steps = max(1, epsilon_decay_steps)

        torch.manual_seed(seed)
        np.random.seed(seed)
        random.seed(seed)

        self.q_net = build_q_network(obs_dim=obs_dim, n_actions=n_actions,
                                     hidden=hidden)
        self.target_net = build_q_network(obs_dim=obs_dim, n_actions=n_actions,
                                          hidden=hidden)
        self.target_net.load_state_dict(self.q_net.state_dict())
        self.target_net.eval()

        self.optimizer = optim.Adam(self.q_net.parameters(), lr=lr)
        self.buffer = ReplayBuffer(buffer_capacity)
        self.steps = 0

    # ------------------------------------------------------------------ 探索与动作
    def epsilon(self) -> float:
        """当前 ε（线性衰减，到达 ε_end 后保持）."""
        frac = min(1.0, self.steps / self.epsilon_decay_steps)
        return self.epsilon_start + frac * (self.epsilon_end - self.epsilon_start)

    def select_action(self, obs, training: bool = True) -> int:
        """ε-greedy 选动作：概率 ε 随机探索，否则取 argmax Q."""
        if training and random.random() < self.epsilon():
            return random.randrange(self.n_actions)
        with torch.no_grad():
            q = self.q_net(torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0))
            return int(q.argmax(dim=1).item())

    # ------------------------------------------------------------------ 学习
    def remember(self, s, a, r, s2, done):
        self.buffer.push(s, a, r, s2, float(done))

    def update(self) -> float:
        """从回放池采样并更新 Q 网络一次，返回 loss（无足够样本时返回 0）."""
        if len(self.buffer) < self.batch_size:
            return 0.0
        s, a, r, s2, done = self.buffer.sample(self.batch_size)

        with torch.no_grad():
            # 目标值：r + γ·max_a' Q_target(s', a')（done 后无未来回报）
            q_next = self.target_net(s2).max(dim=1).values
            target = r + self.gamma * q_next * (1.0 - done)

        q = self.q_net(s).gather(1, a.unsqueeze(1)).squeeze(1)
        loss = nn.functional.mse_loss(q, target)

        self.optimizer.zero_grad()
        loss.backward()
        # 梯度裁剪：稳定训练
        nn.utils.clip_grad_norm_(self.q_net.parameters(), 10.0)
        self.optimizer.step()
        return float(loss.item())

    def maybe_train(self) -> float:
        """每 update_freq 步训练一次（训练主循环每步调用）."""
        self.steps += 1
        if self.steps % self.update_freq == 0:
            loss = self.update()
            self._soft_update()
            return loss
        return 0.0

    def _soft_update(self):
        """目标网络软更新：θ_target ← τ·θ + (1-τ)·θ_target."""
        with torch.no_grad():
            for p, tp in zip(self.q_net.parameters(), self.target_net.parameters()):
                tp.data.mul_(1.0 - self.tau).add_(self.tau * p.data)

    # ------------------------------------------------------------------ 保存 / 加载
    def save(self, path: str, export_npz: str = ""):
        """保存 torch 权重；如给定 export_npz 路径，同时导出纯 numpy 权重."""
        torch.save(self.q_net.state_dict(), path)
        if export_npz:
            np.savez(export_npz, **extract_torch_weights(self.q_net))

    def load(self, path: str):
        self.q_net.load_state_dict(torch.load(path, map_location="cpu"))
        self.target_net.load_state_dict(self.q_net.state_dict())
