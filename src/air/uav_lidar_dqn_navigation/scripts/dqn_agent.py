#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DQN 智能体：经验回放 + 目标网络 + Double DQN

与普通 Q-learning 相比的三处关键改动：
  1. 经验回放：把 (s, a, r, s', done) 存进环形缓冲区，随机抽样打破时间相关性
  2. 目标网络：单独维护一份参数，每 target_sync 步同步一次，抑制自举发散
  3. Double DQN：用在线网络挑动作、目标网络估价值，缓解 Q 值高估
"""

import random
from collections import deque

import numpy as np
import torch
import torch.nn as nn

from dqn_network import DuelingQNet, ImageQNet, NUM_ACTIONS


class ReplayBuffer(object):
    def __init__(self, capacity=50000):
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        s, a, r, s2, d = zip(*batch)
        return (np.stack(s).astype(np.float32),
                np.array(a, dtype=np.int64),
                np.array(r, dtype=np.float32),
                np.stack(s2).astype(np.float32),
                np.array(d, dtype=np.float32))

    def __len__(self):
        return len(self.buffer)


class DQNAgent(object):
    def __init__(self, state_dim=32, action_dim=NUM_ACTIONS, lr=5e-4,
                 gamma=0.99, batch_size=64, capacity=50000,
                 target_sync=500, hidden=128, device=None):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.gamma = gamma
        self.batch_size = batch_size
        self.target_sync = target_sync
        self.steps = 0

        self.device = device or torch.device('cpu')
        self.online = DuelingQNet(state_dim, action_dim, hidden).to(self.device)
        self.target = DuelingQNet(state_dim, action_dim, hidden).to(self.device)
        self.target.load_state_dict(self.online.state_dict())
        self.target.eval()

        self.optimizer = torch.optim.Adam(self.online.parameters(), lr=lr)
        self.loss_fn = nn.SmoothL1Loss()
        self.memory = ReplayBuffer(capacity)

    def q_values(self, state):
        """返回 6 个动作的 Q 值，供安全层做动作筛选。"""
        with torch.no_grad():
            tensor = torch.as_tensor(state, dtype=torch.float32,
                                     device=self.device).unsqueeze(0)
            return self.online(tensor)[0].cpu().numpy()

    def act(self, state, epsilon=0.0):
        if random.random() < epsilon:
            return random.randrange(self.action_dim)
        return int(self.q_values(state).argmax())

    def remember(self, state, action, reward, next_state, done):
        self.memory.push(state, action, reward, next_state, done)

    def can_learn(self):
        return len(self.memory) >= max(self.batch_size, 500)

    def learn(self):
        s, a, r, s2, d = self.memory.sample(self.batch_size)
        s = torch.as_tensor(s, device=self.device)
        a = torch.as_tensor(a, device=self.device)
        r = torch.as_tensor(r, device=self.device)
        s2 = torch.as_tensor(s2, device=self.device)
        d = torch.as_tensor(d, device=self.device)

        q = self.online(s).gather(1, a.unsqueeze(1)).squeeze(1)
        with torch.no_grad():
            # Double DQN：在线网络选动作，目标网络给价值
            next_actions = self.online(s2).argmax(dim=1, keepdim=True)
            next_q = self.target(s2).gather(1, next_actions).squeeze(1)
            target = r + self.gamma * (1.0 - d) * next_q

        loss = self.loss_fn(q, target)
        self.optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(self.online.parameters(), 10.0)
        self.optimizer.step()

        self.steps += 1
        if self.steps % self.target_sync == 0:
            self.target.load_state_dict(self.online.state_dict())
        return float(loss.item())

    def save(self, path):
        torch.save({
            'state_dim': self.state_dim,
            'action_dim': self.action_dim,
            'model': self.online.state_dict(),
            # 续训需要优化器动量与步数，否则 Adam 状态归零
            'optimizer': self.optimizer.state_dict(),
            'steps': self.steps,
        }, path)

    def load(self, path):
        ckpt = torch.load(path, map_location=self.device)
        self.online.load_state_dict(ckpt['model'])
        self.target.load_state_dict(ckpt['model'])
        if 'optimizer' in ckpt:
            self.optimizer.load_state_dict(ckpt['optimizer'])
        if 'steps' in ckpt:
            self.steps = int(ckpt['steps'])
        return ckpt


class ImageReplayBuffer(object):
    """图像回放池：存 84x84 单通道图，容量按内存约束取小一些。"""

    def __init__(self, capacity=10000):
        self.buffer = deque(maxlen=capacity)

    def push(self, img, extra, action, reward, next_img, next_extra, done):
        self.buffer.append((img, extra, action, reward, next_img, next_extra, done))

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        img, extra, a, r, img2, extra2, d = zip(*batch)
        return (np.stack(img).astype(np.float32),
                np.stack(extra).astype(np.float32),
                np.array(a, dtype=np.int64),
                np.array(r, dtype=np.float32),
                np.stack(img2).astype(np.float32),
                np.stack(extra2).astype(np.float32),
                np.array(d, dtype=np.float32))

    def __len__(self):
        return len(self.buffer)


class ImageDQNAgent(object):
    """任务 4：以 84x84 灰度图为输入的端到端 DQN。"""

    def __init__(self, action_dim=NUM_ACTIONS, extra_dim=7, lr=2e-4,
                 gamma=0.99, batch_size=32, capacity=10000,
                 target_sync=400, device=None):
        self.action_dim = action_dim
        self.gamma = gamma
        self.batch_size = batch_size
        self.target_sync = target_sync
        self.steps = 0

        self.device = device or torch.device('cpu')
        self.online = ImageQNet(action_dim, extra_dim).to(self.device)
        self.target = ImageQNet(action_dim, extra_dim).to(self.device)
        self.target.load_state_dict(self.online.state_dict())
        self.target.eval()

        self.optimizer = torch.optim.Adam(self.online.parameters(), lr=lr)
        self.loss_fn = nn.SmoothL1Loss()
        self.memory = ImageReplayBuffer(capacity)

    def act(self, state, epsilon=0.0):
        img, extra = state
        if random.random() < epsilon:
            return random.randrange(self.action_dim)
        with torch.no_grad():
            img_t = torch.as_tensor(img, dtype=torch.float32,
                                    device=self.device).unsqueeze(0).unsqueeze(0)
            extra_t = torch.as_tensor(extra, dtype=torch.float32,
                                      device=self.device).unsqueeze(0)
            q = self.online(img_t, extra_t)
            return int(q.argmax(dim=1).item())

    def remember(self, state, action, reward, next_state, done):
        img, extra = state
        next_img, next_extra = next_state
        self.memory.push(img, extra, action, reward, next_img, next_extra, done)

    def can_learn(self):
        return len(self.memory) >= max(self.batch_size, 400)

    def learn(self):
        img, extra, a, r, img2, extra2, d = self.memory.sample(self.batch_size)
        img = torch.as_tensor(img, device=self.device).unsqueeze(1)
        extra = torch.as_tensor(extra, device=self.device)
        a = torch.as_tensor(a, device=self.device)
        r = torch.as_tensor(r, device=self.device)
        img2 = torch.as_tensor(img2, device=self.device).unsqueeze(1)
        extra2 = torch.as_tensor(extra2, device=self.device)
        d = torch.as_tensor(d, device=self.device)

        q = self.online(img, extra).gather(1, a.unsqueeze(1)).squeeze(1)
        with torch.no_grad():
            next_actions = self.online(img2, extra2).argmax(dim=1, keepdim=True)
            next_q = self.target(img2, extra2).gather(1, next_actions).squeeze(1)
            target = r + self.gamma * (1.0 - d) * next_q

        loss = self.loss_fn(q, target)
        self.optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(self.online.parameters(), 10.0)
        self.optimizer.step()

        self.steps += 1
        if self.steps % self.target_sync == 0:
            self.target.load_state_dict(self.online.state_dict())
        return float(loss.item())

    def save(self, path, include_optimizer=True):
        model = self.online.state_dict()
        if not include_optimizer:
            model = {name: value.half() if value.is_floating_point() else value
                     for name, value in model.items()}
        checkpoint = {
            'action_dim': self.action_dim,
            'model': model,
            'steps': self.steps,
            'mode': 'e2e',
        }
        if include_optimizer:
            checkpoint['optimizer'] = self.optimizer.state_dict()
        torch.save(checkpoint, path)

    def load(self, path):
        ckpt = torch.load(path, map_location=self.device)
        self.online.load_state_dict(ckpt['model'])
        self.target.load_state_dict(ckpt['model'])
        if 'optimizer' in ckpt:
            self.optimizer.load_state_dict(ckpt['optimizer'])
        if 'steps' in ckpt:
            self.steps = int(ckpt['steps'])
        return ckpt
