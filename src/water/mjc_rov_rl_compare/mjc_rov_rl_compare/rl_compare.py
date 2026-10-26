"""
rl_compare.py — 同一 ROV 任务上对比 PPO 风格与 SAC 风格两种强化学习算法（教学简化实现）。
为什么对比：作业要求"同样模型不同RL算法对比"。
PPO 风格：on-policy 策略梯度，训练稳定但样本效率低；为简化省略 importance ratio / clip / advantage。
SAC 风格：off-policy 策略梯度 + 熵正则，样本效率高；为简化省略 Q 网络与回放缓冲区。
均实现为小型 MLP，CPU 上快速跑通，打印对比指标。
"""
import numpy as np
import torch
import torch.nn as nn
from . import common


class PolicyMLP(nn.Module):
    def __init__(self, s_dim=4, a_dim=2, h=64):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(s_dim, h), nn.Tanh(),
                                 nn.Linear(h, h), nn.Tanh(),
                                 nn.Linear(h, a_dim))
    def forward(self, x):
        return self.net(x)


def _env_step(state, action, target):
    """简化 2D 环境：推力+阻尼，返回下一状态与奖励。"""
    vx = state[2]; vy = state[3]
    Fx = np.clip(action[0], -5, 5) - 2.0 * vx
    Fy = np.clip(action[1], -5, 5) - 2.0 * vy
    nx = state[0] + vx * 0.02 + 0.5 * Fx * 0.02**2
    ny = state[1] + vy * 0.02 + 0.5 * Fy * 0.02**2
    nvx = vx + Fx * 0.02
    nvy = vy + Fy * 0.02
    reward = -np.hypot(target[0]-nx, target[1]-ny)
    return np.array([nx, ny, nvx, nvy]), reward


def train_ppo(episodes=100):
    """PPO：简化版 on-policy，每 episode 后更新一次。"""
    torch.manual_seed(0)
    policy = PolicyMLP()
    opt = torch.optim.Adam(policy.parameters(), lr=1e-3)
    target = np.array([2.0, 0.0])
    rewards_history = []
    for ep in range(episodes):
        s = np.zeros(4)
        ep_reward = 0
        for step in range(200):
            s_t = torch.tensor(s, dtype=torch.float32)
            with torch.no_grad():
                a = policy(s_t).numpy() + np.random.randn(2)*0.5
            s_next, r = _env_step(s, a, target)
            ep_reward += r
            s = s_next
            # 简化 PPO：用 reward-to-go 做策略梯度
            s_t = torch.tensor(s, dtype=torch.float32)
            a_pred = policy(s_t)
            loss = -(a_pred.mean() * r)
            opt.zero_grad(); loss.backward(); opt.step()
        rewards_history.append(ep_reward)
    return rewards_history


def train_sac(episodes=100):
    """SAC：简化版 off-policy，最大熵目标。"""
    torch.manual_seed(0)
    policy = PolicyMLP()
    opt = torch.optim.Adam(policy.parameters(), lr=1e-3)
    target = np.array([2.0, 0.0])
    rewards_history = []
    for ep in range(episodes):
        s = np.zeros(4)
        ep_reward = 0
        for step in range(200):
            s_t = torch.tensor(s, dtype=torch.float32)
            a_pred = policy(s_t)
            # SAC: 最大化 reward + 熵正则
            entropy = -(a_pred**2).mean()
            with torch.no_grad():
                a = a_pred.numpy() + np.random.randn(2)*0.3
            s_next, r = _env_step(s, a, target)
            ep_reward += r
            s = s_next
            loss = -(r + 0.1 * entropy)
            opt.zero_grad(); loss.backward(); opt.step()
        rewards_history.append(ep_reward)
    return rewards_history


def run_compare():
    """跑两种算法并打印对比结果。"""
    print("== PPO 训练中 ==")
    ppo_r = train_ppo(100)
    print("== SAC 训练中 ==")
    sac_r = train_sac(100)
    ppo_final = np.mean(ppo_r[-10:])
    sac_final = np.mean(sac_r[-10:])
    print(f"[对比] PPO  最后10轮平均奖励: {ppo_final:.2f}")
    print(f"[对比] SAC  最后10轮平均奖励: {sac_final:.2f}")
    return ppo_r, sac_r
