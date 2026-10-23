"""REINFORCE（蒙特卡洛策略梯度）的纯 NumPy 实现。

策略网络输出 softmax 动作概率，用完整回合的折扣回报估计梯度，
并减去回报均值作为基线以降低方差。
"""
import numpy as np

from .nn import MLP


class REINFORCE:
    """REINFORCE 智能体。layers 为策略网络隐藏层大小列表。"""

    def __init__(self, obs_dim, act_dim, layers, lr=1e-3, gamma=0.99):
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.layers = list(layers)
        self.gamma = gamma
        size = [obs_dim] + list(layers) + [act_dim]
        self.policy = MLP(size, out='softmax', lr=lr)

    def act(self, s, greedy=False):
        """按策略概率采样动作；greedy=True 时取概率最大动作。"""
        p = self.policy.predict(s).ravel()
        if greedy:
            return int(np.argmax(p))
        return int(np.random.choice(self.act_dim, p=p))

    def update(self, trajectory):
        """用一整条回合轨迹更新策略。trajectory: [(s,a,r), ...]。"""
        X = np.array([t[0] for t in trajectory])
        A = np.array([t[1] for t in trajectory])
        rewards = np.array([t[2] for t in trajectory])

        # 计算折扣回报 G_t
        G = np.zeros(len(rewards))
        g = 0.0
        for i in reversed(range(len(rewards))):
            g = rewards[i] + self.gamma * g
            G[i] = g
        G = G - G.mean()  # 基线（减方差）

        prob, _ = self.policy.forward(X)
        # ∇ log π(a|s) 对 softmax 输出 = one_hot - prob
        one_hot = np.zeros_like(prob)
        one_hot[np.arange(len(A)), A] = 1
        dy = (one_hot - prob) * G[:, None]
        gW, gb = self.policy.backward(X, dy)
        self.policy.adam_update(gW, gb)

    def save(self, path):
        self.policy.save(path)

    def load(self, path):
        self.policy.load(path)
        return self
