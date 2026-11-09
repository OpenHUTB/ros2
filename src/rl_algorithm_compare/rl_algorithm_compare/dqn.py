"""DQN（深度 Q 网络）的纯 NumPy 实现。

包含经验回放、目标网络、epsilon-greedy 探索与 Adam 更新，
用于训练 CartPole 的神经网络控制策略。
"""
import numpy as np

from .nn import MLP


class ReplayBuffer:
    """经验回放缓冲区。"""

    def __init__(self, cap=50000):
        self.cap = cap
        self.s, self.a, self.r, self.s2, self.d = [], [], [], [], []

    def push(self, s, a, r, s2, d):
        self.s.append(s)
        self.a.append(a)
        self.r.append(r)
        self.s2.append(s2)
        self.d.append(d)
        if len(self.s) > self.cap:
            for buf in (self.s, self.a, self.r, self.s2, self.d):
                buf.pop(0)

    def __len__(self):
        return len(self.s)

    def sample(self, n):
        idx = np.random.choice(len(self.s), n, replace=False)
        return (np.array(self.s)[idx], np.array(self.a)[idx],
                np.array(self.r)[idx], np.array(self.s2)[idx],
                np.array(self.d)[idx])


class DQN:
    """深度 Q 网络智能体。layers 为隐藏层大小列表。"""

    def __init__(self, obs_dim, act_dim, layers, lr=1e-3, gamma=0.99,
                 eps=1.0, eps_min=0.01, eps_decay=0.995,
                 batch_size=64, target_sync=200):
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.layers = list(layers)
        self.gamma = gamma
        self.eps = eps
        self.eps_min = eps_min
        self.eps_decay = eps_decay
        self.batch_size = batch_size
        self.target_sync = target_sync
        self.n_updates = 0

        size = [obs_dim] + list(layers) + [act_dim]
        self.q = MLP(size, out='linear', lr=lr)
        self.q_target = MLP(size, out='linear', lr=lr)
        self.sync_target()
        self.buffer = ReplayBuffer()

    def sync_target(self):
        self.q_target.W = [w.copy() for w in self.q.W]
        self.q_target.b = [b.copy() for b in self.q.b]

    def act(self, s, greedy=False):
        """epsilon-greedy 选动作；greedy=True 时取 argmax。"""
        if not greedy and np.random.rand() < self.eps:
            return int(np.random.randint(self.act_dim))
        q = self.q.predict(s).ravel()
        return int(np.argmax(q))

    def store(self, s, a, r, s2, d):
        self.buffer.push(s, a, r, s2, d)

    def update(self):
        """从回放采样一批做一次 Q 学习更新（目标网络软/硬同步）。"""
        if len(self.buffer) < self.batch_size:
            return
        s, a, r, s2, d = self.buffer.sample(self.batch_size)
        q_next = self.q_target.predict(s2).max(axis=1)
        target = r + self.gamma * q_next * (1 - d)
        q = self.q.predict(s)
        # 构造输出层梯度：只对所选动作列给 (q - target) 的梯度
        dy = np.zeros_like(q)
        q_sel = q[np.arange(len(a)), a]
        dy[np.arange(len(a)), a] = (q_sel - target) / len(a)
        gW, gb = self.q.backward(s, dy)
        self.q.adam_update(gW, gb)
        self.n_updates += 1
        self.eps = max(self.eps_min, self.eps * self.eps_decay)
        if self.n_updates % self.target_sync == 0:
            self.sync_target()

    def save(self, path):
        self.q.save(path)

    def load(self, path):
        self.q.load(path)
        self.sync_target()
        return self
