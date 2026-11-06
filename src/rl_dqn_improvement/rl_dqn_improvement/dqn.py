"""DQN 及其改进算法（Double DQN / Dueling DQN）的纯 NumPy 实现。

本模块在同一 CartPole 环境上对比三种 DQN 变体：
  - dqn     ：标准 DQN，用目标网络取 max 计算 TD 目标
  - double  ：Double DQN，用在线网络选动作、目标网络评估，缓解过估计
  - dueling ：Dueling DQN，网络输出分裂为状态价值 V 与优势 A 两个流

三者共享经验回放、目标网络与 epsilon-greedy 探索，差异仅在 TD
目标的构造（double）或网络输出结构（dueling）上，从而公平对比。
"""
import pickle

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
    """深度 Q 网络智能体，支持 dqn / double / dueling 三种变体。"""

    def __init__(self, obs_dim, act_dim, layers, variant='dqn', lr=1e-3,
                 gamma=0.99, eps=1.0, eps_min=0.01, eps_decay=0.995,
                 batch_size=64, target_sync=200):
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.layers = list(layers)
        self.variant = variant
        self.gamma = gamma
        self.eps = eps
        self.eps_min = eps_min
        self.eps_decay = eps_decay
        self.batch_size = batch_size
        self.target_sync = target_sync
        self.n_updates = 0

        # dueling 需要多输出一维：V + 每个动作的优势 A
        out_dim = act_dim if variant != 'dueling' else act_dim + 1
        size = [obs_dim] + list(layers) + [out_dim]
        self.q = MLP(size, out='linear', lr=lr)
        self.q_target = MLP(size, out='linear', lr=lr)
        self.sync_target()
        self.buffer = ReplayBuffer()

    def sync_target(self):
        self.q_target.W = [w.copy() for w in self.q.W]
        self.q_target.b = [b.copy() for b in self.q.b]

    def _q_values(self, net, x):
        """由网络原始输出计算各动作的 Q 值（dueling 时组合 V 与 A）。"""
        out = net.predict(x)
        if self.variant != 'dueling':
            return out
        # Q(a) = V + (A(a) - mean(A))
        V = out[:, 0:1]
        A = out[:, 1:]
        return V + (A - A.mean(axis=1, keepdims=True))

    def act(self, s, greedy=False):
        """epsilon-greedy 选动作；greedy=True 时取 argmax。"""
        if not greedy and np.random.rand() < self.eps:
            return int(np.random.randint(self.act_dim))
        q = self._q_values(self.q, s).ravel()
        return int(np.argmax(q))

    def store(self, s, a, r, s2, d):
        self.buffer.push(s, a, r, s2, d)

    def _td_target(self, r, s2, d):
        """构造 TD 目标。double 用在线网络选动作、目标网络评估。"""
        n = len(r)
        if self.variant == 'double':
            a2 = self.q.predict(s2).argmax(axis=1)
            q_next = self._q_values(self.q_target, s2)[np.arange(n), a2]
        else:
            q_next = self._q_values(self.q_target, s2).max(axis=1)
        return r + self.gamma * q_next * (1 - d)

    def update(self):
        """从回放采样一批做一次 Q 学习更新。"""
        if len(self.buffer) < self.batch_size:
            return
        s, a, r, s2, d = self.buffer.sample(self.batch_size)
        target = self._td_target(r, s2, d)

        if self.variant == 'dueling':
            # 反向：Q_a = V + A_a - mean(A)，把 dQ_a 分摊回 V 和 A 流
            out = self.q.predict(s)          # [V, A0, A1, ...]
            q = self._q_values(self.q, s)
            dy = np.zeros_like(out)
            K = self.act_dim
            for i in range(len(a)):
                ai = a[i]
                dq_i = (q[i, ai] - target[i]) / len(a)
                dy[i, 0] += dq_i                          # dV
                for j in range(K):                        # dA
                    coeff = 1.0 - 1.0 / K if j == ai else -1.0 / K
                    dy[i, 1 + j] += dq_i * coeff
            gW, gb = self.q.backward(s, dy)
        else:
            q = self.q.predict(s)
            dy = np.zeros_like(q)
            dy[np.arange(len(a)), a] = (q[np.arange(len(a)), a] - target) / len(a)
            gW, gb = self.q.backward(s, dy)

        self.q.adam_update(gW, gb)
        self.n_updates += 1
        self.eps = max(self.eps_min, self.eps * self.eps_decay)
        if self.n_updates % self.target_sync == 0:
            self.sync_target()

    def save(self, path):
        with open(path, 'wb') as f:
            pickle.dump({'variant': self.variant, 'layers': self.layers,
                         'obs_dim': self.obs_dim, 'act_dim': self.act_dim,
                         'q': self.q}, f)

    def load(self, path):
        with open(path, 'rb') as f:
            d = pickle.load(f)
        self.variant = d['variant']
        self.layers = d['layers']
        self.obs_dim = d['obs_dim']
        self.act_dim = d['act_dim']
        self.q = d['q']
        self.q_target = MLP(self.q.sizes, out='linear', lr=1e-3)
        self.sync_target()
        return self
