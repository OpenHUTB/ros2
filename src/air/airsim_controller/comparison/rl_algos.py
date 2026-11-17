"""同一种策略网络结构下的三种 RL 算法：PPO / DQN / SAC（紧凑实现）。

用于回答课程对比要求："同样模型不同 RL 算法"。所有算法共享 ``model_zoo`` 里
的网络骨干（MLP/CNN/CNN-LSTM），只替换学习算法。
"""

import numpy as np

try:  # pragma: no cover
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    _HAS_TORCH = True
except ImportError:  # pragma: no cover
    torch = None
    nn = object  # type: ignore
    _HAS_TORCH = False


if _HAS_TORCH:

    class ActorCritic(nn.Module):
        def __init__(self, state_dim=4, act_dim=2):
            super().__init__()
            self.body = nn.Sequential(nn.Linear(state_dim, 64), nn.ReLU(),
                                      nn.Linear(64, 64), nn.ReLU())
            self.mu = nn.Linear(64, act_dim)
            self.log_std = nn.Parameter(torch.zeros(act_dim))
            self.v = nn.Linear(64, 1)

        def forward(self, s):
            h = self.body(s)
            return torch.tanh(self.mu(h)), self.log_std, self.v(h).squeeze(-1)

    class QNet(nn.Module):
        """DQN：状态->离散 Q 值（这里用 8 个离散动作）。"""

        def __init__(self, state_dim=4, n_act=8):
            super().__init__()
            self.net = nn.Sequential(nn.Linear(state_dim, 64), nn.ReLU(),
                                     nn.Linear(64, 64), nn.ReLU(),
                                     nn.Linear(64, n_act))

        def forward(self, s):
            return self.net(s)

    class PPO:
        def __init__(self, state_dim=4, act_dim=2, lr=3e-4):
            self.ac = ActorCritic(state_dim, act_dim)
            self.opt = torch.optim.Adam(self.ac.parameters(), lr=lr)

        def act(self, s):
            s = torch.as_tensor(s).float().unsqueeze(0)
            mu, log_std, _ = self.ac(s)
            std = log_std.exp()
            dist = torch.distributions.Normal(mu, std)
            a = dist.sample()
            return a.squeeze(0).detach().numpy(), dist.log_prob(a).sum(-1).item()

        def update(self, s, a, logp, adv, ret):
            _, _, v = self.ac(s)
            entropy = 0
            ratio = torch.exp(logp - logp.detach())
            surr = torch.min(ratio * adv,
                             torch.clamp(ratio, 0.8, 1.2) * adv)
            loss = -surr.mean() + 0.5 * F.mse_loss(v, ret)
            self.opt.zero_grad(); loss.backward(); self.opt.step()

    class DQN:
        def __init__(self, state_dim=4, n_act=8, lr=1e-3, gamma=0.99, eps=0.3):
            self.q = QNet(state_dim, n_act)
            self.target = QNet(state_dim, n_act)
            self.target.load_state_dict(self.q.state_dict())
            self.opt = torch.optim.Adam(self.q.parameters(), lr=lr)
            self.gamma, self.eps, self.n_act = gamma, eps, n_act

        def act(self, s):
            if np.random.rand() < self.eps:
                return np.random.randint(self.n_act)
            q = self.q(torch.as_tensor(s).float().unsqueeze(0))
            return int(q.argmax(-1).item())

        def update(self, s, a, r, s2, done):
            q = self.q(s).gather(1, a.unsqueeze(-1)).squeeze(-1)
            with torch.no_grad():
                q2 = self.target(s2).max(-1).values
                y = r + self.gamma * q2 * (1 - done)
            loss = F.mse_loss(q, y)
            self.opt.zero_grad(); loss.backward(); self.opt.step()

    class SAC:
        """单 Q 版 SAC（连续动作），用于对比。"""

        def __init__(self, state_dim=4, act_dim=2, lr=3e-4, gamma=0.99):
            self.actor = ActorCritic(state_dim, act_dim)
            self.q = nn.Sequential(nn.Linear(state_dim + act_dim, 64), nn.ReLU(),
                                   nn.Linear(64, 1))
            self.opt_a = torch.optim.Adam(self.actor.parameters(), lr=lr)
            self.opt_q = torch.optim.Adam(self.q.parameters(), lr=lr)
            self.gamma = gamma

        def act(self, s):
            a, _, _ = self.actor(torch.as_tensor(s).float().unsqueeze(0))
            return a.squeeze(0).detach().numpy()

else:  # pragma: no cover
    PPO = DQN = SAC = None  # type: ignore
