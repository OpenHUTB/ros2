"""REINFORCE（蒙特卡洛策略梯度）的纯 NumPy 实现。

策略网络输出 softmax 动作概率，用完整回合的折扣回报估计梯度，
并减去**跨回合**回报基线（running baseline）以降低方差。

修正说明（评审意见）：
- 原实现用 ``G - G.mean()`` 在单条轨迹内中心化；CartPole 每步奖励恒为 1，
  轨迹内中心化只反映"离回合结束还有多远"，与动作好坏无关。现改为跨回合的
  回报移动平均基线（baseline），让高于平均回报的回合获得正优势（鼓励）、
  低于平均回报的回合获得负优势（抑制）。
- 策略梯度上升要求 ``theta += lr * grad(log_pi) * adv``；本库的
  ``adam_update`` 做的是 ``theta -= lr * grad``，因此这里传入**负**的策略梯度
  （``dy = (prob - one_hot) * adv``），使参数实际向梯度上升方向更新。
"""
import numpy as np

from .nn import MLP


class REINFORCE:
    """REINFORCE 智能体。layers 为策略网络隐藏层大小列表。"""

    def __init__(self, obs_dim, act_dim, layers, lr=1e-3, gamma=0.99,
                 baseline_rate=0.05):
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.layers = list(layers)
        self.gamma = gamma
        self.baseline_rate = baseline_rate  # 跨回合基线的 EMA 更新率
        self.baseline = 0.0                 # 跨回合回报移动平均基线
        self.n_episodes = 0
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

        # 折扣回报 G_t = sum_{k>=0} gamma^k * r_{t+k}
        G = np.zeros(len(rewards))
        g = 0.0
        for i in reversed(range(len(rewards))):
            g = rewards[i] + self.gamma * g
            G[i] = g

        # 跨回合基线：整个回合回报的移动平均。
        # 单回合回报高于平均 → 优势为正（鼓励该回合动作序列）；
        # 低于平均 → 优势为负（抑制）。
        ep_return = float(G[0])  # 整回合的折扣回报
        if self.n_episodes == 0:
            self.baseline = ep_return
        else:
            self.baseline = ((1 - self.baseline_rate) * self.baseline
                             + self.baseline_rate * ep_return)
        self.n_episodes += 1
        adv = G - self.baseline  # 优势（跨回合去均值）

        prob, _ = self.policy.forward(X)
        # softmax 策略的 ∇ log π(a|s) = one_hot(a) - prob
        one_hot = np.zeros_like(prob)
        one_hot[np.arange(len(A)), A] = 1
        # 配合 adam_update(theta -= lr*grad) 实现策略梯度上升：
        # 传负梯度 dy = (prob - one_hot) * adv
        dy = (prob - one_hot) * adv[:, None]
        gW, gb = self.policy.backward(X, dy)
        self.policy.adam_update(gW, gb)

    def save(self, path):
        self.policy.save(path)

    def load(self, path):
        self.policy.load(path)
        return self
