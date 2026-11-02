"""PPO algorithm implemented in pure NumPy."""

import numpy as np
from .nn import MLP, Adam, clip_grad_norm

LOG_STD_MIN = -4.0
LOG_STD_MAX = 1.0


class RolloutBuffer:
    def __init__(self, obs_dim, act_dim, size, gamma=0.99, lam=0.95):
        self.obs_buf = np.zeros((size, obs_dim))
        self.act_buf = np.zeros((size, act_dim))
        self.rew_buf = np.zeros(size)
        self.val_buf = np.zeros(size)
        self.logp_buf = np.zeros(size)
        self.done_buf = np.zeros(size)
        self.trunc_buf = np.zeros(size)
        self.boot_buf = np.zeros(size)
        self.adv_buf = np.zeros(size)
        self.ret_buf = np.zeros(size)
        self.gamma = gamma
        self.lam = lam
        self.ptr = 0
        self.max_size = size

    def store(self, obs, act, rew, val, logp, done, truncated=0.0, boot_val=0.0):
        assert self.ptr < self.max_size
        self.obs_buf[self.ptr] = obs
        self.act_buf[self.ptr] = act
        self.rew_buf[self.ptr] = rew
        self.val_buf[self.ptr] = val
        self.logp_buf[self.ptr] = logp
        self.done_buf[self.ptr] = done
        # truncated：因时间上限被截断；boot_val：该步之后 V(next_obs) 的自举值。
        self.trunc_buf[self.ptr] = truncated
        self.boot_buf[self.ptr] = boot_val
        self.ptr += 1

    def finish_path(self, last_val=0.0):
        start = self.ptr - 1
        lastgaelam = 0.0
        for t in reversed(range(start + 1)):
            if self.trunc_buf[t] > 0:
                # 时间上限截断：回合是被外部打断而非真正终止，用 V(next_obs) 自举
                # （nextnonterminal 仍为 1），而不是把后继价值直接归零。
                nextnonterminal = 1.0
                nextvalue = self.boot_buf[t]
            else:
                nextnonterminal = 1.0 - self.done_buf[t]
                nextvalue = last_val if t == start else self.val_buf[t + 1]
            delta = self.rew_buf[t] + self.gamma * nextvalue * nextnonterminal - self.val_buf[t]
            # 回合边界（或本段末尾）必须清掉 λ 迹；terminated 时 nextnonterminal=0 已自动清零，
            # truncated 的自举值不为 0，必须在这里显式重置，否则迹会泄漏到下一回合。
            if t == start or self.done_buf[t] > 0:
                lastgaelam = delta
            else:
                lastgaelam = delta + self.gamma * self.lam * lastgaelam
            self.adv_buf[t] = lastgaelam
        self.ret_buf[:self.ptr] = self.adv_buf[:self.ptr] + self.val_buf[:self.ptr]

    def get(self, batch_size=64):
        """按随机顺序产出 minibatch；可重复调用（每次按当前缓冲区内容重新打乱），
        缓冲区指针由调用方在消费完毕后通过 reset() 清空，从而支持多轮（train_iters）复用。"""
        n = self.ptr
        if n == 0:
            return
        obs = self.obs_buf[:n].copy()
        act = self.act_buf[:n].copy()
        logp = self.logp_buf[:n].copy()
        ret = self.ret_buf[:n].copy()
        adv = self.adv_buf[:n].copy()
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        idx = np.arange(n)
        np.random.shuffle(idx)
        for start in range(0, n, batch_size):
            end = start + batch_size
            mb = idx[start:end]
            yield obs[mb], act[mb], logp[mb], ret[mb], adv[mb]

    def reset(self):
        """消费完一轮训练后清空缓冲区指针。"""
        self.ptr = 0


class PPO:
    def __init__(self, obs_dim, act_dim, hidden_sizes=(64, 64), actor_lr=3e-4, critic_lr=1e-3,
                 clip_ratio=0.2, target_kl=0.01, train_iters=1, gamma=0.99, lam=0.95, seed=0):
        self.actor = MLP([obs_dim] + list(hidden_sizes) + [act_dim], seed=seed)
        self.actor_logstd = np.zeros((1, act_dim)) - 0.5
        self.critic = MLP([obs_dim] + list(hidden_sizes) + [1], seed=seed + 1)
        self.clip_ratio = clip_ratio
        self.target_kl = target_kl
        self.train_iters = train_iters
        self.actor_lr = actor_lr
        self.critic_lr = critic_lr
        # 多轮复用同一份 rollout 时，Adam 的自适应步长比朴素 SGD 稳定得多
        self.actor_opt = Adam(self.actor.weights + self.actor.biases, lr=actor_lr)
        # log_std 同样是待优化参数，之前用朴素 SGD 更新：与网络权重的量级差异很大，
        # 同一学习率下既容易过冲也容易几乎不动，改为同样走 Adam。
        self.logstd_opt = Adam([self.actor_logstd], lr=actor_lr)
        self.critic_opt = Adam(self.critic.weights + self.critic.biases, lr=critic_lr)
        self.gamma = gamma
        self.lam = lam
        self.obs_dim = obs_dim
        self.act_dim = act_dim

    def act(self, obs, deterministic=False):
        obs = np.atleast_2d(obs)
        mu = np.tanh(self.actor.forward(obs))
        if deterministic:
            return mu[0], 0.0
        log_std = np.clip(self.actor_logstd, LOG_STD_MIN, LOG_STD_MAX)
        std = np.exp(log_std)
        pi = mu + std * np.random.randn(1, self.act_dim)
        pi = np.clip(pi, -1.0, 1.0)
        logp = self._logp(pi, mu, std)
        return pi[0], float(np.nan_to_num(logp[0], nan=0.0, posinf=0.0, neginf=0.0))

    def _logp(self, pi, mu, std):
        logp = -0.5 * (((pi - mu) / (std + 1e-8)) ** 2 + 2 * np.log(std) + np.log(2 * np.pi))
        return np.sum(logp, axis=-1)

    def update(self, buf: RolloutBuffer):
        # 同一份 rollout 复用 train_iters 轮（每轮重新打乱 minibatch 顺序）；
        # 缓冲区指针在全部轮次结束后才清空，避免第 2 轮起读到空数据。
        for _ in range(self.train_iters):
            for obs, act, old_logp, ret, adv in buf.get(batch_size=64):
                # Critic update
                v_pred = self.critic.forward(obs)
                grad_v = clip_grad_norm(2.0 * (v_pred - ret.reshape(-1, 1)) / len(obs), 10.0)
                self.critic_opt.step(self.critic.grads(grad_v))

                # Actor update
                mu = np.tanh(self.actor.forward(obs))
                std = np.exp(np.clip(self.actor_logstd, LOG_STD_MIN, LOG_STD_MAX))
                logp = self._logp(act, mu, std)
                ratio = np.exp(np.clip(logp - old_logp, -20, 20))
                kl = np.mean(old_logp - logp)
                if not np.isfinite(kl) or kl > self.target_kl:
                    break

                # Clipped surrogate：越过 trust region 正向边界的样本不再产生梯度，
                # 这是多轮（train_iters）复用同一份 rollout 不发散的关键。
                eps = self.clip_ratio
                active = np.where(adv > 0, ratio < 1.0 + eps, ratio > 1.0 - eps)
                weight = np.where(active, adv * ratio, 0.0)
                dmu = weight.reshape(-1, 1) * (act - mu) / (std ** 2 + 1e-8) / len(obs)
                dlogstd = weight.reshape(-1, 1) * (((act - mu) / (std + 1e-8)) ** 2 - 1) / len(obs)
                # tanh 压缩的雅可比：梯度要穿过 mu = tanh(z) 这一层
                dmu = dmu * (1.0 - mu ** 2)
                dmu = clip_grad_norm(np.nan_to_num(dmu, nan=0.0, posinf=0.0, neginf=0.0), 1.0)
                dlogstd = np.nan_to_num(dlogstd, nan=0.0, posinf=0.0, neginf=0.0)
                self.actor_opt.step(self.actor.grads(dmu))
                self.logstd_opt.step([np.mean(dlogstd, axis=0, keepdims=True)])
                # Adam 持有的是参数数组的引用，必须原地裁剪；重新赋值会让优化器
                # 继续更新旧数组，log_std 从此与网络脱钩。
                np.clip(self.actor_logstd, LOG_STD_MIN, LOG_STD_MAX, out=self.actor_logstd)
        buf.reset()

    def save(self, path):
        self.actor.save(path + "_actor.npz")
        np.save(path + "_logstd.npy", self.actor_logstd)
        self.critic.save(path + "_critic.npz")

    def load(self, path):
        self.actor.load(path + "_actor.npz")
        self.actor_logstd = np.load(path + "_logstd.npy")
        self.critic.load(path + "_critic.npz")
        # load 之后参数数组已经是新对象，优化器必须重新绑定，否则之后所有更新的
        # 都会打到已废弃的旧数组上（权重看起来像没在训练）。
        self.actor_opt = Adam(self.actor.weights + self.actor.biases, lr=self.actor_lr)
        self.logstd_opt = Adam([self.actor_logstd], lr=self.actor_lr)
        self.critic_opt = Adam(self.critic.weights + self.critic.biases, lr=self.critic_lr)
