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
        self.adv_buf = np.zeros(size)
        self.ret_buf = np.zeros(size)
        self.gamma = gamma
        self.lam = lam
        self.ptr = 0
        self.max_size = size

    def store(self, obs, act, rew, val, logp, done):
        assert self.ptr < self.max_size
        self.obs_buf[self.ptr] = obs
        self.act_buf[self.ptr] = act
        self.rew_buf[self.ptr] = rew
        self.val_buf[self.ptr] = val
        self.logp_buf[self.ptr] = logp
        self.done_buf[self.ptr] = done
        self.ptr += 1

    def finish_path(self, last_val=0.0):
        start = self.ptr - 1
        lastgaelam = 0.0
        for t in reversed(range(start + 1)):
            if t == start:
                nextnonterminal = 1.0 - self.done_buf[t]
                nextvalue = last_val
            else:
                nextnonterminal = 1.0 - self.done_buf[t]
                nextvalue = self.val_buf[t + 1]
            delta = self.rew_buf[t] + self.gamma * nextvalue * nextnonterminal - self.val_buf[t]
            lastgaelam = delta + self.gamma * self.lam * nextnonterminal * lastgaelam
            self.adv_buf[t] = lastgaelam
        self.ret_buf[:self.ptr] = self.adv_buf[:self.ptr] + self.val_buf[:self.ptr]

    def get(self, batch_size=64):
        n = self.ptr
        self.ptr = 0
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


class PPO:
    def __init__(self, obs_dim, act_dim, hidden_sizes=(64, 64), actor_lr=3e-4, critic_lr=1e-3,
                 clip_ratio=0.2, target_kl=0.01, train_iters=10, gamma=0.99, lam=0.95, seed=0):
        self.actor = MLP([obs_dim] + list(hidden_sizes) + [act_dim], seed=seed)
        self.actor_logstd = np.zeros((1, act_dim)) - 0.5
        self.critic = MLP([obs_dim] + list(hidden_sizes) + [1], seed=seed + 1)
        self.clip_ratio = clip_ratio
        self.target_kl = target_kl
        self.train_iters = train_iters
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
        for _ in range(self.train_iters):
            for obs, act, old_logp, ret, adv in buf.get(batch_size=64):
                # Critic update
                v_pred = self.critic.forward(obs)
                grad_v = clip_grad_norm(2.0 * (v_pred - ret.reshape(-1, 1)) / len(obs), 10.0)
                self.critic.backward(grad_v, lr=1e-3)

                # Actor update
                mu = np.tanh(self.actor.forward(obs))
                std = np.exp(np.clip(self.actor_logstd, LOG_STD_MIN, LOG_STD_MAX))
                logp = self._logp(act, mu, std)
                ratio = np.clip(np.exp(np.clip(logp - old_logp, -20, 20)), 0.0, 10.0)
                kl = np.mean(old_logp - logp)
                if not np.isfinite(kl) or kl > self.target_kl:
                    break

                # Policy gradient (simplified, using advantage-weighted gradient)
                dmu = adv.reshape(-1, 1) * ratio.reshape(-1, 1) * (act - mu) / (std ** 2 + 1e-8) / len(obs)
                dlogstd = adv.reshape(-1, 1) * ratio.reshape(-1, 1) * (((act - mu) / (std + 1e-8)) ** 2 - 1) / len(obs)
                dmu = clip_grad_norm(np.nan_to_num(dmu, nan=0.0, posinf=0.0, neginf=0.0), 1.0)
                dlogstd = np.nan_to_num(dlogstd, nan=0.0, posinf=0.0, neginf=0.0)
                self.actor.backward(dmu, lr=3e-4)
                self.actor_logstd = np.clip(
                    self.actor_logstd - 3e-4 * np.mean(dlogstd, axis=0, keepdims=True),
                    LOG_STD_MIN, LOG_STD_MAX)

    def save(self, path):
        self.actor.save(path + "_actor.npz")
        np.save(path + "_logstd.npy", self.actor_logstd)
        self.critic.save(path + "_critic.npz")

    def load(self, path):
        self.actor.load(path + "_actor.npz")
        self.actor_logstd = np.load(path + "_logstd.npy")
        self.critic.load(path + "_critic.npz")
