"""Soft Actor-Critic implemented in pure NumPy.

Follows the reference SAC formulation:
  - squashed Gaussian policy with reparameterization trick
  - twin Q networks + Polyak-averaged targets
  - automatic temperature tuning with bounded log(alpha)
"""

import numpy as np

from .nn import MLP, soft_update, clip_grad_norm

LOG_STD_MIN = -5.0
LOG_STD_MAX = 2.0
LOG_ALPHA_MIN = np.log(0.01)
LOG_ALPHA_MAX = np.log(10.0)
LOG2PI = np.log(2.0 * np.pi)


class ReplayBuffer:
    def __init__(self, obs_dim, act_dim, capacity=200000):
        self.obs_buf = np.zeros((capacity, obs_dim), dtype=np.float64)
        self.next_obs_buf = np.zeros((capacity, obs_dim), dtype=np.float64)
        self.act_buf = np.zeros((capacity, act_dim), dtype=np.float64)
        self.rew_buf = np.zeros(capacity, dtype=np.float64)
        self.done_buf = np.zeros(capacity, dtype=np.float64)
        self.ptr = 0
        self.size = 0
        self.capacity = capacity

    def store(self, obs, act, rew, next_obs, done):
        self.obs_buf[self.ptr] = obs
        self.next_obs_buf[self.ptr] = next_obs
        self.act_buf[self.ptr] = act
        self.rew_buf[self.ptr] = rew
        self.done_buf[self.ptr] = float(done)
        self.ptr = (self.ptr + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size=256):
        idx = np.random.randint(0, self.size, size=batch_size)
        return (
            self.obs_buf[idx],
            self.act_buf[idx],
            self.rew_buf[idx],
            self.next_obs_buf[idx],
            self.done_buf[idx],
        )


class SAC:
    def __init__(self, obs_dim, act_dim, hidden_sizes=(128, 128), lr=3e-4, gamma=0.99,
                 tau=0.005, alpha=0.2, auto_alpha=True, seed=0):
        self.actor = MLP([obs_dim] + list(hidden_sizes) + [act_dim * 2], seed=seed)
        self.q1 = MLP([obs_dim + act_dim] + list(hidden_sizes) + [1], seed=seed + 1)
        self.q2 = MLP([obs_dim + act_dim] + list(hidden_sizes) + [1], seed=seed + 2)
        self.q1_target = MLP([obs_dim + act_dim] + list(hidden_sizes) + [1], seed=seed + 11)
        self.q2_target = MLP([obs_dim + act_dim] + list(hidden_sizes) + [1], seed=seed + 12)
        self.q1_target.set_params(*self.q1.get_params())
        self.q2_target.set_params(*self.q2.get_params())

        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.gamma = gamma
        self.tau = tau
        self.lr = lr
        self.auto_alpha = auto_alpha
        self.log_alpha = float(np.log(alpha))
        self.target_entropy = -float(act_dim)
        self.rng = np.random.default_rng(seed + 7)
        self.update_count = 0

    # ---------- policy ----------
    def _policy(self, obs):
        out = self.actor.forward(obs)
        mean = out[:, :self.act_dim]
        log_std = np.clip(out[:, self.act_dim:], LOG_STD_MIN, LOG_STD_MAX)
        std = np.exp(log_std)
        eps = self.rng.standard_normal(mean.shape)
        z = mean + std * eps
        action = np.tanh(z)
        return action, eps, mean, std, log_std, z

    @staticmethod
    def _log_prob(eps, log_std, action):
        """log pi(a|s) for a squashed Gaussian using the reparameterized noise."""
        base = np.sum(-0.5 * eps ** 2 - log_std - 0.5 * LOG2PI, axis=1)
        squash = np.sum(np.log(1.0 - action ** 2 + 1e-6), axis=1)
        logp = base - squash
        return np.nan_to_num(logp, nan=-100.0, posinf=100.0, neginf=-100.0)

    def act(self, obs, deterministic=False):
        obs = np.atleast_2d(obs)
        if deterministic:
            out = self.actor.forward(obs)
            return np.tanh(out[:, :self.act_dim])[0]
        action, _, _, _, _, _ = self._policy(obs)
        return np.clip(action[0], -1.0, 1.0)

    # ---------- learning ----------
    def update(self, buf: ReplayBuffer, batch_size=128):
        if buf.size < 1000:
            return
        obs, act, rew, next_obs, done = buf.sample(batch_size)
        rew = rew.reshape(-1, 1)
        done = done.reshape(-1, 1)
        alpha = float(np.exp(self.log_alpha))

        # ---- critic target ----
        n_action, n_eps, _, _, n_log_std, _ = self._policy(next_obs)
        n_logp = self._log_prob(n_eps, n_log_std, n_action).reshape(-1, 1)
        next_x = np.concatenate([next_obs, n_action], axis=1)
        q_t = np.minimum(self.q1_target.forward(next_x), self.q2_target.forward(next_x))
        y = rew + self.gamma * (1.0 - done) * (q_t - alpha * n_logp)
        y = np.clip(np.nan_to_num(y, nan=0.0, posinf=1e4, neginf=-1e4), -1e4, 1e4)

        # ---- critic update ----
        x = np.concatenate([obs, act], axis=1)
        q1_pred = self.q1.forward(x)
        q2_pred = self.q2.forward(x)
        g1 = clip_grad_norm(2.0 * (q1_pred - y) / len(obs), 10.0)
        g2 = clip_grad_norm(2.0 * (q2_pred - y) / len(obs), 10.0)
        self.q1.backward(g1, lr=self.lr)
        self.q2.backward(g2, lr=self.lr)

        # ---- actor update (reparameterized + tanh squashing) ----
        action, eps, mean, std, log_std, z = self._policy(obs)
        new_x = np.concatenate([obs, action], axis=1)
        q_new = np.minimum(self.q1.forward(new_x), self.q2.forward(new_x))
        # dQ/d(action) via backprop of Q1 without weight update
        dq_da = self.q1.backward(np.ones_like(q_new), lr=None)[:, -self.act_dim:]

        # J = alpha * log_pi - Q ; dJ/dz (z = mean + std*eps, a = tanh(z))
        dz = 2.0 * action * alpha - dq_da * (1.0 - action ** 2)
        d_mean = dz
        d_log_std = dz * eps - alpha
        grad_actor = np.concatenate([d_mean, d_log_std], axis=1)
        self.actor.backward(clip_grad_norm(grad_actor, 10.0), lr=self.lr)

        # ---- temperature update (bounded) ----
        if self.auto_alpha:
            logp_now = self._log_prob(eps, log_std, action)
            self.log_alpha = float(np.clip(
                self.log_alpha + self.lr * np.mean(logp_now + self.target_entropy),
                LOG_ALPHA_MIN, LOG_ALPHA_MAX))

        # ---- Polyak averaging ----
        # 注意：必须传入参数的引用（params()），get_params() 返回的是副本，
        # 对副本原地写入不会改变目标网络。
        soft_update(self.q1_target.params(), self.q1.params(), self.tau)
        soft_update(self.q2_target.params(), self.q2.params(), self.tau)
        self.update_count += 1

    # ---------- persistence ----------
    def save(self, path):
        self.actor.save(path + "_actor.npz")
        self.q1.save(path + "_q1.npz")
        self.q2.save(path + "_q2.npz")
        np.save(path + "_logalpha.npy", np.array([self.log_alpha]))

    def load(self, path):
        self.actor.load(path + "_actor.npz")
        self.q1.load(path + "_q1.npz")
        self.q2.load(path + "_q2.npz")
        self.q1_target.set_params(*self.q1.get_params())
        self.q2_target.set_params(*self.q2.get_params())
        la = path + "_logalpha.npy"
        try:
            self.log_alpha = float(np.load(la)[0])
        except Exception:
            pass
