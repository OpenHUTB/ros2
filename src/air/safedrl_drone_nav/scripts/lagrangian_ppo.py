"""
PPO-Lagrangian : constrained reinforcement learning for the navigation task.

Motivation
----------
A Constrained MDP (Altman, 1999) augments reward maximisation with an
expectation constraint on a *cost* signal::

        max_theta  E[ sum_t gamma^t r_t ]
        s.t.       E[ sum_t gamma^t c_t ]  <=  d

The standard primal-dual solution (Stooke, Achiam & Abbeel, 2020) forms the
Lagrangian

    L(theta, lam) = E[ sum gamma^t r_t ] - lam * ( E[ sum gamma^t c_t ] - d )

and performs alternating gradient *ascent* on ``theta`` with gradient *descent*
on ``lam``.  Operationally this is simply **PPO with an extra value head that
predicts the discounted cost**, plus a reward shifted by the current
multiplier::

        r'_t = r_t - lam * c_t

The multiplier follows a projected dual-ascent rule with an integral term so
that it reacts to *persistent* violation instead of oscillating on a single
unlucky episode::

        I_{k+1} = 0.9 I_k + 0.1 ( J_c(pi_k) - d )
        lam_{k+1} = clip( lam_k + eta * I_{k+1}, 0, lam_max )

Why re-implement ``train`` / ``collect_rollouts``
-------------------------------------------------
SB3's ``PPO`` has a single reward stream threaded through its ``RolloutBuffer``.
Rather than mutate that buffer's internals (fragile across SB3 versions), this
module keeps its own rollout storage and runs two Generalised Advantage
Estimations - one on the reward stream and one on the cost stream - from the
same trajectory.  The PPO update itself is the textbook clipped surrogate
loss, so results stay comparable with stock PPO.

The three rows of the ablation in the report differ only by flags:
``constrained=False`` (plain PPO), ``constrained=True`` (PPO-Lagrangian),
and ``EnvConfig.use_cbf=True`` on top (PPO-Lagrangian + CBF filter).
"""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np
import torch as th
import torch.nn as nn
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.utils import explained_variance, obs_as_tensor
from stable_baselines3.common.vec_env import VecEnv

from safety import LagrangeMultiplier


class CostValueNetwork(nn.Module):
    """Mirror of SB3's ``ValueNetwork``, predicting the discounted cost."""

    def __init__(self, feature_dim: int, net_arch: list[int] | None = None, activation=nn.Tanh):
        super().__init__()
        layers: list[nn.Module] = []
        last = feature_dim
        for size in (net_arch or [128, 128]):
            layers += [nn.Linear(last, size), activation()]
            last = size
        layers.append(nn.Linear(last, 1))
        self.mlp = nn.Sequential(*layers)

    def forward(self, features: th.Tensor) -> th.Tensor:
        return self.mlp(features)


def _gae(
    rewards: np.ndarray,
    values: np.ndarray,
    dones: np.ndarray,
    last_value: np.ndarray,
    gamma: float,
    lam: float,
):
    """Generalised Advantage Estimation over a ``(T, N)`` rollout."""
    n_steps, n_envs = rewards.shape
    advantages = np.zeros((n_steps, n_envs), dtype=np.float32)
    last_gae = np.zeros(n_envs, dtype=np.float32)
    for t in reversed(range(n_steps)):
        next_values = last_value if t == n_steps - 1 else values[t + 1]
        next_non_terminal = 1.0 - dones[t].astype(np.float32)
        delta = rewards[t] + gamma * next_values * next_non_terminal - values[t]
        last_gae = delta + gamma * lam * next_non_terminal * last_gae
        advantages[t] = last_gae
    return advantages + values, advantages


class LagrangianPPO(PPO):
    """PPO with a cost critic and dual ascent on the constraint multiplier."""

    def __init__(
        self,
        *args: Any,
        cost_limit: float = 2.0,
        lambda_lr: float = 0.06,
        lambda_max: float = 25.0,
        lambda_init: float = 0.0,
        constrained: bool = True,
        cost_net_arch: list[int] | None = None,
        cost_window: int = 20,
        **kwargs: Any,
    ):
        self.constrained = bool(constrained)
        self.cost_limit = float(cost_limit)
        self.lambda_lr = float(lambda_lr)
        self.lambda_max = float(lambda_max)
        self.lambda_init = float(lambda_init)
        self._cost_net_arch = cost_net_arch or [128, 128]
        self._cost_window = int(cost_window)
        super().__init__(*args, **kwargs)

        self.lagrange = LagrangeMultiplier(
            cost_limit=self.cost_limit, lr=self.lambda_lr, lambda_max=self.lambda_max
        )
        self.lagrange.lam = self.lambda_init

        self._ep_cost_window: deque = deque(maxlen=self._cost_window)
        self.last_episode_cost = 0.0
        self._roll = None  # current rollout storage

        # Metrics exposed for the training-log callback.  SB3's
        # ``logger.dump()`` *clears* ``name_to_value`` at the end of every
        # logging interval, so a callback that reads the logger mid-rollout
        # would always see an empty dict.  Stashing the numbers here makes the
        # terminal trace independent of logger internals.
        self.last_metrics: dict[str, float] = {}

    # ------------------------------------------------------------------
    # model / optimiser plumbing
    # ------------------------------------------------------------------
    def _setup_model(self) -> None:
        super()._setup_model()
        policy = self.policy
        if not hasattr(policy, "extract_features") or not hasattr(policy, "mlp_extractor"):
            raise NotImplementedError(
                "LagrangianPPO expects an ActorCritic-style policy (e.g. MlpPolicy)."
            )
        obs_shape = tuple(self.observation_space.shape)
        dummy = th.zeros((1,) + obs_shape, device=self.device)
        with th.no_grad():
            latent_pi, _ = policy.mlp_extractor(policy.extract_features(dummy))
        self.cost_net = CostValueNetwork(
            int(latent_pi.shape[1]), self._cost_net_arch
        ).to(self.device)
        self.cost_net_optimizer = th.optim.Adam(self.cost_net.parameters(), lr=self.learning_rate)

    def _excluded_save_params(self):
        return list(super()._excluded_save_params()) + ["cost_net", "cost_net_optimizer"]

    def _cost_values(self, obs: np.ndarray) -> np.ndarray:
        """Cost-critic prediction for a batch of observations."""
        obs_t = obs_as_tensor(np.asarray(obs), self.device)
        with th.no_grad():
            latent_pi, _ = self.policy.mlp_extractor(self.policy.extract_features(obs_t))
            return self.cost_net(latent_pi).cpu().numpy().reshape(-1)

    # ------------------------------------------------------------------
    # rollout collection
    # ------------------------------------------------------------------
    def collect_rollouts(  # type: ignore[override]
        self,
        env: VecEnv,
        callback,
        rollout_buffer,
        n_rollout_steps: int,
    ) -> bool:
        assert self._last_obs is not None, "collect_rollouts called before reset()"
        callback.on_rollout_start()
        self.policy.set_training_mode(False)
        if self.use_sde:
            self.policy.reset_noise(env.num_envs)

        n_envs = env.num_envs
        obs_buf = np.zeros((n_rollout_steps, n_envs) + self.observation_space.shape, dtype=np.float32)
        act_buf = np.zeros((n_rollout_steps, n_envs) + self.action_space.shape, dtype=np.float32)
        logp_buf = np.zeros((n_rollout_steps, n_envs), dtype=np.float32)
        val_buf = np.zeros((n_rollout_steps, n_envs), dtype=np.float32)
        rew_buf = np.zeros((n_rollout_steps, n_envs), dtype=np.float32)
        cost_buf = np.zeros((n_rollout_steps, n_envs), dtype=np.float32)
        done_buf = np.zeros((n_rollout_steps, n_envs), dtype=np.float32)
        ep_start_buf = np.zeros((n_rollout_steps, n_envs), dtype=np.float32)

        ep_starts = np.array(self._last_episode_starts, dtype=np.float32)
        n_steps = 0
        while n_steps < n_rollout_steps:
            with th.no_grad():
                obs_t = obs_as_tensor(self._last_obs, self.device)
                actions, values, log_probs = self.policy(obs_t)
            actions_np = actions.cpu().numpy().astype(np.float32)
            if isinstance(self.action_space, spaces.Box):
                actions_np = np.clip(actions_np, self.action_space.low, self.action_space.high)

            new_obs, rewards, dones, infos = env.step(actions_np)

            costs = np.array([float(i.get("cost", 0.0)) for i in infos], dtype=np.float32)

            self.num_timesteps += n_envs
            # SB3 2.x: this single call folds `episode` info into
            # ep_info_buffer (feeding rollout/ep_rew_mean, ep_len_mean) and
            # `is_success` into ep_success_buffer.
            self._update_info_buffer(infos, dones)

            # Collect our own constraint cost per finished episode.  The
            # windows must be filled *before* on_step() runs so a callback
            # reading the multiplier sees the value for this rollout.
            for idx, done in enumerate(dones):
                if done:
                    extra = infos[idx].get("episode_extra")
                    if extra is not None:
                        ec = float(extra.get("episode_cost", 0.0))
                        self._ep_cost_window.append(ec)
                        self.last_episode_cost = ec
                    else:
                        # Some vec-env wrappers move the sub-dicts around; fall
                        # back to whatever the env reported directly.
                        ep = infos[idx].get("episode") or {}
                        if "episode_cost" in infos[idx]:
                            self._ep_cost_window.append(float(infos[idx]["episode_cost"]))

            obs_buf[n_steps] = self._last_obs
            act_buf[n_steps] = actions_np
            logp_buf[n_steps] = log_probs.cpu().numpy().reshape(-1)
            val_buf[n_steps] = values.cpu().numpy().reshape(-1)
            rew_buf[n_steps] = rewards
            cost_buf[n_steps] = costs
            done_buf[n_steps] = dones
            ep_start_buf[n_steps] = ep_starts

            self._last_obs = new_obs
            self._last_episode_starts = dones
            ep_starts = np.array(dones, dtype=np.float32)
            n_steps += 1

            # Dispatch the callback exactly like SB3 does.  Without this the
            # training-log and evaluation callbacks never fire (and SB3's
            # `learn` loop would never see an early stop request).
            self._last_obs = new_obs
            callback.update_locals(locals())
            if not callback.on_step():
                break

        with th.no_grad():
            obs_t = obs_as_tensor(self._last_obs, self.device)
            last_values = self.policy.predict_values(obs_t).cpu().numpy().reshape(-1)
        last_cost_values = self._cost_values(self._last_obs)

        ret, adv = _gae(rew_buf, val_buf, done_buf, last_values, self.gamma, self.gae_lambda)
        cost_ret, cost_adv = _gae(
            cost_buf, self._cost_values(obs_buf.reshape((-1,) + self.observation_space.shape))
            .reshape(n_rollout_steps, n_envs),
            done_buf, last_cost_values, self.gamma, self.gae_lambda,
        )

        self._roll = {
            "obs": obs_buf.reshape((-1,) + self.observation_space.shape),
            "actions": act_buf.reshape((-1,) + self.action_space.shape),
            "log_probs": logp_buf.reshape(-1),
            "values": val_buf.reshape(-1),
            "returns": ret.reshape(-1),
            "advantages": adv.reshape(-1),
            "cost_returns": cost_ret.reshape(-1),
            "cost_advantages": cost_adv.reshape(-1),
        }

        # ---- dual ascent on the constraint multiplier --------------------
        if self.constrained and len(self._ep_cost_window) > 0:
            mean_cost = float(np.mean(self._ep_cost_window))
            self.lagrange.update(mean_cost)
            self.logger.record("constraint/mean_episode_cost", mean_cost)
        self.logger.record("constraint/lambda", float(self.lagrange.lam))

        callback.update_locals(locals())
        callback.on_rollout_end()
        return True

    # ------------------------------------------------------------------
    # PPO update (clipped surrogate + value loss + cost value loss)
    # ------------------------------------------------------------------
    def train(self) -> None:
        assert self._roll is not None, "train() called before collect_rollouts()"
        roll = self._roll
        self.policy.set_training_mode(True)
        self._update_learning_rate(self.policy.optimizer)
        self._update_learning_rate(self.cost_net_optimizer)

        clip_range = self.clip_range(self._current_progress_remaining)
        clip_range_vf = (
            self.clip_range_vf(self._current_progress_remaining)
            if self.clip_range_vf is not None else None
        )

        n_samples = roll["obs"].shape[0]
        batch_size = min(self.batch_size, n_samples)

        pg_losses, value_losses, cost_losses = [], [], []
        entropy_losses, clip_fractions, kls = [], [], []

        for _ in range(self.n_epochs):
            indices = np.random.permutation(n_samples)
            approx_kls = []
            for start in range(0, n_samples, batch_size):
                mb = indices[start:start + batch_size]
                obs = obs_as_tensor(roll["obs"][mb], self.device)
                actions = th.as_tensor(roll["actions"][mb]).to(self.device)
                old_log_prob = th.as_tensor(roll["log_probs"][mb]).to(self.device)
                old_values = th.as_tensor(roll["values"][mb]).to(self.device)
                returns = th.as_tensor(roll["returns"][mb]).to(self.device)
                advantages = th.as_tensor(roll["advantages"][mb]).to(self.device)

                latent_pi, _ = self.policy.mlp_extractor(self.policy.extract_features(obs))
                values, log_prob, entropy = self.policy.evaluate_actions(obs, actions)
                values = values.flatten()

                if self.normalize_advantage and advantages.numel() > 1:
                    advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

                ratio = th.exp(log_prob - old_log_prob)
                pg_loss = -th.min(
                    advantages * ratio,
                    advantages * th.clamp(ratio, 1 - clip_range, 1 + clip_range),
                ).mean()
                pg_losses.append(pg_loss.item())
                clip_fractions.append(th.mean((th.abs(ratio - 1) > clip_range).float()).item())

                if clip_range_vf is None:
                    values_pred = values
                else:
                    values_pred = old_values + th.clamp(values - old_values, -clip_range_vf, clip_range_vf)
                value_loss = th.nn.functional.mse_loss(returns, values_pred)
                value_losses.append(value_loss.item())

                entropy_loss = -th.mean(entropy) if entropy is not None else -th.mean(-log_prob)
                entropy_losses.append(entropy_loss.item())

                loss = pg_loss + self.ent_coef * entropy_loss + self.vf_coef * value_loss

                if self.constrained:
                    cost_pred = self.cost_net(latent_pi).flatten()
                    cost_target = th.as_tensor(roll["cost_returns"][mb]).to(self.device)
                    cost_loss = th.nn.functional.mse_loss(cost_target, cost_pred)
                    cost_losses.append(cost_loss.item())
                    loss = loss + self.vf_coef * cost_loss

                self.policy.optimizer.zero_grad()
                if self.constrained:
                    self.cost_net_optimizer.zero_grad()
                loss.backward()
                if self.max_grad_norm is not None:
                    th.nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
                    if self.constrained:
                        th.nn.utils.clip_grad_norm_(self.cost_net.parameters(), self.max_grad_norm)
                self.policy.optimizer.step()
                if self.constrained:
                    self.cost_net_optimizer.step()

                with th.no_grad():
                    log_ratio = log_prob - old_log_prob
                    approx_kls.append(
                        th.mean((th.exp(log_ratio) - 1) - log_ratio).item()
                    )

            kls.append(float(np.mean(approx_kls)))
            if self.target_kl is not None and kls[-1] > 1.5 * self.target_kl:
                break

        self._n_updates += 1
        self.logger.record("train/policy_gradient_loss", float(np.mean(pg_losses)))
        self.logger.record("train/value_loss", float(np.mean(value_losses)))
        self.logger.record("train/entropy_loss", float(np.mean(entropy_losses)))
        self.logger.record("train/approx_kl", float(np.mean(kls)))
        self.logger.record("train/clip_fraction", float(np.mean(clip_fractions)))
        self.logger.record("train/explained_variance",
                           explained_variance(roll["values"], roll["returns"]))
        self.logger.record("train/n_updates", self._n_updates)
        if cost_losses:
            self.logger.record("train/cost_value_loss", float(np.mean(cost_losses)))
        self.logger.record("constraint/lambda", float(self.lagrange.lam))

        self.last_metrics = {
            "policy_gradient_loss": float(np.mean(pg_losses)),
            "value_loss": float(np.mean(value_losses)),
            "entropy_loss": float(np.mean(entropy_losses)),
            "approx_kl": float(np.mean(kls)),
            "cost_value_loss": float(np.mean(cost_losses)) if cost_losses else float("nan"),
        }
        self._roll = None
