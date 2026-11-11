# flake8: noqa  (vendored from lac-code, MIT; 保持上游原样，不参与本仓库 lint)
"""Two-phase teacher-student actor-critic (23-joint humanoid).

Architecture:
  - encoder_priv : priv_obs -> latent_dim              (teacher path, privileged info)
  - adapt_module : policy history -> latent_dim        (student estimation, mlp_hist)
  - teacher_actor: cat(policy_obs, encoder(priv)) -> (mean, std)
  - student_actor: cat(policy_obs, adapt(hist))   -> (mean, std)
  - critic       : cat(policy_obs, priv_obs[, priv_critic_obs]) -> 1

Deployment uses ONLY the student inference path (``act_inference`` with
phase="finetune"): policy_obs -> normalizer -> adapter latent -> student actor
mean. The teacher/critic members must still be constructed so the released
checkpoint loads with ``strict=True``.

Conventions: LayerNorm + Mish MLPs, orthogonal init gain=0.01, empirical obs
normalization with a shared per-frame normalizer for the history window, and
the action std living INSIDE the Actor sub-module (DDP-friendly).
"""

from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal

from .normalization import EmpiricalNormalization

from .adapt_module import deinterleave_history



def _build_mlp(
    input_dim: int,
    output_dim: int,
    hidden_dims: list[int],
) -> nn.Sequential:
    """MLP with LayerNorm-before-Mish blocks, ending in a bare Linear."""
    layers: list[nn.Module] = []
    prev = input_dim
    for h in hidden_dims:
        layers.append(nn.Linear(prev, h))
        layers.append(nn.LayerNorm(h))
        layers.append(nn.Mish())
        prev = h
    layers.append(nn.Linear(prev, output_dim))
    return nn.Sequential(*layers)


class Actor(nn.Module):
    """Action-mean MLP + per-dim std Parameter bundled into one nn.Module.

    The std lives here (not on the parent ActorCriticTwoPhaseBase) so DDP
    wrapping covers it automatically and its gradient is all-reduced across
    ranks without manual intervention.

    ``forward(features) -> (mean, std)`` returns a tuple. ``std`` is
    ``self.std.expand_as(mean)`` so it has the same ``[B, action_dim]`` shape
    as ``mean``, even though the underlying Parameter is just ``[action_dim]``.
    The expand keeps gradient flow: ``backward()`` sums grads back into the
    1-D Parameter.
    """

    def __init__(
        self,
        input_dim: int,
        action_dim: int,
        hidden_dims: list[int],
        init_noise_std: float = 1.0,
    ):
        super().__init__()
        self.mlp = _build_mlp(input_dim, action_dim, hidden_dims)
        self.std = nn.Parameter(init_noise_std * torch.ones(action_dim))

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        mean = self.mlp(x)
        std = self.std.expand_as(mean)
        return mean, std


class ActorCriticTwoPhaseBase(nn.Module):
    """Teacher-student actor-critic with critic-only priv_critic group and MLP adapter."""

    RAW_OBS_CLIP = 1e5
    NORMALIZED_OBS_CLIP = 100.0

    def __init__(
        self,
        num_policy_obs: int,
        num_priv_obs: int,
        action_dim: int = 23,
        num_priv_critic_obs: int = 0,
        actor_hidden_dims: list[int] | None = None,
        critic_hidden_dims: list[int] | None = None,
        encoder_hidden_dims: list[int] | None = None,
        adapt_hidden_dims: list[int] | None = None,
        latent_dim: int = 256,
        init_noise_std: float = 1.0,
        obs_normalization: bool = False,
        # MLP-only adapter. "mlp" = flat 10-step policy_obs baseline;
        # "mlp_hist" = flat MLP over the stride-subsampled policy_hist window.
        adapt_type: str = "mlp",
        adapt_seq_len: int = 64,
        num_policy_hist_obs: int | None = None,
        policy_frame_dim: int = 96,
    ):
        super().__init__()
        actor_hidden_dims = actor_hidden_dims or [512, 512, 256]
        critic_hidden_dims = critic_hidden_dims or [512, 512, 256]
        encoder_hidden_dims = encoder_hidden_dims or [256]
        adapt_hidden_dims = adapt_hidden_dims or [512, 256]

        self.action_dim = action_dim
        self.latent_dim = latent_dim
        self.num_policy_obs = num_policy_obs
        self.num_priv_obs = num_priv_obs
        # critic-only privileged group (e.g. command_stand). 0 disables the group.
        self.num_priv_critic_obs = int(num_priv_critic_obs or 0)

        # -- privileged encoder (teacher path): priv_obs -> latent --
        self.encoder_priv = _build_mlp(num_priv_obs, latent_dim, encoder_hidden_dims)

        # -- adaptation module (student path): policy_obs/history -> latent --
        # MLP-only. "mlp_hist" = plain MLP over the stride-subsampled
        # policy_hist window (de-interleaved + per-frame-normalized [B,T,96]
        # flattened to [B, T*96]); "mlp" = original _build_mlp over the flat
        # 10-step policy_obs. Output ẑ stays [B, latent_dim] either way.
        self.adapt_type = adapt_type
        self._adapt_is_mlp_hist = (adapt_type == "mlp_hist")
        self._adapt_uses_hist = self._adapt_is_mlp_hist
        self.adapt_seq_len = adapt_seq_len
        self.policy_frame_dim = policy_frame_dim
        self.num_policy_hist_obs = num_policy_hist_obs
        if self._adapt_is_mlp_hist:
            # MLP over the flattened stride-4 history: input = adapt_seq_len * frame_dim
            # (= num_policy_hist_obs, the SUBSAMPLED dim the runner passes, e.g. 64*96=6144).
            self.adapt_module = _build_mlp(num_policy_hist_obs, latent_dim, adapt_hidden_dims)
        elif adapt_type == "mlp":
            self.adapt_module = _build_mlp(num_policy_obs, latent_dim, adapt_hidden_dims)
        else:
            raise ValueError(
                f"adapt_type must be 'mlp' or 'mlp_hist', got {adapt_type!r}"
            )

        # -- actors: cat(policy_obs, latent) -> (mean, std). std now lives INSIDE Actor. --
        actor_input_dim = num_policy_obs + latent_dim
        self.teacher_actor = Actor(actor_input_dim, action_dim, actor_hidden_dims, init_noise_std)
        self.student_actor = Actor(actor_input_dim, action_dim, actor_hidden_dims, init_noise_std)

        # -- critic: cat(policy_obs, priv_obs, priv_critic_obs) -> 1 (no redundancy) --
        # + critic-only priv_critic group (command_stand); num_priv_critic_obs=0
        # reproduces the plain cat(policy, priv) critic exactly.
        critic_input_dim = num_policy_obs + num_priv_obs + self.num_priv_critic_obs
        self.critic = _build_mlp(critic_input_dim, 1, critic_hidden_dims)


        # -- normalizers --
        self.obs_normalization = obs_normalization
        self.policy_obs_normalizer: nn.Module = (
            EmpiricalNormalization(num_policy_obs) if obs_normalization else nn.Identity()
        )
        self.priv_obs_normalizer: nn.Module = (
            EmpiricalNormalization(num_priv_obs) if obs_normalization else nn.Identity()
        )
        # normalizer for the critic-only priv_critic group. nn.Identity when the
        # group is absent (num_priv_critic_obs==0) so broadcast_parameters is a no-op.
        self.priv_critic_obs_normalizer: nn.Module = (
            EmpiricalNormalization(self.num_priv_critic_obs)
            if (obs_normalization and self.num_priv_critic_obs > 0) else nn.Identity()
        )
        # per-FRAME (96-dim) normalizer for the long policy_hist obs. Sized to the
        # SINGLE-frame dim (NOT T*96): every one of the T frames is normalized with the
        # same shared stats. nn.Identity when the adapter is the short-obs MLP (which
        # ignores policy_hist) so broadcast_parameters is a no-op.
        self.policy_hist_normalizer: nn.Module = (
            EmpiricalNormalization(policy_frame_dim)
            if (obs_normalization and self._adapt_uses_hist) else nn.Identity()
        )

        # -- orthogonal init with small gain (gain=0.01) --
        # Note: applies to nn.Linear inside Actor.mlp; Actor.std (a Parameter,
        # not a Linear) is left at the init_noise_std fill from above.
        self.apply(self._ortho_init)

    @staticmethod
    def _ortho_init(m: nn.Module):
        if isinstance(m, nn.Linear):
            nn.init.orthogonal_(m.weight, gain=0.01)
            if m.bias is not None:
                nn.init.zeros_(m.bias)

    # ------------------------------------------------------------------ #
    # Normalization helpers
    # ------------------------------------------------------------------ #

    def _normalize_policy_obs(self, obs: torch.Tensor) -> torch.Tensor:
        return self.policy_obs_normalizer(obs).clamp(
            -self.NORMALIZED_OBS_CLIP, self.NORMALIZED_OBS_CLIP,
        )

    def _normalize_priv_obs(self, obs: torch.Tensor) -> torch.Tensor:
        return self.priv_obs_normalizer(obs).clamp(
            -self.NORMALIZED_OBS_CLIP, self.NORMALIZED_OBS_CLIP,
        )

    def _normalize_priv_critic_obs(self, obs: torch.Tensor) -> torch.Tensor:
        return self.priv_critic_obs_normalizer(obs).clamp(
            -self.NORMALIZED_OBS_CLIP, self.NORMALIZED_OBS_CLIP,
        )

    def _critic_value(
        self,
        pol_norm: torch.Tensor,
        priv_norm: torch.Tensor,
        priv_critic_obs: torch.Tensor | None,
    ) -> torch.Tensor:
        """Critic forward over cat(policy, priv[, priv_critic]). Appends the
        critic-only priv_critic group when configured (num_priv_critic_obs>0). pol/priv
        are already normalized by the caller; priv_critic is normalized here."""
        parts = [pol_norm, priv_norm]
        if self.num_priv_critic_obs > 0:
            if priv_critic_obs is None:
                raise ValueError(
                    "num_priv_critic_obs>0 but priv_critic_obs is None — the critic "
                    "input would be undersized. Thread priv_critic_obs through this call."
                )
            parts.append(self._normalize_priv_critic_obs(priv_critic_obs))
        return self.critic(torch.cat(parts, dim=-1))

    # ------------------------------------------------------------------ #
    # long-history adapter helpers
    # ------------------------------------------------------------------ #

    def _policy_hist_frames(self, policy_hist_obs: torch.Tensor) -> torch.Tensor:
        """Flat term-major policy_hist ``[B, T*96]`` -> per-frame-normalized ``[B, T, 96]``.
        De-interleave (frame -1 = newest), then normalize each frame with the shared
        96-dim normalizer. See transformer_adapt_module.deinterleave_history for the layout."""
        frames = deinterleave_history(policy_hist_obs, self.adapt_seq_len)   # [B, T, 96]
        b, t, d = frames.shape
        fn = self.policy_hist_normalizer(frames.reshape(b * t, d)).clamp(
            -self.NORMALIZED_OBS_CLIP, self.NORMALIZED_OBS_CLIP,
        )
        return fn.reshape(b, t, d)

    def _adapt_latent(self, pol_norm: torch.Tensor, policy_hist_obs: torch.Tensor | None) -> torch.Tensor:
        """Produce ẑ from the adapter.
        mlp_hist -> plain MLP over the policy_hist window, flattened [B,T*96];
        mlp      -> over the (already-normalized) short policy_obs (unchanged baseline)."""
        if self._adapt_uses_hist:
            if policy_hist_obs is None:
                raise ValueError(f"adapt_type='{self.adapt_type}' requires policy_hist_obs (got None)")
            frames = self._policy_hist_frames(policy_hist_obs)              # [B, T, 96] normalized
            return self.adapt_module(frames.flatten(1))                    # mlp_hist: [B, T*96] -> [B, latent]
        return self.adapt_module(pol_norm)

    def precompute_adapt_latent(
        self,
        policy_obs: torch.Tensor,
        policy_hist_obs: torch.Tensor | None,
        chunk_size: int,
        transform_policy=None,
        transform_hist=None,
    ) -> torch.Tensor:
        """Detached ẑ for every sample, in no_grad chunks. Returns ``[N, latent_dim]``.

        The adapter is FROZEN during the entire train-phase actor loop (only the
        estimator updates it, afterwards), so its reg_loss-target output is constant
        across the actor's ``num_learning_epochs × num_mini_batches`` minibatches.
        Computing it ONCE here and indexing replaces that many redundant full-window
        transformer forwards — cheap for the old MLP, the dominant per-iteration cost
        for the transformer. Chunked to cap the no_grad activation peak (also lowers
        the train-phase memory ceiling vs. the per-minibatch 16k-sample transient).

        ``transform_policy`` / ``transform_hist``: optional symmetry mirrors applied
        PER-CHUNK (so the mirrored history — ~3 GB at full batch — never materializes
        in one piece). Used to precompute the symmetry-augmented half."""
        n = policy_obs.shape[0]
        chunk_size = max(1, int(chunk_size))
        outs = []
        with torch.no_grad():
            for s in range(0, n, chunk_size):
                e = min(s + chunk_size, n)
                pol_c = policy_obs[s:e]
                hist_c = None if policy_hist_obs is None else policy_hist_obs[s:e]
                if transform_policy is not None:
                    pol_c = transform_policy(pol_c)
                if transform_hist is not None and hist_c is not None:
                    hist_c = transform_hist(hist_c)
                pol_c = self._normalize_policy_obs(pol_c)
                outs.append(self._adapt_latent(pol_c, hist_c))
        return torch.cat(outs, dim=0)

    def update_normalization(
        self,
        policy_obs: torch.Tensor,
        priv_obs: torch.Tensor,
        policy_hist_obs: torch.Tensor | None = None,
        priv_critic_obs: torch.Tensor | None = None,
    ):
        if self.obs_normalization:
            self.policy_obs_normalizer.update(
                policy_obs.clamp(-self.RAW_OBS_CLIP, self.RAW_OBS_CLIP),
            )
            self.priv_obs_normalizer.update(
                priv_obs.clamp(-self.RAW_OBS_CLIP, self.RAW_OBS_CLIP),
            )
            # update the critic-only priv_critic normalizer.
            if self.num_priv_critic_obs > 0 and priv_critic_obs is not None:
                self.priv_critic_obs_normalizer.update(
                    priv_critic_obs.clamp(-self.RAW_OBS_CLIP, self.RAW_OBS_CLIP),
                )
            # update the per-frame policy_hist normalizer on de-interleaved frames.
            # mlp_hist consumes the same policy_hist, so update for it too.
            if self._adapt_uses_hist and policy_hist_obs is not None:
                frames = deinterleave_history(policy_hist_obs, self.adapt_seq_len)
                b, t, d = frames.shape
                self.policy_hist_normalizer.update(
                    frames.reshape(b * t, d).clamp(-self.RAW_OBS_CLIP, self.RAW_OBS_CLIP),
                )

    # ------------------------------------------------------------------ #
    # Rollout (called inside torch.inference_mode)
    # ------------------------------------------------------------------ #

    def act(
        self,
        policy_obs: torch.Tensor,
        priv_obs: torch.Tensor,
        phase: str,
        policy_hist_obs: torch.Tensor | None = None,
        priv_critic_obs: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Sample actions and values for one step.

        Returns (actions, log_prob, mean, std, value).
        """
        pol = self._normalize_policy_obs(policy_obs)
        priv = self._normalize_priv_obs(priv_obs)

        if phase == "train":
            latent = self.encoder_priv(priv)
            mean, std = self.teacher_actor(torch.cat([pol, latent], dim=-1))
        else:
            latent = self._adapt_latent(pol, policy_hist_obs)
            mean, std = self.student_actor(torch.cat([pol, latent], dim=-1))

        dist = Normal(mean, std)
        actions = dist.sample()
        log_prob = dist.log_prob(actions).sum(dim=-1, keepdim=True)

        value = self._critic_value(pol, priv, priv_critic_obs)
        return actions, log_prob, mean.detach(), std.detach(), value

    # ------------------------------------------------------------------ #
    # PPO evaluate (gradient-enabled, called during update)
    # ------------------------------------------------------------------ #

    def evaluate_actions_train(
        self,
        policy_obs: torch.Tensor,
        priv_obs: torch.Tensor,
        actions: torch.Tensor,
        policy_hist_obs: torch.Tensor | None = None,
        precomputed_adapt_latent: torch.Tensor | None = None,
        priv_critic_obs: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Teacher-phase forward for PPO mini-batch.

        Returns (log_prob, entropy, value, mu, sigma, reg_loss).

        ``sigma`` is the current per-dim std (broadcast to mu's shape). Caller
        uses it for KL computation. It's part of the return tuple now (was
        read separately from a top-level std Parameter in older layouts).

        ``reg_loss`` is per-sample MSE (shape [B, latent_dim]); caller applies
        mask/reduction. Gradient flows through encoder only (the adapt_module
        prediction is a detached target).

        ``precomputed_adapt_latent``: if given, use it as the (detached) adapter
        target instead of re-running the adapter. The adapter is frozen across the
        actor loop, so this is numerically identical to the in-line no_grad forward
        — it just avoids recomputing it every minibatch (see precompute_adapt_latent).
        """
        pol = self._normalize_policy_obs(policy_obs)
        priv = self._normalize_priv_obs(priv_obs)

        priv_feature = self.encoder_priv(priv)
        mean, std = self.teacher_actor(torch.cat([pol, priv_feature], dim=-1))
        dist = Normal(mean, std)
        log_prob = dist.log_prob(actions).sum(dim=-1, keepdim=True)
        entropy = dist.entropy().sum(dim=-1, keepdim=True)

        value = self._critic_value(pol, priv, priv_critic_obs)
        if precomputed_adapt_latent is not None:
            priv_pred = precomputed_adapt_latent          # detached (no_grad precompute)
        else:
            with torch.no_grad():
                priv_pred = self._adapt_latent(pol, policy_hist_obs)
        reg_loss = F.mse_loss(priv_pred, priv_feature, reduction="none")

        return log_prob, entropy, value, mean, std, reg_loss

    def evaluate_actions_finetune(
        self,
        policy_obs: torch.Tensor,
        priv_obs: torch.Tensor,
        actions: torch.Tensor,
        policy_hist_obs: torch.Tensor | None = None,
        priv_critic_obs: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Student-phase forward for PPO mini-batch.

        Returns (log_prob, entropy, value, mu, sigma).
        """
        pol = self._normalize_policy_obs(policy_obs)
        priv = self._normalize_priv_obs(priv_obs)

        priv_pred = self._adapt_latent(pol, policy_hist_obs)
        mean, std = self.student_actor(torch.cat([pol, priv_pred], dim=-1))
        dist = Normal(mean, std)
        log_prob = dist.log_prob(actions).sum(dim=-1, keepdim=True)
        entropy = dist.entropy().sum(dim=-1, keepdim=True)

        value = self._critic_value(pol, priv, priv_critic_obs)
        return log_prob, entropy, value, mean, std

    def evaluate(
        self,
        policy_obs: torch.Tensor,
        priv_obs: torch.Tensor,
        priv_critic_obs: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Value estimate only (used for GAE bootstrap)."""
        pol = self._normalize_policy_obs(policy_obs)
        priv = self._normalize_priv_obs(priv_obs)
        return self._critic_value(pol, priv, priv_critic_obs)
    # ------------------------------------------------------------------ #
    # Adaptation module training
    # ------------------------------------------------------------------ #

    def compute_adapt_loss(
        self,
        policy_obs: torch.Tensor,
        priv_obs: torch.Tensor,
        policy_hist_obs: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Per-sample MSE(adapt(policy_obs/history), encoder_priv(priv_obs).detach()).

        Returns shape [B, latent_dim]; caller applies mask/reduction.
        """
        pol = self._normalize_policy_obs(policy_obs)
        priv = self._normalize_priv_obs(priv_obs)
        with torch.no_grad():
            priv_feature = self.encoder_priv(priv)
        priv_pred = self._adapt_latent(pol, policy_hist_obs)
        return F.mse_loss(priv_pred, priv_feature, reduction="none")

    # ------------------------------------------------------------------ #
    # Inference
    # ------------------------------------------------------------------ #

    def act_inference(
        self,
        policy_obs: torch.Tensor,
        priv_obs: torch.Tensor | None = None,
        phase: str = "finetune",
        policy_hist_obs: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Deterministic action (mean only, no sampling)."""
        pol = self._normalize_policy_obs(policy_obs)
        # Ablation hook (inference-only, default off): zero the force latent to
        # measure how much the actor relies on it. Gated by getattr default False
        # so training and all other play paths are byte-identical. Set via
        # ``runner.alg.actor_critic._zero_latent = True`` (play --zero_latent).
        _zero = getattr(self, "_zero_latent", False)
        if phase == "train" and priv_obs is not None:
            priv = self._normalize_priv_obs(priv_obs)
            latent = self.encoder_priv(priv)
            if _zero:
                latent = torch.zeros_like(latent)
            mean, _ = self.teacher_actor(torch.cat([pol, latent], dim=-1))
            return mean
        latent = self._adapt_latent(pol, policy_hist_obs)
        if _zero:
            latent = torch.zeros_like(latent)
        mean, _ = self.student_actor(torch.cat([pol, latent], dim=-1))
        return mean

    # ------------------------------------------------------------------ #
    # Utilities
    # ------------------------------------------------------------------ #

    def hard_copy_teacher_to_student(self):
        """Copy teacher actor weights (incl. std) to student. DDP-aware.

        ``self.teacher_actor.state_dict()`` includes the ``std`` Parameter
        (a child of the ``Actor`` sub-module), so a single ``load_state_dict``
        copies the MLP weights AND the std.
        """
        from torch.nn.parallel import DistributedDataParallel as DDP

        src = self.teacher_actor.module if isinstance(self.teacher_actor, DDP) else self.teacher_actor
        dst = self.student_actor.module if isinstance(self.student_actor, DDP) else self.student_actor
        dst.load_state_dict(src.state_dict())
