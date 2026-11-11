# flake8: noqa  (vendored from lac-code, MIT; 保持上游原样，不参与本仓库 lint)
"""Deployed actor-critic — 95-dim observation frame variant.

Thin subclass of :class:`ActorCriticTwoPhaseBase`: the ``policy`` /
``policy_hist`` observation frame is 95-dim (not 96) because the training env
drops the dead torso-angular stiffness channel (the stiffness term is 5-dim).
The policy_hist buffer is TERM-MAJOR, so :func:`deinterleave_history` must be
told the per-term widths to slice frames correctly; the base class uses the
module-default 96-dim ``POLICY_FRAME_DIMS``. This subclass overrides the two
methods that de-interleave the history so they pass ``POLICY_FRAME_DIMS_95``
(…, 5) instead.

Everything else (network construction, adapter, actor/critic,
normalizers) is inherited UNCHANGED — the network input widths (policy 950,
policy_hist 6080, adapter 6080->1024) auto-derive from the checkpoint, and the
per-frame normalizer is sized by the ``policy_frame_dim=95`` cfg entry.
"""

from __future__ import annotations

import torch

from .actor_critic_base import ActorCriticTwoPhaseBase
from .adapt_module import deinterleave_history

# 11 policy terms, per-frame dims, in declaration order. Stiffness term is 5-dim
# (torso-angular dropped). sum == 95. Mirrors POLICY_FRAME_DIMS in adapt_module.py
# (the 6-dim original).
POLICY_FRAME_DIMS_95: tuple[int, ...] = (3, 3, 2, 1, 1, 1, 10, 23, 23, 23, 5)


class ActorCriticTwoPhase(ActorCriticTwoPhaseBase):
    """Base actor-critic with a 95-dim policy_hist frame (torso-angular stiffness dropped)."""

    def _policy_hist_frames(self, policy_hist_obs: torch.Tensor) -> torch.Tensor:
        """Flat term-major policy_hist ``[B, T*95]`` -> per-frame-normalized ``[B, T, 95]``.
        Identical to the base except ``dims=POLICY_FRAME_DIMS_95`` (95, not 96)."""
        frames = deinterleave_history(
            policy_hist_obs, self.adapt_seq_len, dims=POLICY_FRAME_DIMS_95
        )  # [B, T, 95]
        b, t, d = frames.shape
        fn = self.policy_hist_normalizer(frames.reshape(b * t, d)).clamp(
            -self.NORMALIZED_OBS_CLIP, self.NORMALIZED_OBS_CLIP,
        )
        return fn.reshape(b, t, d)

    def update_normalization(
        self,
        policy_obs: torch.Tensor,
        priv_obs: torch.Tensor,
        policy_hist_obs: torch.Tensor | None = None,
        priv_critic_obs: torch.Tensor | None = None,
    ):
        """Identical to the base except the policy_hist de-interleave uses the 95-dim tuple."""
        if self.obs_normalization:
            self.policy_obs_normalizer.update(
                policy_obs.clamp(-self.RAW_OBS_CLIP, self.RAW_OBS_CLIP),
            )
            self.priv_obs_normalizer.update(
                priv_obs.clamp(-self.RAW_OBS_CLIP, self.RAW_OBS_CLIP),
            )
            if self.num_priv_critic_obs > 0 and priv_critic_obs is not None:
                self.priv_critic_obs_normalizer.update(
                    priv_critic_obs.clamp(-self.RAW_OBS_CLIP, self.RAW_OBS_CLIP),
                )
            if self._adapt_uses_hist and policy_hist_obs is not None:
                frames = deinterleave_history(
                    policy_hist_obs, self.adapt_seq_len, dims=POLICY_FRAME_DIMS_95
                )
                b, t, d = frames.shape
                self.policy_hist_normalizer.update(
                    frames.reshape(b * t, d).clamp(-self.RAW_OBS_CLIP, self.RAW_OBS_CLIP),
                )
