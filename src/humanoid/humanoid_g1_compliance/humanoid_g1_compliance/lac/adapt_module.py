# flake8: noqa  (vendored from lac-code, MIT; 保持上游原样，不参与本仓库 lint)
"""History-layout helpers for the student adapter (mlp_hist).

HISTORY LAYOUT (pinned against the IsaacLab observation manager — do NOT change
without re-checking):

  IsaacLab ``ObservationManager.compute_group`` flattens each term's
  ``CircularBuffer`` and concatenates terms:

    * ``CircularBuffer.buffer`` has shape ``[N, H, d]`` with the OLDEST entry at
      index 0 and the NEWEST (current) entry at index ``H-1``.
    * ``flatten_history_dim`` -> ``buffer.reshape(N, H*d)`` row-major
      -> ``[oldest(d), step1(d), ..., newest(d)]`` (step-major WITHIN a term).
    * terms are concatenated in DECLARATION order along the last dim
      -> ``[term0_block(H*d0) | term1_block(H*d1) | ... ]`` (TERM-major across terms).

  So the flat policy-history vector is ``[B, sum_i(H*d_i)]`` and the per-frame
  layout is term-major / step-major-within-term / oldest-first.

  ``deinterleave_history`` inverts this to frame-major ``[B, T, frame_dim]``
  where frame 0 is the OLDEST and frame ``T-1`` is the NEWEST (current).

  ``POLICY_FRAME_DIMS`` lists the 11 policy terms' per-frame dims in declaration
  order (base_ang_vel, projected_gravity, command_lin_vel, command_ang_vel,
  command_waist_yaw, command_base_height, ref_upper_dof_pos, joint_pos_rel,
  joint_vel_rel, last_action, stiffness). ``last_action`` is already a term, so
  each frame token already contains the previous action. The deployed policy
  uses the 5-dim stiffness variant (95-dim frame, ``POLICY_FRAME_DIMS_95`` in
  ``actor_critic.py``); the 6-dim tuple here is the module default consumed by
  the actor-critic base class signatures.
"""

from __future__ import annotations

import torch

# 11 policy terms, per-frame dims, in declaration order. sum == 96.
POLICY_FRAME_DIMS: tuple[int, ...] = (3, 3, 2, 1, 1, 1, 10, 23, 23, 23, 6)
POLICY_FRAME_DIM: int = sum(POLICY_FRAME_DIMS)  # 96


def deinterleave_history(
    flat: torch.Tensor,
    seq_len: int,
    dims: tuple[int, ...] = POLICY_FRAME_DIMS,
) -> torch.Tensor:
    """Term-major flat ``[B, T*sum(dims)]`` -> frame-major ``[B, T, sum(dims)]``.

    frame 0 = oldest, frame -1 = newest (current). See module docstring for the
    layout proof. Inverse of :func:`interleave_history` (used by the round-trip test).
    """
    B = flat.shape[0]
    out = []
    off = 0
    for d in dims:
        block = flat[:, off : off + d * seq_len]          # [B, T*d], step-major oldest-first
        out.append(block.reshape(B, seq_len, d))          # [B, T, d]
        off += d * seq_len
    assert off == flat.shape[-1], (
        f"deinterleave_history size mismatch: consumed {off} but flat has "
        f"{flat.shape[-1]} (seq_len={seq_len}, dims sum={sum(dims)})"
    )
    return torch.cat(out, dim=-1)                          # [B, T, sum(dims)]


def interleave_history(
    frames: torch.Tensor,
    dims: tuple[int, ...] = POLICY_FRAME_DIMS,
) -> torch.Tensor:
    """Frame-major ``[B, T, sum(dims)]`` -> term-major flat ``[B, T*sum(dims)]``.

    Mimics IsaacLab's per-term ``buffer.reshape(N, H*d)`` + term concat, so that
    ``deinterleave_history(interleave_history(x)) == x``. Used only for testing.
    """
    B, T, _ = frames.shape
    blocks = []
    off = 0
    for d in dims:
        blk = frames[:, :, off : off + d]                 # [B, T, d]
        blocks.append(blk.reshape(B, T * d))              # [B, T*d] row-major (oldest-first)
        off += d
    return torch.cat(blocks, dim=-1)
