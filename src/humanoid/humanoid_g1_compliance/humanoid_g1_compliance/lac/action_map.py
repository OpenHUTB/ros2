# flake8: noqa  (vendored from lac-code, MIT; 保持上游原样，不参与本仓库 lint)
"""Action -> joint target translation + SDK packing for LAC.

The policy outputs a 23-D action vector in JOINT_NAMES_23DOF order. To turn
that into a low-level joint position command:

  1. q_target_23 = action_23 * ACTION_SCALE_23 + DEFAULT_JOINT_POS_23
     (per-joint affine in JOINT_NAMES_23DOF order — matches
     ``JointPositionActionCfg(use_default_offset=True, preserve_order=True)``).

  2. Optional safety: clamp q_target to soft joint limits.

  3. Pack into the SDK 35-slot ``MotorCmd`` array using ``JOINT_23_TO_SDK``.
     Slots 13/14/20/21 (waist roll/pitch + left wrist pitch/yaw for 29DOF SDK
     truncated to 23DOF) and 27..34 (anything past the SDK end-of-arm) stay at
     whatever the caller initialized them to — typically 0 with kp=kd=0.

This module is **stateless** — no buffers, no torch device. Callers (inference
node) own state and devices.
"""

from __future__ import annotations

import torch

from .constants import (
    ACTION_SCALE_23,
    DEFAULT_JOINT_POS_23,
    JOINT_23_TO_SDK,
    NUM_JOINTS,
)


def make_scale_offset_tensors(device: str | torch.device = "cuda:0"):
    """Build (scale, offset) tensors of shape (23,) on the given device.

    Cached creation is the caller's responsibility — typically build once in
    the inference node ``__init__`` and reuse every step.
    """
    scale = torch.tensor(ACTION_SCALE_23, dtype=torch.float32, device=device)
    offset = torch.tensor(DEFAULT_JOINT_POS_23, dtype=torch.float32, device=device)
    return scale, offset


def action_to_q_target(
    action_23: torch.Tensor,
    scale_23: torch.Tensor,
    default_joint_pos_23: torch.Tensor,
) -> torch.Tensor:
    """q_target = action * scale + default (all in JOINT_NAMES_23DOF order).

    Args:
      action_23: (23,) or (1, 23) raw policy output.
      scale_23: (23,) per-joint scale tensor.
      default_joint_pos_23: (23,) per-joint offset tensor.

    Returns:
      q_target: same shape as ``action_23``, all in JOINT_NAMES_23DOF order.
    """
    assert action_23.shape[-1] == NUM_JOINTS, f"action last-dim {action_23.shape[-1]} != {NUM_JOINTS}"
    return action_23 * scale_23 + default_joint_pos_23


def pack_q_target_to_sdk(q_target_23: torch.Tensor) -> dict[int, float]:
    """Return ``{sdk_slot: q_value}`` for the 23 live SDK slots.

    Args:
      q_target_23: (23,) tensor in JOINT_NAMES_23DOF order. CPU or GPU, will be
        materialized to python floats.

    Returns:
      Mapping from SDK slot index (0..26) to scalar q. The 4 inactive 23DOF
      slots (13, 14, 20, 21) and any slot >= 27 are NOT in the dict — the
      caller (e.g. ``unitree_relay`` / ``inference``) is responsible for
      leaving those at zero (kp=kd=0 is set elsewhere).
    """
    assert q_target_23.shape == (NUM_JOINTS,), f"expected (23,), got {tuple(q_target_23.shape)}"
    q_cpu = q_target_23.detach().cpu()
    return {sdk: float(q_cpu[i]) for i, sdk in enumerate(JOINT_23_TO_SDK)}


def apply_q_target_to_motor_cmd(motor_cmd_array, q_target_23: torch.Tensor) -> None:
    """Mutate a 35-element ``motor_cmd`` array in place with q_target values.

    Args:
      motor_cmd_array: a ``LowCmd.motor_cmd`` field (length 35) where each
        element has a ``.q`` attribute. Mutated in place.
      q_target_23: (23,) tensor in JOINT_NAMES_23DOF order.

    Only writes the 23 SDK slots that correspond to JOINT_NAMES_23DOF; leaves
    slots 13/14/20/21 and 27..34 untouched. Caller is expected to have set
    kp=kd=0 on those untouched slots.
    """
    packed = pack_q_target_to_sdk(q_target_23)
    for sdk_slot, q in packed.items():
        motor_cmd_array[sdk_slot].q = q
