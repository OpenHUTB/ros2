# flake8: noqa  (vendored from lac-code, MIT; 保持上游原样，不参与本仓库 lint)
"""Observation builder — policy (950-d) + policy_hist (6080-d).

The policy consumes 11 observation terms with a 95-dim per-frame layout
(``stiffness`` is 5-dim: ``[K_lin_la, K_lin_ra, K_lin_to, K_ang_la, K_ang_ra]``).
Two groups are emitted every step:

  * ``policy`` (950-d) = newest 10 frames, term-major oldest-first.
  * ``policy_hist`` (6080-d) = dense 64-deep CONTIGUOUS window @ 50 Hz (1.28 s),
    the student adapter's input.
  * log-transform on stiffness for BOTH groups — the single easiest footgun; a
    checkpoint fed raw (non-log) stiffness is train/deploy mismatched.
  * per-term CircularBuffer semantics: first push after reset fills all 64 slots.

**LAYOUT — policy_hist (6080-d, term-major, 64 contiguous frames):**

  offset  size  term                (per-frame dim x 64 tokens)
  ------  ----  ----
       0   192  base_ang_vel        (3 x 64)
     192   192  projected_gravity   (3 x 64)
     384   128  command_lin_vel     (2 x 64)
     512    64  command_ang_vel     (1 x 64)
     576    64  command_waist_yaw   (1 x 64)
     640    64  command_base_height (1 x 64)
     704   640  ref_upper_dof_pos   (10 x 64)
    1344  1472  joint_pos_rel       (23 x 64)
    2816  1472  joint_vel_rel       (23 x 64)
    4288  1472  last_action         (23 x 64)
    5760   320  stiffness           (5 x 64, log)

The ``policy`` group uses the same term order with H=10 newest frames.
"""

from __future__ import annotations

import torch


POLICY_HISTORY_LENGTH = 10          # actor policy group
POLICY_HIST_RAW_LEN = 64            # long-history depth
POLICY_HIST_STRIDE = 1              # contiguous, no subsample
POLICY_HIST_SEL_LEN = POLICY_HIST_RAW_LEN // POLICY_HIST_STRIDE  # = 64 (adapt_seq_len)

# Term declaration order — MUST match the training env's PolicyCfg / PolicyHistCfg
# class bodies AND POLICY_FRAME_DIMS_95 in actor_critic.py.
TERM_ORDER: tuple[str, ...] = (
    "base_ang_vel",
    "projected_gravity",
    "command_lin_vel",
    "command_ang_vel",
    "command_waist_yaw",
    "command_base_height",
    "ref_upper_dof_pos",
    "joint_pos_rel",
    "joint_vel_rel",
    "last_action",
    "stiffness",
)

TERM_DIM: dict[str, int] = {
    "base_ang_vel": 3,
    "projected_gravity": 3,
    "command_lin_vel": 2,
    "command_ang_vel": 1,
    "command_waist_yaw": 1,
    "command_base_height": 1,
    "ref_upper_dof_pos": 10,
    "joint_pos_rel": 23,
    "joint_vel_rel": 23,
    "last_action": 23,
    "stiffness": 5,  # torso-angular stiffness channel does not exist (dead in training)
}

POLICY_FRAME_DIM = sum(TERM_DIM[n] for n in TERM_ORDER)  # 95

# ---- policy (950D) term-major segment offsets (newest H=10 frames) ----
SEGMENT: dict[str, tuple[int, int]] = {}
_off = 0
for _name in TERM_ORDER:
    _sz = TERM_DIM[_name] * POLICY_HISTORY_LENGTH
    SEGMENT[_name] = (_off, _off + _sz)
    _off += _sz
TOTAL_OBS_DIM = _off  # 950
assert TOTAL_OBS_DIM == 950, f"policy obs dim {TOTAL_OBS_DIM} != 950"

# ---- policy_hist (6080D) term-major segment offsets (64 contiguous frames) ----
HIST_SEGMENT: dict[str, tuple[int, int]] = {}
_hoff = 0
for _name in TERM_ORDER:
    _sz = TERM_DIM[_name] * POLICY_HIST_SEL_LEN
    HIST_SEGMENT[_name] = (_hoff, _hoff + _sz)
    _hoff += _sz
TOTAL_HIST_DIM = _hoff  # 6080
assert TOTAL_HIST_DIM == 6080, f"policy_hist dim {TOTAL_HIST_DIM} != 6080"


def newest_slice(term_name: str) -> tuple[int, int]:
    """(start, end) in the flat 950D policy obs holding the NEWEST frame of a term."""
    seg_start, seg_end = SEGMENT[term_name]
    return (seg_end - TERM_DIM[term_name], seg_end)


class ObsBuilder:
    """Single 64-deep per-term history -> (policy 950D, policy_hist 6080D).

    ``build_and_push`` takes RAW (linear-scale) stiffness and applies ``log``
    internally, so the inference node keeps passing the same raw stiffness command
    it always has.
    """

    def __init__(
        self,
        default_joint_pos_art: torch.Tensor,  # (23,) ARTICULATION order
        device: str | torch.device = "cuda:0",
        raw_len: int = POLICY_HIST_RAW_LEN,
        policy_history_length: int = POLICY_HISTORY_LENGTH,
        stride: int = POLICY_HIST_STRIDE,
    ):
        self.device = torch.device(device)
        self.raw_len = int(raw_len)
        self.H_policy = int(policy_history_length)
        self.stride = int(stride)
        assert self.raw_len % self.stride == 0
        self.sel_len = self.raw_len // self.stride  # 64
        self.default_joint_pos_art = default_joint_pos_art.to(self.device).reshape(23)
        # Per-term buffer: dict[name -> (raw_len, term_dim) tensor or None until first push].
        self._buffers: dict[str, torch.Tensor | None] = {n: None for n in TERM_ORDER}

    def reset(self) -> None:
        """Clear buffers; next push fills all raw_len slots with the current value."""
        for n in self._buffers:
            self._buffers[n] = None

    def _push_term(self, name: str, value: torch.Tensor) -> None:
        expected_dim = TERM_DIM[name]
        assert value.shape == (expected_dim,), \
            f"term {name} expected ({expected_dim},), got {tuple(value.shape)}"
        value = value.to(self.device)
        if self._buffers[name] is None:
            # First push: replicate current across all raw_len slots (IsaacLab CircularBuffer).
            self._buffers[name] = value.unsqueeze(0).repeat(self.raw_len, 1).contiguous()
        else:
            # Roll: drop oldest [0], append newest at the back.
            self._buffers[name] = torch.cat(
                [self._buffers[name][1:], value.unsqueeze(0)], dim=0
            ).contiguous()

    def build_and_push(
        self,
        *,
        base_ang_vel: torch.Tensor,         # (3,)
        projected_gravity: torch.Tensor,    # (3,)
        cmd_lin_vel: torch.Tensor,          # (2,)
        cmd_ang_vel_yaw: torch.Tensor,      # (1,)
        cmd_waist_yaw: torch.Tensor,        # (1,)
        cmd_base_height: torch.Tensor,      # (1,)
        ref_arm_joints: torch.Tensor,       # (10,) JOINT_NAMES_23DOF[13:23] order
        dof_pos: torch.Tensor,              # (23,) ARTICULATION order
        dof_vel: torch.Tensor,              # (23,) ARTICULATION order
        last_action: torch.Tensor,          # (23,) JOINT_NAMES_23DOF order
        stiffness: torch.Tensor,            # (5,) RAW [K_lin_la, K_lin_ra, K_lin_to, K_ang_la, K_ang_ra]
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Push every term; return (policy (1,950), policy_hist (1,6080))."""
        joint_pos_rel = dof_pos.to(self.device).reshape(23) - self.default_joint_pos_art
        joint_vel_rel = dof_vel.to(self.device).reshape(23)
        # log-transform stiffness, matching the training-time observation term.
        stiffness_log = torch.log(stiffness.to(self.device).reshape(5).clamp_min(1e-6))

        terms = {
            "base_ang_vel": base_ang_vel.reshape(3),
            "projected_gravity": projected_gravity.reshape(3),
            "command_lin_vel": cmd_lin_vel.reshape(2),
            "command_ang_vel": cmd_ang_vel_yaw.reshape(1),
            "command_waist_yaw": cmd_waist_yaw.reshape(1),
            "command_base_height": cmd_base_height.reshape(1),
            "ref_upper_dof_pos": ref_arm_joints.reshape(10),
            "joint_pos_rel": joint_pos_rel,
            "joint_vel_rel": joint_vel_rel,
            "last_action": last_action.reshape(23),
            "stiffness": stiffness_log,
        }
        for name in TERM_ORDER:
            self._push_term(name, terms[name])

        def _emit(name, count, subsample):
            buf = self._buffers[name]
            if subsample:  # policy_hist: stride-1 {0::1} = full 64-deep buffer, newest aligned
                return buf[self.stride - 1::self.stride].reshape(-1)
            return buf[-count:].reshape(-1)  # policy: newest `count` frames, oldest-first

        # policy (950): newest H_policy frames, term-major oldest-first.
        policy_obs = torch.cat(
            [_emit(name, self.H_policy, subsample=False) for name in TERM_ORDER]
        ).unsqueeze(0)  # (1, 950)

        # policy_hist (6080): contiguous 64-deep window, term-major — matches the runner's
        # stride=1 no-op passthrough before act_inference.
        policy_hist = torch.cat(
            [_emit(name, self.sel_len, subsample=True) for name in TERM_ORDER]
        ).unsqueeze(0)  # (1, 6080)

        return policy_obs, policy_hist

    # ---- Debug accessors ----

    def newest_term(self, name: str) -> torch.Tensor | None:
        buf = self._buffers[name]
        return None if buf is None else buf[-1].clone()

    def buffer_view(self, name: str) -> torch.Tensor | None:
        buf = self._buffers[name]
        return None if buf is None else buf.clone()
