# flake8: noqa  (vendored from lac-code, MIT; 保持上游原样，不参与本仓库 lint)
"""Load the LAC student-path policy from a checkpoint.

For DEPLOYMENT only the student actor path is exercised —
``adapt_module(policy_hist) -> z -> student_actor(cat(policy_obs, z))`` via
``act_inference(..., phase='finetune')``. The other submodules (teacher,
critic, priv normalizers) are NOT on the deploy compute path but are present
in the checkpoint, so the strict load needs the right shapes:

  * ``priv_obs`` normalizer (440-d): teacher-only privileged group.
  * ``priv_critic_obs_normalizer`` (5-d): critic-only group; sized from the
    ckpt, never read by the actor path.
``make_act_fn`` returns ``act(policy_obs, policy_hist) -> action``.

Obs dims are inferred from the ckpt normalizer / adapt_module buffers:
  num_policy_obs        = policy_obs_normalizer._mean.shape[-1]          # 950
  num_priv_obs          = priv_obs_normalizer._mean.shape[-1]            # 440
  num_priv_critic_obs   = priv_critic_obs_normalizer._mean.shape[-1]     # 5 (0 if absent)
  num_policy_hist_obs   = adapt_module.0.weight.shape[-1]                # 6080 (= 64*95)

``policy_frame_dim`` is read from the policy cfg yaml (95 for the released model).
"""

from __future__ import annotations

import os
import torch
import yaml

from .actor_critic import ActorCriticTwoPhase


def _infer_obs_dims(ckpt: dict) -> tuple[int, int, int, int]:
    ac = ckpt["actor_critic"]
    if "policy_obs_normalizer._mean" not in ac:
        raise KeyError(
            "Checkpoint missing policy_obs_normalizer._mean — was the model trained "
            "with obs_normalization=True?"
        )
    num_policy_obs = int(ac["policy_obs_normalizer._mean"].shape[-1])
    num_priv_obs = int(ac["priv_obs_normalizer._mean"].shape[-1])
    # Critic-only priv_critic group. The normalizer key is present only when
    # the group exists (num_priv_critic_obs>0).
    num_priv_critic_obs = (
        int(ac["priv_critic_obs_normalizer._mean"].shape[-1])
        if "priv_critic_obs_normalizer._mean" in ac else 0
    )
    # mlp_hist adapter: first Linear input dim = subsampled policy_hist dim (64*95).
    num_policy_hist_obs = int(ac["adapt_module.0.weight"].shape[-1])
    return num_policy_obs, num_priv_obs, num_priv_critic_obs, num_policy_hist_obs


def load_policy(
    ckpt_path: str,
    train_cfg_path: str,
    device: str | torch.device = "cuda:0",
) -> tuple[ActorCriticTwoPhase, dict]:
    """Instantiate ActorCriticTwoPhase + load checkpoint (strict)."""
    # Historical training cfg yamls may contain !!python/tuple tags, which
    # yaml.safe_load rejects. The cfg is a trusted local artifact produced by
    # the training runner, so UnsafeLoader is fine here.
    with open(train_cfg_path, "r") as f:
        train_cfg = yaml.load(f, Loader=yaml.UnsafeLoader)
    policy_cfg = train_cfg["policy"]

    ckpt = torch.load(ckpt_path, map_location=device, weights_only=False)
    (
        num_policy_obs,
        num_priv_obs,
        num_priv_critic_obs,
        num_policy_hist_obs,
    ) = _infer_obs_dims(ckpt)

    adapt_type = policy_cfg.get("adapt_type", "mlp_hist")
    if adapt_type != "mlp_hist":
        raise RuntimeError(
            f"policy_loader expects adapt_type='mlp_hist', got {adapt_type!r}."
        )

    model = ActorCriticTwoPhase(
        num_policy_obs=num_policy_obs,
        num_priv_obs=num_priv_obs,
        num_priv_critic_obs=num_priv_critic_obs,
        action_dim=policy_cfg.get("action_dim", 23),
        actor_hidden_dims=policy_cfg.get("actor_hidden_dims"),
        critic_hidden_dims=policy_cfg.get("critic_hidden_dims"),
        encoder_hidden_dims=policy_cfg.get("encoder_hidden_dims"),
        adapt_hidden_dims=policy_cfg.get("adapt_hidden_dims"),
        latent_dim=policy_cfg.get("latent_dim", 64),
        init_noise_std=policy_cfg.get("init_noise_std", 1.0),
        obs_normalization=policy_cfg.get("obs_normalization", True),
        adapt_type=adapt_type,
        adapt_seq_len=policy_cfg.get("adapt_seq_len", 64),
        num_policy_hist_obs=num_policy_hist_obs,
        policy_frame_dim=policy_cfg.get("policy_frame_dim", 96),
    ).to(device)

    model.load_state_dict(ckpt["actor_critic"], strict=True)
    model.eval()
    for p in model.parameters():
        p.requires_grad_(False)

    info = {
        "num_policy_obs": num_policy_obs,
        "num_priv_obs": num_priv_obs,
        "num_priv_critic_obs": num_priv_critic_obs,
        "num_policy_hist_obs": num_policy_hist_obs,
        "adapt_seq_len": policy_cfg.get("adapt_seq_len", 64),
        "policy_frame_dim": policy_cfg.get("policy_frame_dim", 96),
        "ckpt_path": os.path.abspath(ckpt_path),
        "ckpt_real_path": os.path.realpath(ckpt_path),
        "train_cfg_path": os.path.abspath(train_cfg_path),
        "train_cfg_real_path": os.path.realpath(train_cfg_path),
        "experiment_name": train_cfg.get("runner", {}).get("experiment_name", "?"),
    }
    return model, info


@torch.inference_mode()
def make_act_fn(model: ActorCriticTwoPhase, device: str | torch.device = "cuda:0"):
    """Return ``act(policy_obs_1x950, policy_hist_1x6080) -> action_1x23`` (student path).

    ``policy_hist`` must ALREADY be the full 64-deep contiguous window (6080-d) —
    ``obs_builder`` emits it that way, mirroring the training runner's no-op
    passthrough before ``act_inference``.
    """
    dev = torch.device(device)

    def _act(policy_obs: torch.Tensor, policy_hist: torch.Tensor) -> torch.Tensor:
        if policy_obs.device != dev:
            policy_obs = policy_obs.to(dev, non_blocking=True)
        if policy_hist.device != dev:
            policy_hist = policy_hist.to(dev, non_blocking=True)
        if policy_obs.dim() == 1:
            policy_obs = policy_obs.unsqueeze(0)
        if policy_hist.dim() == 1:
            policy_hist = policy_hist.unsqueeze(0)
        return model.act_inference(
            policy_obs, priv_obs=None, phase="finetune", policy_hist_obs=policy_hist,
        )

    return _act
