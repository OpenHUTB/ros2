"""Dependency-free policy inference and Stable-Baselines3 weight export.

Training uses Stable-Baselines3 (PPO/SAC) exactly like the reference project
``zhaoran2000/DRL-drones``.  Deployment, however, must run on a ROS machine
that may only have NumPy, so the actor MLP is exported to a plain ``.npz`` and
re-implemented here with NumPy.  Both PPO and SAC actors in SB3 are

    obs -> [Linear + Tanh] * k -> Linear -> raw

but the final mapping differs: PPO uses a plain Gaussian and clips the mean to
the action-space bounds, while SAC uses a squashed Gaussian and applies ``tanh``.
The exported archive stores this as the ``squash`` flag.  This module reproduces
both forward passes without importing torch.

中文说明：训练端用 SB3，部署端只需要 NumPy。本文件负责（1）把 SB3 的 Actor MLP
导出成 `.npz`；（2）在 ROS 节点里用 NumPy 复现同一前向计算，避免在无人机机载
环境安装 PyTorch。

Developer: 汤湘粤 (学号: 2535030067)
"""

from __future__ import annotations

import os
from typing import Any, Dict, Optional, Sequence

import numpy as np


class NumpyActor:
    """NumPy implementation of an SB3 ``MlpPolicy`` actor."""

    def __init__(self, path: str) -> None:
        if not os.path.isfile(path):
            raise FileNotFoundError("policy weights not found: %s" % path)
        with np.load(path, allow_pickle=False) as data:
            self.n_layers = int(data["n_layers"])
            self.obs_dim = int(data["obs_dim"])
            self.action_dim = int(data["action_dim"])
            self.output_dim = int(data["output_dim"])
            self.algo = str(data["algo"]) if "algo" in data else "unknown"
            self.activation = (
                str(data["activation"]) if "activation" in data else "tanh"
            )
            self.weights = [data["layer%d_W" % i].astype(np.float64)
                            for i in range(self.n_layers)]
            self.biases = [data["layer%d_b" % i].astype(np.float64)
                           for i in range(self.n_layers)]
            # SAC uses a squashed Gaussian (tanh); PPO uses a plain Gaussian
            # whose actions are clipped to the action-space bounds.
            if "squash" in data:
                self.squash = bool(data["squash"])
            else:
                self.squash = self.output_dim == 2 * self.action_dim
        self.path = path
        self._has_log_std = self.output_dim == 2 * self.action_dim

    def predict(self,
                observation: Sequence[float],
                deterministic: bool = True,
                rng: Optional[np.random.Generator] = None) -> np.ndarray:
        """Return a ``[-1, 1]`` action for one observation."""
        vector = np.asarray(observation, dtype=np.float64).reshape(-1)
        if vector.size != self.obs_dim:
            raise ValueError(
                "policy expects %d observations, got %d" % (self.obs_dim, vector.size)
            )
        hidden = vector
        for index in range(self.n_layers - 1):
            hidden = np.tanh(self.weights[index] @ hidden + self.biases[index])
        output = self.weights[-1] @ hidden + self.biases[-1]
        mean = output[: self.action_dim]
        if deterministic or not self._has_log_std:
            return self._squash(mean)

        log_std = np.clip(output[self.action_dim:], -20.0, 2.0)
        generator = rng if rng is not None else np.random.default_rng()
        sample = mean + np.exp(log_std) * generator.standard_normal(self.action_dim)
        return self._squash(sample)

    def _squash(self, value: np.ndarray) -> np.ndarray:
        """Map the raw actor output to the ``[-1, 1]`` action box."""
        if self.squash:
            return np.tanh(value).astype(np.float32)
        return np.clip(value, -1.0, 1.0).astype(np.float32)

    def describe(self) -> Dict[str, Any]:
        """Small metadata dict for logging."""
        return {
            "path": self.path,
            "algo": self.algo,
            "obs_dim": self.obs_dim,
            "action_dim": self.action_dim,
            "hidden_layers": self.n_layers - 1,
            "squash": self.squash,
        }


def export_sb3_policy(model: Any, path: str) -> Dict[str, Any]:
    """Export the actor MLP of an SB3 PPO/SAC model to ``path``.

    Parameters
    ----------
    model:
        A trained ``stable_baselines3.PPO`` or ``SAC`` instance.
    path:
        Destination ``.npz`` file.  The parent directory is created if needed.

    Returns
    -------
    dict
        Export metadata (also stored inside the archive).
    """
    try:
        from torch import nn
    except ImportError as exc:  # pragma: no cover - training-only dependency
        raise ImportError(
            "export_sb3_policy needs PyTorch (installed with stable-baselines3)"
        ) from exc

    policy = getattr(model, "policy", None)
    if policy is None or not hasattr(policy, "mlp_extractor"):
        raise TypeError("model does not expose an SB3 MlpPolicy actor")

    layers = [module for module in policy.mlp_extractor.policy_net.modules()
              if isinstance(module, nn.Linear)]
    layers.append(policy.action_net)
    if not layers:
        raise RuntimeError("no Linear layers found in the policy")

    obs_dim = int(np.prod(policy.observation_space.shape))
    action_dim = int(np.prod(policy.action_space.shape))
    output_dim = int(layers[-1].out_features)
    if output_dim not in (action_dim, 2 * action_dim):
        raise RuntimeError(
            "unexpected actor head size %d for action_dim %d"
            % (output_dim, action_dim)
        )
    squash = type(policy.action_dist).__name__.startswith("Squashed")

    payload: Dict[str, Any] = {
        "n_layers": np.int64(len(layers)),
        "obs_dim": np.int64(obs_dim),
        "action_dim": np.int64(action_dim),
        "output_dim": np.int64(output_dim),
        "activation": np.array("tanh"),
        "algo": np.array(type(model).__name__),
        "squash": np.array(bool(squash)),
    }
    for index, layer in enumerate(layers):
        # nn.Linear.weight is (out_features, in_features); keep it so inference
        # can use the straightforward ``W @ x + b``.
        payload["layer%d_W" % index] = (
            layer.weight.detach().cpu().numpy().astype(np.float32)
        )
        payload["layer%d_b" % index] = (
            layer.bias.detach().cpu().numpy().astype(np.float32)
        )

    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    np.savez_compressed(path, **payload)
    return {
        "path": path,
        "algo": str(payload["algo"]),
        "obs_dim": obs_dim,
        "action_dim": action_dim,
        "output_dim": output_dim,
        "squash": bool(squash),
        "layer_shapes": [(int(w.shape[0]), int(w.shape[1])) for w in
                         [payload["layer%d_W" % i] for i in range(len(layers))]],
    }


def save_actor_npz(path: str,
                   weights: Sequence[np.ndarray],
                   biases: Sequence[np.ndarray],
                   obs_dim: int,
                   action_dim: int,
                   algo: str = "numpy",
                   activation: str = "tanh",
                   squash: bool = True) -> None:
    """Write a hand-built NumPy actor (used by tests and debugging).

    ``weights[i]`` must have shape ``(out_features, in_features)``, matching
    ``torch.nn.Linear.weight``; inference computes ``W @ x + b``.
    """
    if len(weights) != len(biases) or not weights:
        raise ValueError("weights and biases must be non-empty and equal length")
    output_dim = int(np.asarray(weights[-1]).shape[1])
    payload: Dict[str, Any] = {
        "n_layers": np.int64(len(weights)),
        "obs_dim": np.int64(obs_dim),
        "action_dim": np.int64(action_dim),
        "output_dim": np.int64(output_dim),
        "activation": np.array(activation),
        "algo": np.array(algo),
        "squash": np.array(bool(squash)),
    }
    for index, (weight, bias) in enumerate(zip(weights, biases)):
        payload["layer%d_W" % index] = np.asarray(weight, dtype=np.float32)
        payload["layer%d_b" % index] = np.asarray(bias, dtype=np.float32)
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    np.savez_compressed(path, **payload)


def load_actor(path: str) -> Optional[NumpyActor]:
    """Load an actor, returning ``None`` instead of raising when missing."""
    if not os.path.isfile(path):
        return None
    return NumpyActor(path)
