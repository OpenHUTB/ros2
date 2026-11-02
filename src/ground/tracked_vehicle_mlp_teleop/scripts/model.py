#!/usr/bin/env python3
"""Shared model, normalization and controller utilities.

This module deliberately has no ROS imports so that it can be used by the
training script, offline tests and the ROS node alike.
"""
import json
import os

import numpy as np
import torch
import torch.nn as nn


FEATURE_NAMES = [
    "v_target",
    "w_target",
    "v_meas",
    "w_meas",
    "e_v",
    "e_w",
    "int_v",
    "int_w",
    "pitch",
    "roll",
    "yaw_rate",
]
INPUT_DIM = len(FEATURE_NAMES)
OUTPUT_DIM = 2
DEFAULT_HIDDEN = [64, 64]


class MLPController(nn.Module):
    """A small fully-connected policy: 11 sensor/error features -> (v, w)."""

    def __init__(self, input_dim=INPUT_DIM, hidden=None, output_dim=OUTPUT_DIM):
        super().__init__()
        hidden = list(hidden or DEFAULT_HIDDEN)
        layers = []
        prev = input_dim
        for width in hidden:
            layers.append(nn.Linear(prev, width))
            layers.append(nn.ReLU(inplace=True))
            prev = width
        layers.append(nn.Linear(prev, output_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


class NormStats:
    """Feature standardization statistics."""

    def __init__(self, mean, std):
        self.mean = np.asarray(mean, dtype=np.float32)
        self.std = np.asarray(std, dtype=np.float32)
        self.std = np.where(self.std < 1e-6, 1.0, self.std).astype(np.float32)

    @classmethod
    def from_features(cls, features):
        features = np.asarray(features, dtype=np.float32)
        return cls(features.mean(axis=0), features.std(axis=0))

    def normalize(self, features):
        arr = np.asarray(features, dtype=np.float32)
        return (arr - self.mean) / self.std

    def save(self, path, target_scale=None):
        payload = {"mean": self.mean, "std": self.std}
        if target_scale is not None:
            payload["target_scale"] = np.asarray(target_scale, dtype=np.float32)
        np.savez(path, **payload)

    @classmethod
    def load(cls, path):
        data = np.load(path)
        target_scale = None
        if "target_scale" in data:
            target_scale = data["target_scale"].astype(np.float32)
        return cls(data["mean"], data["std"]), target_scale


class PIExpert:
    """PI controller used to generate behaviour-cloning labels and as baseline."""

    def __init__(self, kp_v, ki_v, kp_w, ki_w, limit_v, limit_w,
                 max_v, max_w):
        self.kp_v = float(kp_v)
        self.ki_v = float(ki_v)
        self.kp_w = float(kp_w)
        self.ki_w = float(ki_w)
        self.limit_v = float(limit_v)
        self.limit_w = float(limit_w)
        self.max_v = float(max_v)
        self.max_w = float(max_w)
        self.reset()

    def reset(self):
        self.int_v = 0.0
        self.int_w = 0.0

    def command(self, v_target, w_target, v_meas, w_meas, dt):
        e_v = float(v_target) - float(v_meas)
        e_w = float(w_target) - float(w_meas)
        self.int_v = float(np.clip(self.int_v + e_v * dt,
                                   -self.limit_v, self.limit_v))
        self.int_w = float(np.clip(self.int_w + e_w * dt,
                                   -self.limit_w, self.limit_w))
        v_cmd = v_target + self.kp_v * e_v + self.ki_v * self.int_v
        w_cmd = w_target + self.kp_w * e_w + self.ki_w * self.int_w
        return (float(np.clip(v_cmd, -self.max_v, self.max_v)),
                float(np.clip(w_cmd, -self.max_w, self.max_w)))


class NeuralController:
    """Thin inference wrapper around the trained MLP."""

    def __init__(self, weights_path, norm_path, max_v, max_w,
                 hidden=None, device="cpu"):
        self.device = torch.device(device)
        self.max_v = float(max_v)
        self.max_w = float(max_w)

        config_path = os.path.join(os.path.dirname(weights_path), "model_config.json")
        if hidden is None and os.path.isfile(config_path):
            with open(config_path, "r", encoding="utf-8") as handle:
                hidden = json.load(handle).get("hidden", DEFAULT_HIDDEN)

        self.model = MLPController(hidden=hidden).to(self.device)
        state = torch.load(weights_path, map_location=self.device, weights_only=True)
        if isinstance(state, dict) and "state_dict" in state:
            state = state["state_dict"]
        self.model.load_state_dict(state)
        self.model.eval()

        self.norm, self.target_scale = NormStats.load(norm_path)
        if self.target_scale is None:
            self.target_scale = np.array([self.max_v, self.max_w], dtype=np.float32)

    def command(self, features):
        x = self.norm.normalize(np.asarray(features, dtype=np.float32).reshape(1, -1))
        tensor = torch.from_numpy(x).to(self.device)
        with torch.no_grad():
            out = self.model(tensor).cpu().numpy().reshape(-1)
        out = np.clip(out, -1.0, 1.0)
        v_cmd = float(out[0] * self.target_scale[0])
        w_cmd = float(out[1] * self.target_scale[1])
        return (float(np.clip(v_cmd, -self.max_v, self.max_v)),
                float(np.clip(w_cmd, -self.max_w, self.max_w)))


def package_path(relative_path):
    """Resolve a path inside this package both from source and after install."""
    try:
        import rospkg  # noqa: WPS433 (optional ROS dependency)
        root = rospkg.RosPack().get_path("tracked_vehicle_mlp_teleop")
        candidate = os.path.join(root, relative_path)
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", relative_path))


def load_json(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)
