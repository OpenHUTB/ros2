"""Offline pytest suite for the tracked_vehicle_mlp_teleop package.

The tests do not require ROS or Gazebo.  They are registered with
``catkin_add_nosetests`` in CMakeLists.txt, so ``catkin_make run_tests``
executes them, and they can also be run directly with pytest:

    python3 -m pytest scripts/test_offline.py
"""
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from model import (  # noqa: E402
    FEATURE_NAMES,
    INPUT_DIM,
    MLPController,
    NeuralController,
    PIExpert,
    package_path,
)

MAX_V = 0.9
MAX_W = 1.6
TOL = 1e-6


def test_feature_layout():
    """Feature vector length must match the first network layer input."""
    assert len(FEATURE_NAMES) == INPUT_DIM
    assert INPUT_DIM == 11


def test_mlp_batch_shape():
    """The policy maps a batch of feature vectors to (v, w) pairs."""
    model = MLPController()
    out = model(torch.zeros(3, INPUT_DIM))
    assert tuple(out.shape) == (3, 2)


def test_model_artifacts_present():
    """The exported weights and normalization statistics must be in the repo."""
    weights = package_path("models/mlp_policy.pt")
    norm = package_path("models/norm_stats.npz")
    assert os.path.isfile(weights), "缺少 MLP 权重: {}".format(weights)
    assert os.path.isfile(norm), "缺少归一化文件: {}".format(norm)


def test_trained_policy_inference():
    """The shipped policy must produce finite commands inside the limits."""
    weights = package_path("models/mlp_policy.pt")
    norm = package_path("models/norm_stats.npz")
    controller = NeuralController(weights, norm, MAX_V, MAX_W)
    v_cmd, w_cmd = controller.command(np.zeros(INPUT_DIM, dtype=np.float32))
    assert np.isfinite(v_cmd) and np.isfinite(w_cmd)
    assert abs(v_cmd) <= MAX_V + TOL
    assert abs(w_cmd) <= MAX_W + TOL


def test_pi_expert_tracks_error():
    """The PI expert must react to a positive speed error."""
    expert = PIExpert(1.1, 1.6, 1.8, 3.2, 0.8, 1.0, MAX_V, MAX_W)
    v, w = expert.command(0.5, 0.0, 0.0, 0.0, 0.05)
    assert v >= 0.5
    assert abs(w) < TOL


def test_pi_expert_saturates():
    """Integral windup must be clipped to the configured command limits."""
    expert = PIExpert(1.1, 1.6, 1.8, 3.2, 0.8, 1.0, MAX_V, MAX_W)
    for _ in range(100):
        v, w = expert.command(0.5, 0.2, 0.0, 0.0, 0.05)
    assert v <= MAX_V + TOL
    assert w <= MAX_W + TOL
