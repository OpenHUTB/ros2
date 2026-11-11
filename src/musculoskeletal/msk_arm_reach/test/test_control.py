"""控制层测试：IK/FK 闭环、可达性拒绝、激活分配、限位。"""

from __future__ import annotations

import math

import numpy as np
import pytest

from mujoco_musculoskeletal_arm_reach.config import load_config
from mujoco_musculoskeletal_arm_reach.control import (
    ReachController,
    allocate_activations,
    inverse_kinematics,
)
from mujoco_musculoskeletal_arm_reach.model import ArmModel


@pytest.fixture(scope="module")
def cfg():
    return load_config()


def test_ik_fk_roundtrip(cfg):
    for target in ((0.38, -0.17), (0.40, -0.20), (0.25, -0.10), (0.45, -0.30)):
        ik = inverse_kinematics(*target, cfg)
        assert ik is not None, target
        x, z = ArmModel.forward_kinematics(ik[0], ik[1],
                                           cfg.model.upper_arm,
                                           cfg.model.forearm)
        assert x == pytest.approx(target[0], abs=1e-9)
        assert z == pytest.approx(target[1], abs=1e-9)


def test_unreachable_rejected(cfg):
    assert inverse_kinematics(0.9, 0.0, cfg) is None      # 超出臂展
    assert inverse_kinematics(0.0, 0.0, cfg) is None      # 小于内半径
    assert inverse_kinematics(float("nan"), 0.0, cfg) is None
    assert inverse_kinematics(0.3, float("inf"), cfg) is None


def test_ik_respects_shoulder_limit(cfg):
    # 目标在肩后上方，需要 q1 超出 [-1.25, 1.45]
    assert inverse_kinematics(-0.30, 0.20, cfg) is None


def test_agonist_selection_shoulder(cfg):
    acts = allocate_activations(tau1=5.0, tau2=0.0, cfg=cfg)
    assert acts[0] > 0.0 and acts[1] == 0.0     # shoulder_flexor
    acts = allocate_activations(tau1=-5.0, tau2=0.0, cfg=cfg)
    assert acts[0] == 0.0 and acts[1] > 0.0     # shoulder_extensor


def test_agonist_selection_elbow(cfg):
    acts = allocate_activations(tau1=0.0, tau2=3.0, cfg=cfg)
    assert acts[2] > 0.0 and acts[3] == 0.0     # elbow_flexor
    acts = allocate_activations(tau1=0.0, tau2=-3.0, cfg=cfg)
    assert acts[2] == 0.0 and acts[3] > 0.0     # elbow_extensor


def test_biarticular_shares_common_torque(cfg):
    acts = allocate_activations(tau1=5.0, tau2=5.0, cfg=cfg)
    assert acts[4] > 0.0          # biarticular_flexor
    assert acts[5] == 0.0
    acts = allocate_activations(tau1=-5.0, tau2=-5.0, cfg=cfg)
    assert acts[5] > 0.0


def test_activations_bounded(cfg):
    for tau1 in (-100.0, -5.0, 0.0, 5.0, 100.0):
        for tau2 in (-100.0, -5.0, 0.0, 5.0, 100.0):
            acts = allocate_activations(tau1, tau2, cfg)
            assert np.all(acts >= 0.0) and np.all(acts <= 1.0)


def test_controller_keeps_target_when_unreachable(cfg):
    arm = ArmModel(cfg)
    arm.reset(0.0, 0.6)
    ctl = ReachController(cfg)
    q_before = (arm.q1, arm.q2)
    q_des = ctl.joint_target((2.0, 0.0), (arm.q1, arm.q2))
    assert q_des == pytest.approx(q_before)


def test_controller_integral_is_limited(cfg):
    arm = ArmModel(cfg)
    arm.reset(0.0, 0.6)
    ctl = ReachController(cfg)
    for _ in range(500):
        ctl.step(arm, (0.38, -0.17))
        arm.step(cfg.sim.steps_per_control)
    assert np.all(np.abs(ctl.integral) <= ctl.limit + 1e-12)
