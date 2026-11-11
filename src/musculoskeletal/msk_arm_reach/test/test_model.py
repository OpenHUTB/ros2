"""模型测试：自由度、腱、肌肉数量、腱行程、正运动学。"""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from mujoco_musculoskeletal_arm_reach.config import MUSCLES, load_config
from mujoco_musculoskeletal_arm_reach.model import ArmModel


@pytest.fixture(scope="module")
def cfg():
    return load_config()


@pytest.fixture(scope="module")
def arm(cfg):
    return ArmModel(cfg)


def test_joint_count_and_names(arm):
    assert arm.model.nq == 2
    assert arm.model.njnt == 2
    assert arm.model.joint("shoulder").id == 0
    assert arm.model.joint("elbow").id == 1


def test_six_tendons_and_muscles(arm):
    assert arm.model.ntendon == 6
    assert arm.model.nu == 6
    for k, name in enumerate(MUSCLES):
        assert arm.model.actuator(name).id == k


def test_joint_ranges(arm):
    lo, hi = arm.model.jnt_range[arm.sh.id]
    assert lo == pytest.approx(-1.25) and hi == pytest.approx(1.45)
    lo, hi = arm.model.jnt_range[arm.el.id]
    assert lo == pytest.approx(-0.10) and hi == pytest.approx(2.30)


def test_tendon_lengths_cover_full_joint_range(arm):
    """在整个关节行程内，固定腱长度都要落在各肌肉的 lengthrange 内。"""
    for q1 in np.linspace(-1.25, 1.45, 7):
        for q2 in np.linspace(-0.10, 2.30, 7):
            arm.reset(q1, q2)
            lengths = arm.data.ten_length
            for k in range(len(MUSCLES)):
                lo, hi = arm.model.actuator_lengthrange[k]
                assert lo - 1e-6 <= lengths[k] <= hi + 1e-6, (q1, q2, k)


def test_biarticular_tendon_spans_two_joints(arm):
    arm.reset(0.0, 0.0)
    base = arm.data.ten_length.copy()
    arm.data.qpos[arm.sh.qposadr[0]] = 0.2
    mujoco.mj_forward(arm.model, arm.data)
    assert abs(arm.data.ten_length[4] - base[4]) > 1e-6
    arm.reset(0.0, 0.0)
    arm.data.qpos[arm.el.qposadr[0]] = 0.2
    mujoco.mj_forward(arm.model, arm.data)
    assert abs(arm.data.ten_length[4] - base[4]) > 1e-6


def test_forward_kinematics_matches_mujoco(cfg, arm):
    for q1, q2 in ((0.0, 0.0), (0.4, 0.8), (-0.6, 1.5)):
        arm.reset(q1, q2)
        x, z = ArmModel.forward_kinematics(q1, q2,
                                           cfg.model.upper_arm, cfg.model.forearm)
        tip = arm.tip_position
        assert tip[0] == pytest.approx(x, abs=2e-3)
        assert tip[1] == pytest.approx(z, abs=2e-3)


def test_straight_arm_reaches_full_length(cfg, arm):
    arm.reset(0.0, 0.0)
    assert np.linalg.norm(arm.tip_position) == pytest.approx(
        cfg.model.upper_arm + cfg.model.forearm, abs=1e-3)


def test_muscle_activation_produces_torque(cfg, arm):
    arm.reset(0.0, 0.6)
    arm.set_activations(np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    arm.step(20)
    assert arm.joint_torques[0] != pytest.approx(0.0, abs=1e-6)
