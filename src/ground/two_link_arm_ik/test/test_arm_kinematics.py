import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import math

from two_link_arm_ik import config, kinematics
from two_link_arm_ik.arm import Arm


def test_fk_full_extension():
    elbow, tip = kinematics.forward_kinematics(0.0, 0.0)
    assert abs(elbow[0] - config.L1) < 1e-9
    assert abs(elbow[1]) < 1e-9
    assert abs(tip[0] - (config.L1 + config.L2)) < 1e-9
    assert abs(tip[1]) < 1e-9


def test_fk_right_angle():
    elbow, tip = kinematics.forward_kinematics(0.0, math.pi / 2)
    assert abs(elbow[0] - config.L1) < 1e-9
    assert abs(tip[0] - config.L1) < 1e-9
    assert abs(tip[1] - config.L2) < 1e-9


def test_ik_roundtrip_for_forward_solutions():
    """对若干可行关节角，先正向求末端，再逆解，末端应重合。"""
    for q1 in (-1.0, -0.3, 0.4, 1.1):
        for q2 in (-0.9, 0.35, 1.2):
            _, tip = kinematics.forward_kinematics(q1, q2)
            sol = kinematics.inverse_kinematics(tip, elbow_up=(q2 > 0))
            assert sol is not None
            _, tip2 = kinematics.forward_kinematics(*sol)
            assert math.hypot(tip[0] - tip2[0], tip[1] - tip2[1]) < 1e-6


def test_ik_unreachable_returns_none():
    assert kinematics.inverse_kinematics((400.0, 0.0)) is None
    assert kinematics.reachable((400.0, 0.0)) is False


def test_ik_on_boundary_is_reachable():
    r = config.L1 + config.L2
    assert kinematics.reachable((r, 0.0)) is True


def test_joint_limit_clipping():
    q1, q2, clipped = kinematics.clamp_joints(4.0, -4.0)
    assert clipped is True
    assert q1 == config.Q1_MAX
    assert q2 == config.Q2_MIN


def test_arm_reaches_all_targets():
    """依次给四个目标，限定步数内末端应逐个抵达。"""
    arm = Arm()
    steps = 0
    while arm.target is not None and steps < 2000:
        arm.step()
        steps += 1
    assert len(arm.reached) == len(config.TARGETS), arm.reached
    assert steps < 2000
