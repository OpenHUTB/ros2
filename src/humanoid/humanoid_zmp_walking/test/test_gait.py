"""步态规划与 LIPM 的基础单元测试。"""

import math

from humanoid_zmp_walking.footstep import foot_rect, zmp_inside_support
from humanoid_zmp_walking.gait import GaitPlan, joint_targets
from humanoid_zmp_walking.lipm import com_acceleration, omega, zmp_from_com


def test_omega_positive():
    assert omega(0.65) > 0.0


def test_zmp_equals_com_without_acceleration():
    assert zmp_from_com((0.1, -0.2), (0.0, 0.0), 0.65) == (0.1, -0.2)


def test_zmp_and_acceleration_are_inverse():
    acc = com_acceleration((0.1, 0.0), (0.0, 0.0), 0.65)
    back = zmp_from_com((0.1, 0.0), acc, 0.65)
    assert abs(back[0]) < 1e-9
    assert abs(back[1]) < 1e-9


def test_inside_support_polygon():
    rect = foot_rect(0.0, 0.0, 0.14, 0.12)
    assert zmp_inside_support((0.0, 0.0), [rect])
    assert not zmp_inside_support((0.2, 0.0), [rect])


def test_plan_step_count_and_alternation():
    plan = GaitPlan(num_steps=4)
    assert len(plan.steps) == 4
    feet = [step.foot for step in plan.steps]
    assert feet == ['left', 'right', 'left', 'right']


def test_zmp_stays_inside_support_polygon():
    plan = GaitPlan()
    assert all(plan.in_support)


def test_zmp_x_matches_com_for_uniform_motion():
    """前进方向无加速度，ZMP x 应等于质心 x。"""
    plan = GaitPlan()
    for zmp, com in zip(plan.zmp, plan.com):
        assert abs(zmp[0] - com[0]) < 1e-9


def test_knee_targets_are_non_negative():
    for index in range(101):
        targets = joint_targets(2.0 * math.pi * index / 100.0)
        assert targets[1] >= 0.0
        assert targets[4] >= 0.0


def test_hips_are_anti_phase():
    for index in range(11):
        targets = joint_targets(2.0 * math.pi * index / 10.0)
        assert abs(targets[0] + targets[3]) < 1e-9
