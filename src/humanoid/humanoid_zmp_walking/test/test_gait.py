"""步态规划与 LIPM 的基础单元测试。"""

from humanoid_zmp_walking.footstep import foot_rect, zmp_inside_support
from humanoid_zmp_walking.gait import GaitPlan
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


def test_single_support_interval_leaves_one_rect():
    """摆动期间支撑域应只剩支撑脚，双支撑期才包含两只脚。"""
    plan = GaitPlan()
    first_left = plan.steps[0]
    mid_swing = (first_left.swing_start + first_left.swing_end) / 2.0
    assert len(plan._landed_rects(mid_swing)) == 1
    assert len(plan._landed_rects(1.2)) == 2


def test_knee_targets_are_non_negative():
    plan = GaitPlan()
    for index in range(len(plan.t)):
        targets = plan.joint_targets(index * plan.dt)
        assert targets[1] >= 0.0
        assert targets[4] >= 0.0


def test_swing_window_matches_footstep_timing():
    """抬腿窗口应与摆动窗口一致，双支撑期关节角为零。"""
    plan = GaitPlan()
    first_left = plan.steps[0]
    mid_swing = (first_left.swing_start + first_left.swing_end) / 2.0
    targets = plan.joint_targets(mid_swing)
    assert targets[0] > 0.0
    assert targets[1] > 0.0
    assert abs(targets[3]) < 1e-9
    assert abs(targets[4]) < 1e-9
    assert plan.joint_targets(0.3) == [0.0] * 6
