"""步态解析轨迹测试：周期位移、相位、摆动脚高度、支撑脚切换。"""

from __future__ import annotations

import math

import pytest

from humanoid_zmp_walk_planner.gait import (
    PHASE_SWITCH,
    body_x,
    foot_heights,
    leg_targets,
    phase,
)

STEP_LENGTH = 0.6
STEP_TIME = 1.2
STEP_HEIGHT = 0.15


def test_phase_is_normalized():
    assert phase(0.0, STEP_TIME) == 0.0
    assert phase(STEP_TIME * 0.5, STEP_TIME) == pytest.approx(0.5)
    assert phase(STEP_TIME, STEP_TIME) == pytest.approx(0.0)
    for t in (0.0, 0.3, 1.1, 5.7):
        assert 0.0 <= phase(t, STEP_TIME) < 1.0


def test_body_advances_two_steps_per_cycle():
    """step_length 是单步长度，一个周期（左右各一步）前进 2 个 step_length。"""
    for t in (0.0, 0.3, 2.0):
        assert body_x(t + STEP_TIME, STEP_LENGTH, STEP_TIME) - body_x(
            t, STEP_LENGTH, STEP_TIME) == pytest.approx(2 * STEP_LENGTH)


def test_body_advances_one_step_per_half_cycle():
    for t in (0.0, 0.4):
        assert body_x(t + STEP_TIME / 2, STEP_LENGTH, STEP_TIME) - body_x(
            t, STEP_LENGTH, STEP_TIME) == pytest.approx(STEP_LENGTH)


def test_body_speed():
    assert body_x(STEP_TIME, STEP_LENGTH, STEP_TIME) == pytest.approx(2 * STEP_LENGTH)
    assert body_x(2 * STEP_TIME, STEP_LENGTH, STEP_TIME) == pytest.approx(4 * STEP_LENGTH)
    # 速度 = 2 * step_length / step_time = 1.0 m/s
    assert body_x(STEP_TIME, STEP_LENGTH, STEP_TIME) / STEP_TIME == pytest.approx(1.0)


def test_support_alternates_by_half_cycle():
    assert foot_heights(0.1, STEP_HEIGHT)[2] == "left"
    assert foot_heights(0.4, STEP_HEIGHT)[2] == "left"
    assert foot_heights(0.6, STEP_HEIGHT)[2] == "right"
    assert foot_heights(0.9, STEP_HEIGHT)[2] == "right"


def test_support_foot_stays_on_ground():
    for p in (0.0, 0.2, 0.49):
        left, _right, support = foot_heights(p, STEP_HEIGHT)
        assert support == "left"
        assert left == 0.0
    for p in (0.5, 0.7, 0.99):
        _left, right, support = foot_heights(p, STEP_HEIGHT)
        assert support == "right"
        assert right == 0.0


def test_swing_foot_lifts_and_lands():
    """摆动脚在相位 0 / 0.5 处贴地，峰值等于 step_height。"""
    assert foot_heights(0.0, STEP_HEIGHT)[1] == pytest.approx(0.0)
    peak = foot_heights(0.25, STEP_HEIGHT)[1]
    assert peak == pytest.approx(STEP_HEIGHT, rel=1e-9)
    assert foot_heights(0.5, STEP_HEIGHT)[1] == pytest.approx(0.0, abs=1e-12)
    peak2 = foot_heights(0.75, STEP_HEIGHT)[0]
    assert peak2 == pytest.approx(STEP_HEIGHT, rel=1e-9)


def test_swing_height_never_exceeds_limit():
    for k in range(200):
        p = k / 200.0
        left, right, _ = foot_heights(p, STEP_HEIGHT)
        assert -1e-12 <= left <= STEP_HEIGHT + 1e-12
        assert -1e-12 <= right <= STEP_HEIGHT + 1e-12


def test_switch_point_is_continuous():
    """相位 0.5 两侧摆动脚都回到 0，动画不跳变。"""
    eps = 1e-9
    before = foot_heights(PHASE_SWITCH - eps, STEP_HEIGHT)
    after = foot_heights(PHASE_SWITCH + eps, STEP_HEIGHT)
    assert before[1] == pytest.approx(0.0, abs=1e-6)
    assert after[0] == pytest.approx(0.0, abs=1e-6)


def test_leg_targets_matches_parts():
    t = 2.7
    bx, left, right, p, support = leg_targets(t, STEP_LENGTH, STEP_TIME, STEP_HEIGHT)
    assert bx == pytest.approx(body_x(t, STEP_LENGTH, STEP_TIME))
    assert p == pytest.approx(phase(t, STEP_TIME))
    assert (left, right, support) == foot_heights(p, STEP_HEIGHT)


@pytest.mark.parametrize("bad", [0.0, -1.0])
def test_invalid_step_time_rejected(bad):
    with pytest.raises(ValueError):
        phase(0.0, bad)
    with pytest.raises(ValueError):
        body_x(0.0, STEP_LENGTH, bad)


@pytest.mark.parametrize("bad", [-0.1, 1.0, 2.0])
def test_invalid_phase_rejected(bad):
    with pytest.raises(ValueError):
        foot_heights(bad, STEP_HEIGHT)
