"""circle.py 的纯函数测试（不需要 ROS / turtlesim）。"""

from __future__ import annotations

import math

import pytest

from turtle_motion.circle import (
    FULL_TURN,
    TurnAccumulator,
    command_for_circle,
    wrap_to_pi,
)


def test_command_for_circle_relation():
    v, omega = command_for_circle(2.0, 2.0)
    assert v == pytest.approx(2.0)
    assert omega == pytest.approx(1.0)
    assert v / omega == pytest.approx(2.0)      # 半径 = v / omega


def test_command_scales_with_radius():
    _v, w1 = command_for_circle(1.0, 3.0)
    _v, w2 = command_for_circle(3.0, 3.0)
    assert w2 == pytest.approx(w1 / 3.0)


@pytest.mark.parametrize('radius,speed', [(0.0, 1.0), (-1.0, 1.0), (1.0, 0.0), (1.0, -2.0)])
def test_command_rejects_bad_input(radius, speed):
    with pytest.raises(ValueError):
        command_for_circle(radius, speed)


def test_wrap_to_pi():
    assert wrap_to_pi(0.0) == pytest.approx(0.0)
    assert wrap_to_pi(math.pi) == pytest.approx(math.pi)
    assert wrap_to_pi(-math.pi) == pytest.approx(math.pi)
    assert wrap_to_pi(3 * math.pi) == pytest.approx(math.pi)
    assert wrap_to_pi(2 * math.pi + 0.1) == pytest.approx(0.1)


def test_accumulator_follows_turn():
    acc = TurnAccumulator(FULL_TURN)
    for theta in [0.0, 0.5, 1.0, 1.5]:
        acc.update(theta)
    assert acc.total == pytest.approx(1.5)
    assert not acc.done


def test_accumulator_handles_wrap_around():
    """从 +3.0 跳到 -3.0 实际只转了 0.283 rad，不能算成 6 rad。"""
    acc = TurnAccumulator(FULL_TURN)
    acc.update(3.0)
    acc.update(-3.0)
    assert acc.total == pytest.approx(2 * math.pi - 6.0, abs=1e-9)


def test_accumulator_finishes_after_full_turn():
    acc = TurnAccumulator(FULL_TURN)
    for k in range(0, 361):
        acc.update(math.radians(k))
    assert acc.done
    assert acc.progress == pytest.approx(1.0)


def test_deadband_ignores_stationary_noise():
    acc = TurnAccumulator(FULL_TURN)
    acc.update(0.0)
    for k in range(2000):
        acc.update(0.0005 if k % 2 else -0.0005)
    assert acc.total == 0.0
    assert not acc.done


def test_accumulator_reset():
    acc = TurnAccumulator()
    acc.update(0.0)
    acc.update(1.0)
    acc.reset()
    assert acc.total == 0.0
    assert not acc.done


def test_accumulator_rejects_bad_target():
    with pytest.raises(ValueError):
        TurnAccumulator(0.0)
