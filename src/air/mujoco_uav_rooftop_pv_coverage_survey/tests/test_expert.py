"""Expert-controller tests / 专家控制器测试。"""

import math

import pytest

from rooftop_pv.expert import ExpertConfig, ExpertInput, FlightLineExpert


def test_cross_track_and_heading_errors_have_correct_steering_sign() -> None:
    expert = FlightLineExpert()
    assert expert.command(ExpertInput(0.8, 0.0, 10.0)).yaw_rate < 0.0
    assert expert.command(ExpertInput(-0.8, 0.0, 10.0)).yaw_rate > 0.0
    assert expert.command(ExpertInput(0.0, 0.2, 10.0)).yaw_rate < 0.0
    assert expert.command(ExpertInput(0.0, -0.2, 10.0)).yaw_rate > 0.0


def test_commands_are_bounded_and_slow_near_turns() -> None:
    config = ExpertConfig(cruise_speed=2.0, min_speed=0.25, max_yaw_rate=1.2)
    expert = FlightLineExpert(config)
    saturated = expert.command(ExpertInput(100.0, math.pi, 0.0))
    cruise = expert.command(ExpertInput(0.0, 0.0, 20.0))
    assert abs(saturated.yaw_rate) <= config.max_yaw_rate
    assert config.min_speed <= saturated.forward_speed <= config.cruise_speed
    assert cruise.forward_speed == pytest.approx(config.cruise_speed)
    assert saturated.forward_speed < cruise.forward_speed


def test_invalid_expert_input_is_rejected() -> None:
    with pytest.raises(ValueError):
        FlightLineExpert().command(ExpertInput(0.0, 0.0, -0.1))
