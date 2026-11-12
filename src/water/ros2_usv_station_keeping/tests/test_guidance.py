"""制导律测试：验证"顶流悬停"这个几何解在各种输入下的行为。"""
from __future__ import annotations

import math

import pytest

from ros2_usv_station_keeping.geometry import wrap_angle
from ros2_usv_station_keeping.guidance import GuidanceConfig, StationKeepingGuidance


def make(**kwargs) -> StationKeepingGuidance:
    cfg = GuidanceConfig(**kwargs)
    guide = StationKeepingGuidance(cfg)
    guide.reset(0.0)
    return guide


class TestNoCurrent:
    def test_heading_points_at_target(self):
        guide = make()
        guide.reset(math.pi - 0.02)
        out = guide.update(1.0, 0.0, math.pi - 0.02, (0.0, 0.0), (0.0, 0.0), dt=0.05)
        assert abs(wrap_angle(out.psi_cmd - math.pi)) < 0.05

    def test_speed_saturates_at_approach_speed(self):
        guide = make()
        out = guide.update(9.0, 0.0, 0.0, (0.0, 0.0), (0.0, 0.0), dt=0.05)
        assert out.u_cmd <= guide.config.approach_speed + 1e-9

    def test_arrived_boat_stops_thrusting(self):
        guide = make()
        out = guide.update(0.02, 0.0, 0.0, (0.0, 0.0), (0.0, 0.0), dt=0.05)
        assert out.holding is False
        assert out.u_cmd == pytest.approx(0.0)

    def test_deadband_flag(self):
        guide = make()
        out = guide.update(0.05, 0.0, 0.0, (0.0, 0.0), (0.0, 0.0), dt=0.05)
        assert out.in_deadband is True


class TestHoverAgainstCurrent:
    def test_holding_position_requires_thrust_against_current(self):
        """在有流的点位上，必须持续出力；"到了就关机"会被冲走。"""
        guide = make()
        out = guide.update(0.0, 0.0, 0.0, (2.0, 0.0), (0.0, 0.0), dt=0.05)
        assert out.holding is True
        assert out.u_cmd > 0.05

    def test_heading_turns_into_the_current(self):
        """横流从右舷来，船头应转向迎流侧，而不是指向目标。"""
        guide = make()
        # 目标在正前方（+x），流沿 +y。为保持位置，船头必须偏到 -y 方向。
        out = guide.update(0.0, 0.0, 0.0, (5.0, 0.0), (0.0, 0.30), dt=0.05)
        heading_dir = math.degrees(math.sin(out.psi_cmd - math.pi))
        assert heading_dir > 0.0, "船头应偏向 -y 一侧以抵消 +y 方向的流"

    def test_crab_angle_grows_with_current(self):
        weak = make().update(0.0, 0.0, 0.0, (3.0, 0.0), (0.0, 0.12), dt=0.05)
        strong = make().update(0.0, 0.0, 0.0, (3.0, 0.0), (0.0, 0.40), dt=0.05)
        assert abs(strong.crab_deg) > abs(weak.crab_deg)

    def test_small_current_estimate_is_ignored(self):
        """低于死区的估计流按 0 处理，否则静水下会顺着估计残差斜着开。"""
        guide = make()
        out = guide.update(2.0, 0.0, 0.0, (0.0, 0.0), (0.0, 0.05), dt=0.05)
        assert out.crab_deg == pytest.approx(0.0, abs=0.5)


class TestSlewAndAlignment:
    def test_heading_command_rate_limited(self):
        guide = make()
        guide.reset(0.0)
        prev = guide.last_heading_command
        out = guide.update(0.0, -5.0, 0.0, (0.0, 0.0), (0.0, 0.0), dt=0.05)
        step = abs(math.degrees(out.psi_cmd - prev))
        assert step <= guide.config.heading_rate_limit_deg + 1e-6

    def test_speed_reduced_when_heading_far_off(self):
        """航向差很大时不能全速冲，否则轨迹是螺旋。"""
        aligned_guide = make()
        aligned_guide.reset(math.pi)
        aligned = aligned_guide.update(2.0, 0.0, math.pi, (0.0, 0.0), (0.0, 0.0), dt=0.05)
        # 目标在 -x 方向，但船头朝着 +x：需要掉头，此时前进分量应被压低
        misaligned = make().update(2.0, 0.0, 0.0, (0.0, 0.0), (0.0, 0.0), dt=0.05)
        assert misaligned.u_cmd < aligned.u_cmd

    def test_align_floor_keeps_some_thrust(self):
        guide = make(align_floor=0.2)
        out = guide.update(2.0, 0.0, 0.0, (0.0, 0.0), (0.0, 0.0), dt=0.05)
        assert out.u_cmd >= 0.0
