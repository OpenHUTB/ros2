# -*- coding: utf-8 -*-
"""geometry / gait / metrics / state_machine 等纯函数层的单测。

这些测试**不需要 MuJoCo**，是整个模块的快速回归网。
"""

from __future__ import annotations

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _path  # noqa: E402

# --- geometry -------------------------------------------------------
from humanoid_embodied_control import geometry as geom  # noqa: E402


class TestWrapPi:
    def test_identity_in_range(self):
        for a in (-3.0, -0.5, 0.0, 0.5, 3.0):
            assert abs(geom.wrap_pi(a) - a) < 1e-12

    def test_two_pi_is_zero(self):
        assert abs(geom.wrap_pi(2.0 * math.pi)) < 1e-9

    def test_negative_two_pi_is_zero(self):
        assert abs(geom.wrap_pi(-2.0 * math.pi)) < 1e-9

    def test_pi_stays_pi_not_minus_pi(self):
        v = geom.wrap_pi(math.pi)
        assert abs(abs(v) - math.pi) < 1e-9

    def test_over_pi_maps_to_negative(self):
        # 3π/2 → -π/2
        assert abs(geom.wrap_pi(1.5 * math.pi) + 0.5 * math.pi) < 1e-9

    def test_continuous(self):
        # 附近两个角度的 wrap 结果必须接近
        a = geom.wrap_pi(math.pi - 1e-6)
        b = geom.wrap_pi(-math.pi + 1e-6)
        assert abs(abs(a) - math.pi) < 1e-3
        assert abs(abs(b) - math.pi) < 1e-3


class TestQuatToRollPitch:
    def test_identity(self):
        r, p = geom.quat_to_roll_pitch(1.0, 0.0, 0.0, 0.0)
        assert abs(r) < 1e-12 and abs(p) < 1e-12

    def test_roll_90(self):
        # 绕 x 轴 +90°：w=cos45, x=sin45
        r, p = geom.quat_to_roll_pitch(math.cos(math.pi / 4), math.sin(math.pi / 4), 0, 0)
        assert abs(math.degrees(r) - 90.0) < 1e-6
        assert abs(p) < 1e-9

    def test_pitch_30(self):
        r, p = geom.quat_to_roll_pitch(math.cos(math.pi / 12), 0, math.sin(math.pi / 12), 0)
        assert abs(r) < 1e-9
        assert abs(math.degrees(p) - 30.0) < 1e-6

    def test_near_singularity_stays_finite(self):
        # 俯仰接近 90° 时 asin 参数会超出 [-1,1]；必须被裁剪
        r, p = geom.quat_to_roll_pitch(0.7071, 0.0, 0.70711, 0.0)
        assert -math.pi <= p <= math.pi
        assert not math.isnan(p)


class TestClampRateLimit:
    def test_clamp(self):
        assert geom.clamp(5.0, 0.0, 1.0) == 1.0
        assert geom.clamp(-5.0, 0.0, 1.0) == 0.0
        assert geom.clamp(0.5, 0.0, 1.0) == 0.5

    def test_rate_limit(self):
        assert geom.rate_limit(0.0, 1.0, 0.25) == 0.25
        assert geom.rate_limit(0.0, -1.0, 0.25) == -0.25
        assert geom.rate_limit(0.0, 0.1, 0.25) == 0.1


# --- gait -----------------------------------------------------------
from humanoid_embodied_control import gait  # noqa: E402


class TestGaitPure:
    def test_joint_order_length(self):
        assert len(gait.JOINT_ORDER) == 12
        assert len(set(gait.JOINT_ORDER)) == 12

    def test_stand_pose_is_clipped(self):
        pose = gait.stand_pose()
        for name, val in pose.items():
            lo, hi = gait.JOINT_LIMITS[name]
            assert lo - 1e-9 <= val <= hi + 1e-9, name

    def test_stand_pose_has_knee_bend(self):
        # 站姿必须屈膝：伸直时 0.05 rad 屈膝就让躯干掉半米（模型奇异点）
        pose = gait.stand_pose()
        assert pose["knee_l_pitch"] > 0.15

    def test_walk_pose_periodic(self):
        a = gait.walk_pose(0.0)
        b = gait.walk_pose(1.0)
        for k in a:
            assert abs(a[k] - b[k]) < 1e-9, k

    def test_walk_pose_legs_anti_phase(self):
        pose = gait.walk_pose(0.0)
        # phase=0：左腿后摆(+)、右腿前摆(-)
        assert pose["hip_l_pitch"] > 0
        assert pose["hip_r_pitch"] < 0

    def test_walk_pose_within_limits(self):
        for i in range(40):
            pose = gait.walk_pose(i / 40.0, stride=0.3, lift=0.3)
            for name, val in pose.items():
                lo, hi = gait.JOINT_LIMITS[name]
                assert lo - 1e-9 <= val <= hi + 1e-9, (name, val)

    def test_vector_roundtrip(self):
        pose = gait.walk_pose(0.31)
        vec = gait.to_vector(pose)
        assert len(vec) == 12
        back = gait.from_vector(vec)
        for k in pose:
            assert abs(pose[k] - back[k]) < 1e-12

    def test_blend_endpoints(self):
        a = gait.stand_pose()
        b = gait.walk_pose(0.25)
        assert gait.blend_pose(a, b, 0.0) == a
        mid = gait.blend_pose(a, b, 0.5)
        for k in a:
            assert abs(mid[k] - 0.5 * (a[k] + b[k])) < 1e-12

    def test_support_foot_switches(self):
        assert gait.support_foot(0.0) == "left"
        assert gait.support_foot(0.49) == "left"
        assert gait.support_foot(0.5) == "right"
        assert gait.support_foot(0.99) == "right"

    def test_phase_rate_not_capped_by_period(self):
        # 关键回归：0.77 cycle/s 会让本模型摔倒，phase_rate 必须允许更高步频
        r = gait.phase_rate(0.2, 0.2, 1.3)
        assert r > 1.0 / 1.3, "phase_rate 被 period 上限压回发散区了"


# --- metrics --------------------------------------------------------
from humanoid_embodied_control import metrics  # noqa: E402


class TestMetrics:
    def test_rms(self):
        assert abs(metrics.rms([3.0, 4.0]) - 3.5355339059) < 1e-6
        assert metrics.rms([0.0, 0.0]) == 0.0
        assert metrics.rms([]) == 0.0
        assert abs(metrics.rms([1.0, -1.0]) - 1.0) < 1e-12

    def test_mean(self):
        assert metrics.mean([1.0, 2.0, 3.0]) == 2.0
        assert metrics.mean([]) == 0.0

    def test_max_abs(self):
        assert metrics.max_abs([-3.0, 2.0]) == 3.0
        assert metrics.max_abs([]) == 0.0

    def test_point_to_polyline(self):
        pts = [(0.0, 0.0), (1.0, 0.0)]
        assert metrics.point_to_polyline_distance(0.5, 2.0, pts) == 2.0
        assert metrics.point_to_polyline_distance(-1.0, 0.0, pts) == 1.0
        assert metrics.point_to_polyline_distance(5.0, 0.0, pts) == 4.0

    def test_action_smoothness(self):
        assert metrics.action_smoothness([]) == 0.0
        hist = [[0.0] * 12, [0.1] * 12]
        assert abs(metrics.action_smoothness(hist) - 0.1) < 1e-12

    def test_summarize_keys(self):
        s = metrics.summarize([{"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "speed": 0.1,
                                "path_error": 3.0, "targets": [0.0] * 12},
                               {"t": 1.0, "x": 0.1, "y": 0.0, "yaw": 0.0, "speed": 0.2,
                                "path_error": 2.0, "targets": [0.1] * 12}])
        assert s["samples"] == 2
        assert s["path_rmse"] is not None
        assert s["aos"] is not None


# --- state machine --------------------------------------------------
from humanoid_embodied_control.state_machine import State, TaskStateMachine  # noqa: E402


class TestStateMachine:
    def test_avoid_clear_must_exceed_stop(self):
        try:
            TaskStateMachine(obstacle_stop=1.0, avoid_clear=0.5)
        except ValueError:
            pass
        else:
            raise AssertionError("迟滞参数未校验")

    def test_initial_is_idle_then_walk(self):
        sm = TaskStateMachine()
        sm.update(0.0, 10.0, 3.0)
        assert sm.state is State.WALK

    def test_reach_goal(self):
        sm = TaskStateMachine(goal_tolerance=0.5)
        sm.update(0.0, 10.0, 3.0)
        sm.update(0.1, 0.2, 3.0)
        assert sm.state is State.REACHED

    def test_avoid_and_recover(self):
        sm = TaskStateMachine()
        sm.update(0.0, 10.0, 3.0)
        sm.update(0.1, 9.0, 0.4)
        assert sm.state is State.AVOID
        sm.update(0.2, 9.0, 1.5)
        assert sm.state is State.WALK

    def test_fallen_wins(self):
        sm = TaskStateMachine()
        sm.update(0.0, 10.0, 3.0)
        sm.update(0.1, 9.0, 3.0, fallen=True)
        assert sm.state is State.FALLEN

    def test_speed_scale_monotone(self):
        sm = TaskStateMachine(obstacle_slow=1.2, obstacle_stop=0.65)
        vals = [sm.speed_scale(d) for d in (0.0, 0.65, 0.9, 1.2, 2.0, 5.0)]
        assert vals[0] == 0.25
        assert vals[1] == 0.25
        assert vals[4] == 1.0
        assert all(vals[i] <= vals[i + 1] + 1e-12 for i in range(len(vals) - 1))
