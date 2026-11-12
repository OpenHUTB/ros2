"""运动学、传感器胶水与脱困状态机的单元测试（不需要 ROS）。"""
from __future__ import annotations

import math

import pytest

from ros2_stuck_recovery.detect import CAUSE_BLOCKED, CAUSE_NONE, CAUSE_SLIP, Verdict
from ros2_stuck_recovery.recovery import (
    STATE_ABORT,
    STATE_BACK_OFF,
    STATE_CRUISE,
    STATE_PROBE,
    STATE_PULSE,
    STATE_STALE,
    STATE_TURN,
    RecoveryConfig,
    RecoverySupervisor,
)
from ros2_stuck_recovery.robot import (
    Limits,
    Twist,
    body_forward_delta,
    clamp_twist,
    integrate_pose,
    limit_accel,
    wheel_speeds,
    wheels_to_twist,
    wrap_angle,
)
from ros2_stuck_recovery.sensors import front_range

LIMITS = Limits()


# ---------------------------------------------------------------- 运动学
def test_wrap_angle_bounds():
    assert abs(wrap_angle(3 * math.pi)) == pytest.approx(math.pi, abs=1e-9)
    assert abs(wrap_angle(-3 * math.pi)) == pytest.approx(math.pi, abs=1e-9)
    assert wrap_angle(0.5) == pytest.approx(0.5)
    assert wrap_angle(0.5 + 2 * math.pi) == pytest.approx(0.5, abs=1e-9)


def test_clamp_twist_is_asymmetric():
    cmd = clamp_twist(Twist(1.0, 5.0), LIMITS)
    assert cmd.v == pytest.approx(LIMITS.v_max)
    assert cmd.w == pytest.approx(LIMITS.w_max)
    assert clamp_twist(Twist(-1.0, 0.0), LIMITS).v == pytest.approx(-LIMITS.v_back_max)


def test_integrate_straight_line():
    pose = integrate_pose((0.0, 0.0, 0.0), Twist(0.5, 0.0), 2.0)
    assert pose[0] == pytest.approx(1.0)
    assert pose[1] == pytest.approx(0.0, abs=1e-12)
    assert pose[2] == pytest.approx(0.0)


def test_integrate_full_circle_returns_home():
    """整圆之后应回到起点：检查圆弧积分不会累积位置漂移。"""
    radius, speed = 1.0, 0.5
    omega = speed / radius
    dt = 0.001
    pose = (0.0, 0.0, 0.0)
    for _ in range(int(2 * math.pi / omega / dt)):
        pose = integrate_pose(pose, Twist(speed, omega), dt)
    assert pose[0] == pytest.approx(0.0, abs=1e-3)
    assert pose[1] == pytest.approx(0.0, abs=1e-3)
    assert pose[2] == pytest.approx(0.0, abs=1e-3)


def test_pure_rotation_does_not_translate():
    pose = integrate_pose((1.0, 2.0, 0.0), Twist(0.0, 1.0), 1.0)
    assert pose[0] == pytest.approx(1.0)
    assert pose[1] == pytest.approx(2.0)
    assert pose[2] == pytest.approx(1.0)


def test_wheel_speed_roundtrip():
    cmd = Twist(0.2, 0.7)
    left, right = wheel_speeds(cmd, LIMITS.wheel_base)
    back = wheels_to_twist(left, right, LIMITS.wheel_base)
    assert back.v == pytest.approx(cmd.v)
    assert back.w == pytest.approx(cmd.w)


def test_limit_accel_bounds_change():
    out = limit_accel(Twist(0.0, 0.0), Twist(1.0, 2.0), 0.05, LIMITS)
    assert out.v == pytest.approx(LIMITS.a_v * 0.05)
    assert out.w == pytest.approx(LIMITS.a_w * 0.05)
    assert limit_accel(Twist(0.1, 0.0), Twist(1.0, 0.0), 0.0, LIMITS) == Twist(0.1, 0.0)


def test_body_forward_delta_ignores_lateral_motion():
    """原地横移（不可能，但用于验证投影方向）不计入前进位移。"""
    prev = (0.0, 0.0, math.pi / 2)
    now = (0.05, 0.0, math.pi / 2)
    assert body_forward_delta(prev, now) == pytest.approx(0.0, abs=1e-9)
    assert body_forward_delta((0, 0, 0), (0.05, 0, 0)) == pytest.approx(0.05)


# ---------------------------------------------------------------- 传感器胶水
def test_front_range_picks_min_in_sector():
    ranges = [2.0, 1.5, 0.8, 1.2, 3.0]          # 角度 -0.2 .. 0.2 rad
    value = front_range(ranges, angle_min=-0.2, angle_increment=0.1)
    assert value == pytest.approx(0.8)


def test_front_range_ignores_outside_sector():
    ranges = [0.3, 5.0]                          # 两束分别在 -0.9 / -0.4 rad，都在扇区外
    assert front_range(ranges, angle_min=-0.9, angle_increment=0.5, sector=0.2) is None


def test_front_range_skips_invalid_values():
    ranges = [float("nan"), float("inf"), 0.0, -1.0, 1.4]
    value = front_range(ranges, angle_min=-0.2, angle_increment=0.1)
    assert value == pytest.approx(1.4)


def test_front_range_all_invalid_returns_none():
    assert front_range([float("nan")] * 5, 0.0, 0.1) is None
    assert front_range([], 0.0, 0.1) is None


def test_front_range_zero_increment_returns_none():
    assert front_range([1.0, 2.0], 0.0, 0.0) is None


def test_front_range_clamps_to_max():
    assert front_range([99.0], 0.0, 0.1, max_range=10.0) == pytest.approx(10.0)


# ---------------------------------------------------------------- 脱困状态机
def drive(sup: RecoverySupervisor, t: float, pose, verdict=Verdict(), task=Twist(0.2, 0.0), odom_age=0.0, imu_gap=0.0):
    return sup.step(t, task, verdict, odom_age, pose, imu_gap)


def loop(sup: RecoverySupervisor, n: int = 400, dt: float = 0.05, verdict=Verdict(), imu_gap=0.0,
         v_scale: float = 1.0, pose=(0.0, 0.0, 0.0), t0: float = 0.0, verdict_fn=None):
    """闭环驱动：按输出指令积分位姿，返回 (指令列表, 末位姿, 末时刻)。

    ``verdict_fn(t, pose)`` 可让检测结论随时间变化（例如只在开头 2 s 卡死）。
    """
    cmds, t = [], t0
    for _ in range(n):
        v = verdict if verdict_fn is None else verdict_fn(t, pose)
        out = drive(sup, t, pose, v, imu_gap=imu_gap)
        cmds.append(out)
        pose = integrate_pose(pose, Twist(out.v * v_scale, out.w), dt)
        t += dt
    return cmds, pose, t


def test_cruise_passes_task_command_through():
    sup = RecoverySupervisor()
    out = drive(sup, 0.0, (0, 0, 0), Verdict(), Twist(0.5, 3.0))
    assert out.state == STATE_CRUISE
    assert out.override is False
    assert out.v == pytest.approx(LIMITS.v_max)   # 仍要限幅
    assert out.w == pytest.approx(LIMITS.w_max)


def test_stale_odom_stops_immediately():
    sup = RecoverySupervisor()
    out = drive(sup, 0.0, (0, 0, 0), Verdict(), Twist(0.2, 0.0), odom_age=1.0)
    assert out.state == STATE_STALE
    assert out.v == 0.0 and out.w == 0.0


def test_detection_starts_backoff():
    sup = RecoverySupervisor()
    verdict = Verdict(stuck=True, cause=CAUSE_BLOCKED, confidence=1.0)
    out = drive(sup, 0.0, (0, 0, 0), verdict)
    assert out.state == STATE_BACK_OFF
    assert out.v < 0.0 and out.override is True
    assert sup.triggers == 1


def test_full_recovery_sequence_reaches_cruise():
    sup = RecoverySupervisor()
    stuck = Verdict(stuck=True, cause=CAUSE_BLOCKED, confidence=1.0)
    cmds, _, _ = loop(sup, n=400, verdict_fn=lambda t, pose: stuck if t < 2.0 else Verdict())
    states = {c.state for c in cmds}
    assert STATE_BACK_OFF in states and STATE_TURN in states and STATE_PROBE in states
    assert sup.escapes == 1
    assert sup.state == STATE_CRUISE


def test_immobile_robot_aborts_and_stops():
    """轮子完全动不了时，模块必须有限次尝试后停车，而不是一直挣扎。"""
    sup = RecoverySupervisor()
    verdict = Verdict(stuck=True, cause=CAUSE_BLOCKED, confidence=1.0)
    cmds, _, _ = loop(sup, n=400, verdict=verdict, v_scale=0.0)
    assert sup.state == STATE_ABORT
    assert sup.escapes == 0
    tail = cmds[-5:]
    assert all(c.v == 0.0 and c.w == 0.0 for c in tail)


def test_abort_persists_until_cleared():
    sup = RecoverySupervisor()
    verdict = Verdict(stuck=True, cause=CAUSE_BLOCKED, confidence=1.0)
    loop(sup, n=400, verdict=verdict, v_scale=0.0)
    out = drive(sup, 100.0, (0, 0, 0), Verdict())
    assert out.state == STATE_ABORT and out.v == 0.0
    sup.clear_abort()
    out = drive(sup, 100.1, (0, 0, 0), Verdict(), Twist(0.1, 0.0))
    assert out.state == STATE_CRUISE and out.override is False


def test_memory_escalates_at_same_spot():
    """同一位置短时间内再次卡住，直接从更高一级开始，避免重复同一套无效动作。"""
    sup = RecoverySupervisor()
    stuck = Verdict(stuck=True, cause=CAUSE_BLOCKED, confidence=1.0)
    loop(sup, n=400, verdict_fn=lambda t, pose: stuck if t < 2.0 else Verdict())
    assert sup._memory.valid and sup._memory.level == 1
    pose = (sup._memory.x, sup._memory.y, 0.0)
    out = drive(sup, sup._memory.t + 1.0, pose, stuck)
    assert out.level == 2


def test_slip_cause_starts_pulse():
    sup = RecoverySupervisor()
    verdict = Verdict(stuck=True, cause=CAUSE_SLIP, confidence=1.0)
    out = drive(sup, 0.0, (0, 0, 0), verdict)
    assert out.state == STATE_PULSE


def test_failed_pulse_switches_action_family():
    """脉冲失败后要换成倒车，而不是在"脉冲"这条路上加码。"""
    sup = RecoverySupervisor()
    verdict = Verdict(stuck=True, cause=CAUSE_SLIP, confidence=1.0)
    loop(sup, n=300, verdict=verdict, v_scale=0.0)
    assert any("BACK_OFF" in item[1] for item in sup.history)


def test_probe_requires_imu_consistency():
    """试探动作的验收不能只看里程计：陀螺仪说车身还在乱转就不算脱困成功。"""
    sup = RecoverySupervisor()
    verdict = Verdict(stuck=True, cause=CAUSE_BLOCKED, confidence=1.0)
    loop(sup, n=400, verdict=verdict, imu_gap=1.2)
    assert sup.escapes == 0
    assert sup.state == STATE_ABORT


def test_back_off_distance_grows_with_level():
    sup = RecoverySupervisor()
    cfg = sup.cfg
    targets = [min(cfg.back_base + cfg.back_step * (level - 1), cfg.back_wall_margin + cfg.back_base)
               for level in range(1, cfg.max_level + 1)]
    assert targets == sorted(targets)
    assert targets[-1] <= cfg.back_wall_margin + cfg.back_base


def test_all_outputs_respect_limits():
    sup = RecoverySupervisor()
    verdict = Verdict(stuck=True, cause=CAUSE_BLOCKED, confidence=1.0)
    cmds, _, _ = loop(sup, n=300, verdict=verdict)
    for c in cmds:
        assert -LIMITS.v_back_max - 1e-9 <= c.v <= LIMITS.v_max + 1e-9
        assert abs(c.w) <= LIMITS.w_max + 1e-9


def test_turn_targets_are_bounded():
    sup = RecoverySupervisor(RecoveryConfig(max_level=3))
    verdict = Verdict(stuck=True, cause=CAUSE_BLOCKED, confidence=1.0)
    loop(sup, n=120, verdict=verdict, v_scale=1.0)
    for _, note in sup.history:
        if note.startswith("转向"):
            angle = float(note.split()[1])
            assert abs(angle) <= max(sup.cfg.turn_angles) + 1e-9


def test_status_dict_shape():
    sup = RecoverySupervisor()
    status = sup.status()
    for key in ("state", "cause", "level", "triggers", "escapes", "history"):
        assert key in status


def test_stale_resets_recovery_in_progress():
    sup = RecoverySupervisor()
    verdict = Verdict(stuck=True, cause=CAUSE_BLOCKED, confidence=1.0)
    drive(sup, 0.0, (0, 0, 0), verdict)
    assert sup.state == STATE_BACK_OFF
    out = drive(sup, 0.1, (0, 0, 0), Verdict(), odom_age=5.0)
    assert out.state == STATE_STALE


def test_verdict_cause_zh_mapping():
    assert Verdict(cause=CAUSE_NONE).cause_zh == "正常"
    assert Verdict(cause=CAUSE_BLOCKED).cause_zh == "被顶住"
    assert Verdict(cause=CAUSE_SLIP).cause_zh == "轮胎打滑"
