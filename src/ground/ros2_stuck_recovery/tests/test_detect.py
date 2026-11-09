"""检测器测试：三路证据各自的判定、优先级、确认时长与降级行为。"""
from __future__ import annotations

import pytest

from ros2_stuck_recovery.detect import (
    CAUSE_BLOCKED,
    CAUSE_NONE,
    CAUSE_ONE_WHEEL,
    CAUSE_SLIP,
    CAUSE_SPIN,
    DetectorConfig,
    Sample,
    StuckDetector,
)

DT = 0.05


def feed(det: StuckDetector, samples) -> list:
    return [det.update(s) for s in samples]


def series(n=60, t0=0.0, v_cmd=0.25, v_odom=0.25, w_odom=0.0, w_imu=0.0, rng=None, dt=DT):
    out = []
    for k in range(n):
        t = t0 + k * dt
        out.append(
            Sample(
                t=t,
                v_cmd=v_cmd,
                w_cmd=0.0,
                v_odom=v_odom,
                w_odom=w_odom,
                w_imu=w_imu,
                range_front=rng[k] if rng is not None else None,
            )
        )
    return out


def causes(verdicts, start=0):
    return [v.cause for v in verdicts[start:]]


def test_normal_driving_never_alarms():
    det = StuckDetector()
    verdicts = feed(det, series(n=120))
    assert all(v.cause == CAUSE_NONE for v in verdicts)


def test_blocked_detected_after_confirm_window():
    det = StuckDetector()
    verdicts = feed(det, series(n=60, v_odom=0.0))
    assert verdicts[5].cause == CAUSE_NONE      # 0.25 s 证据还不够
    assert verdicts[-1].cause == CAUSE_BLOCKED
    assert verdicts[-1].stuck is True


def test_confirm_window_is_respected():
    det = StuckDetector(DetectorConfig(confirm=1.0))
    verdicts = feed(det, series(n=40, v_odom=0.0))
    first = next(i for i, v in enumerate(verdicts) if v.cause != CAUSE_NONE)
    assert first * DT >= 1.0


def test_spin_classified_separately_from_blocked():
    """里程计显示在原地打转（有角速度、几乎没位移）应判为"原地打转"。"""
    det = StuckDetector()
    verdicts = feed(det, series(n=80, v_odom=0.02, w_odom=0.8, w_imu=0.8))
    assert verdicts[-1].cause == CAUSE_SPIN


def test_one_wheel_from_imu_mismatch():
    """编码器说直着走，陀螺仪说在打转 -> 单轮失抓地。"""
    det = StuckDetector()
    verdicts = feed(det, series(n=60, v_odom=0.25, w_odom=0.0, w_imu=1.5))
    assert verdicts[-1].cause == CAUSE_ONE_WHEEL


def test_one_wheel_requires_both_sources():
    det = StuckDetector()
    verdicts = feed(det, series(n=60, v_odom=0.25, w_odom=0.0, w_imu=0.05))
    assert all(v.cause == CAUSE_NONE for v in verdicts)


def test_slip_detected_with_range_stall():
    """先接近、后空转：里程计继续走，前向距离不再减少 -> 打滑。"""
    det = StuckDetector()
    approach = [1.20 - 0.015 * k for k in range(20)]     # 接近 0.3 m
    stall = [0.90 + 0.002 * (k % 3) for k in range(60)]  # 抖动但不再接近
    samples = series(n=20, rng=approach) + series(n=60, t0=1.0, rng=stall)
    verdicts = feed(det, samples)
    assert verdicts[-1].cause == CAUSE_SLIP
    assert verdicts[-1].slip_ratio > 0.8               # 里程计认为自己在正常走


def test_wall_hug_does_not_alarm():
    """贴墙平行行驶：测距恒定但从未接近过，不能判成打滑。"""
    det = StuckDetector()
    flat = [0.5] * 120
    verdicts = feed(det, series(n=120, rng=flat))
    assert all(v.cause == CAUSE_NONE for v in verdicts)


def test_pure_rotation_is_not_stuck():
    det = StuckDetector()
    samples = [
        Sample(t=k * DT, v_cmd=0.0, w_cmd=0.8, v_odom=0.0, w_odom=0.8, w_imu=0.8)
        for k in range(80)
    ]
    verdicts = feed(det, samples)
    assert all(v.cause == CAUSE_NONE for v in verdicts)


def test_missing_sensors_degrade_gracefully():
    """只有里程计时仍要能抓"顶住"，不能因为缺 IMU/测距就抛异常或误报。"""
    det = StuckDetector()
    verdicts = feed(det, series(n=60, v_odom=0.0, w_imu=None))
    assert verdicts[-1].cause == CAUSE_BLOCKED


def test_acceleration_transient_does_not_alarm():
    """起步阶段指令先到、里程计滞后，不应被判成卡死。"""
    det = StuckDetector()
    samples = []
    for k in range(60):
        ramp = min(1.0, k / 8.0)
        samples.append(
            Sample(t=k * DT, v_cmd=0.25, w_cmd=0.0, v_odom=0.25 * ramp, w_odom=0.0, w_imu=0.0)
        )
    verdicts = feed(det, samples)
    assert all(v.cause == CAUSE_NONE for v in verdicts)


def test_reset_clears_state():
    det = StuckDetector()
    feed(det, series(n=60, v_odom=0.0))
    det.reset()
    verdicts = feed(det, series(n=4, v_odom=0.0))
    assert all(v.cause == CAUSE_NONE for v in verdicts)


def test_verdict_reports_evidence_spans():
    det = StuckDetector()
    verdicts = feed(det, series(n=60, v_odom=0.0))
    span = verdicts[-1].span
    assert span["starved"] >= det.cfg.confirm
    assert verdicts[-1].confidence == pytest.approx(1.0, abs=0.2)
    assert verdicts[-1].cmd_dist > verdicts[-1].odom_dist


def test_short_burst_does_not_alarm():
    """0.3 s 的顶住（例如蹭到门框后自己滑开）不应触发脱困。"""
    det = StuckDetector()
    samples = series(n=20, v_odom=0.25) + series(n=6, t0=1.0, v_odom=0.0) + series(n=20, t0=1.3, v_odom=0.25)
    verdicts = feed(det, samples)
    assert all(v.cause == CAUSE_NONE for v in verdicts)
