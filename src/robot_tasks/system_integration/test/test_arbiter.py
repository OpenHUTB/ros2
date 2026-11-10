"""速度仲裁层的单元测试（不需要 ROS）。"""

import pytest

from system_integration.arbiter import VelocityArbiter


def test_manual_passthrough():
    arb = VelocityArbiter(timeout=1.0)
    arb.set_mode('manual')
    arb.feed_manual(0.5, 0.2, now=0.0)
    v, w, st = arb.step(now=0.1)
    assert (v, w) == (0.5, 0.2)
    assert st['source'] == 'manual' and st['reason'] == 'ok'


def test_mode_switch_selects_other_source():
    arb = VelocityArbiter(timeout=1.0)
    arb.feed_manual(0.5, 0.0, now=0.0)
    arb.feed_auto(-0.3, 0.4, now=0.0)
    arb.set_mode('auto')
    v, w, st = arb.step(now=0.1)
    assert (v, w) == (-0.3, 0.4)
    assert st['source'] == 'auto'


def test_switch_to_never_fed_source():
    """切到另一路，但那一帧都没发过，应输出零而不是沿用上一路的速度。"""
    arb = VelocityArbiter(timeout=5.0)
    arb.feed_manual(0.9, 0.3, now=0.0)
    assert arb.step(now=0.1)[:2] == (0.9, 0.3)

    arb.set_mode('auto')
    v, w, st = arb.step(now=0.2)
    assert (v, w) == (0.0, 0.0)
    assert st['reason'] == 'no_data'


def test_switch_to_stale_source():
    """切到另一路，但它的数据已经过期，同样要回零。"""
    arb = VelocityArbiter(timeout=0.5)
    arb.feed_manual(0.9, 0.0, now=0.0)
    arb.feed_auto(-0.4, 0.2, now=0.0)
    arb.set_mode('auto')

    v, w, st = arb.step(now=0.4)
    assert (v, w) == (-0.4, 0.2) and st['reason'] == 'ok'

    v, w, st = arb.step(now=1.0)
    assert (v, w) == (0.0, 0.0)
    assert st['reason'] == 'stale'


def test_feed_rejects_non_finite():
    """NaN / inf 不能进入仲裁层：NaN 的比较恒为 False，会绕过限幅直接下发。"""
    arb = VelocityArbiter(timeout=10.0)
    assert arb.feed_manual(float('nan'), 0.0, now=0.0) is False
    assert arb.feed_manual(0.0, float('nan'), now=0.0) is False
    assert arb.feed_auto(float('inf'), 0.0, now=0.0) is False
    assert arb.feed_auto(0.0, float('-inf'), now=0.0) is False

    v, w, st = arb.step(now=0.1)
    assert (v, w) == (0.0, 0.0)
    assert st['reason'] == 'no_data'


def test_feed_rejects_non_finite_time():
    arb = VelocityArbiter(timeout=1.0)
    assert arb.feed_manual(0.5, 0.0, now=float('nan')) is False
    assert arb.step(now=0.1)[:2] == (0.0, 0.0)


def test_finite_values_still_accepted_after_reject():
    arb = VelocityArbiter(timeout=10.0)
    assert arb.feed_manual(float('nan'), 0.0, now=0.0) is False
    assert arb.feed_manual(0.6, 0.2, now=0.1) is True
    v, w, st = arb.step(now=0.2)
    assert (v, w) == (0.6, 0.2) and st['reason'] == 'ok'


def test_feed_rejects_unknown_source():
    arb = VelocityArbiter()
    assert arb.feed('teleop', 0.5, 0.0) is False
    assert arb.is_stale(now=0.0) is True


def test_watchdog_zero_after_timeout():
    arb = VelocityArbiter(timeout=0.5)
    arb.feed_manual(0.8, 0.0, now=0.0)
    assert arb.step(now=0.4)[:2] == (0.8, 0.0)
    v, w, st = arb.step(now=0.6)
    assert (v, w) == (0.0, 0.0)
    assert st['reason'] == 'stale'


def test_no_data_yields_zero():
    arb = VelocityArbiter()
    v, w, st = arb.step(now=1.0)
    assert (v, w) == (0.0, 0.0)
    assert st['reason'] == 'no_data'


def test_estop_overrides_and_recovers():
    arb = VelocityArbiter(timeout=10.0)
    arb.feed_manual(0.8, 0.5, now=0.0)
    arb.set_estop(True, now=0.5)
    v, w, st = arb.step(now=0.6)
    assert (v, w) == (0.0, 0.0) and st['reason'] == 'estop'

    arb.set_estop(False, now=0.8)
    v, w, st = arb.step(now=0.9)
    assert (v, w) == (0.8, 0.5) and st['reason'] == 'ok'


def test_clamp_limits():
    arb = VelocityArbiter(timeout=10.0, max_linear=1.0, max_angular=1.5)
    arb.feed_manual(3.0, -9.0, now=0.0)
    v, w, _ = arb.step(now=0.1)
    assert v == pytest.approx(1.0)
    assert w == pytest.approx(-1.5)


def test_mode_alias_and_invalid():
    arb = VelocityArbiter()
    assert arb.set_mode('自动') is True and arb.mode == 'auto'
    assert arb.set_mode('manual') is True
    assert arb.set_mode('nonsense') is False
    assert arb.mode == 'manual'


def test_counters():
    arb = VelocityArbiter(timeout=10.0)
    arb.set_mode('auto')
    arb.set_mode('manual')
    arb.set_estop(True)
    arb.set_estop(True)      # 重复置位不重复计数
    arb.set_estop(False)
    assert arb.mode_switches == 2
    assert arb.estop_events == 1


def test_demo_runs():
    from system_integration.demo import run
    rows = run(verbose=False)
    assert len(rows) == 13
    assert any(r['reason'] == 'stale' for r in rows)
    assert any(r['reason'] == 'estop' for r in rows)
