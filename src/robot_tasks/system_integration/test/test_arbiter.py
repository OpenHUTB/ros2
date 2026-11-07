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
