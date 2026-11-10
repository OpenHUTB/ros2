"""检查指令边界与超时行为."""
import pytest
from uav_altitude_command_guard.core import AltitudeGuard


def test_rate_clipping_and_timeout():
    guard = AltitudeGuard()
    guard.tick(0)
    assert guard.command(100, 0)
    assert guard.target == 3
    assert guard.tick(0.2)[0] == pytest.approx(1.6)
    assert guard.tick(0.7) == (pytest.approx(1.6), '超时保持')
    assert guard.tick(1.0)[0] == pytest.approx(1.6)


def test_invalid_command_does_not_refresh_watchdog():
    guard = AltitudeGuard()
    guard.command(2, 0)
    guard.tick(0)
    assert not guard.command(float('nan'), 0.5)
    assert guard.tick(0.6)[1] == '超时保持'
    with pytest.raises(ValueError):
        guard.tick(0.1)
    with pytest.raises(ValueError):
        guard.tick(float('inf'))


def test_parameters_and_restart():
    with pytest.raises(ValueError):
        AltitudeGuard(rate=0)
    guard = AltitudeGuard()
    assert guard.tick(10)[0] == 1.5
    guard.command(-9, 10)
    assert guard.target == 0.4
    assert guard.tick(10.2)[0] == pytest.approx(1.4)
