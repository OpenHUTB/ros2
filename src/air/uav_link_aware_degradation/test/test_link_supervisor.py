#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""链路模型与状态机的纯逻辑测试（不需要 MuJoCo / ROS2）。"""
import numpy as np

from uav_link_aware_degradation.core.link import (LinkEmulator, LinkMonitor,
                                                  LinkProfile)
from uav_link_aware_degradation.core.mission import Mission
from uav_link_aware_degradation.core.supervisor import (DegradationSupervisor,
                                                        Mode, SupervisorParams)


def test_outage_drops_packets():
    prof = LinkProfile("t", "t", outages=((1.0, 2.0),))
    emu = LinkEmulator(prof, seed=0)
    sends = [emu.transmit(t) for t in np.arange(0, 4, 0.1)]
    lost = [d for d in sends if not d.delivered]
    # 1.0~2.0 s 内发送的包全部丢
    assert len(lost) >= 9
    assert all(1.0 <= d.send_t < 2.0 for d in lost)


def test_delay_positive():
    prof = LinkProfile("t", "t", one_way_delay=0.25, jitter_std=0.0)
    emu = LinkEmulator(prof, seed=0)
    for t in (0.0, 0.1, 0.2):
        d = emu.transmit(t)
        assert d.delivered and abs(d.arrival - (t + 0.25)) < 1e-9


def test_monitor_warm_flag():
    mon = LinkMonitor(rate_hz=20.0, window=2.0)
    s0 = mon.update(0.0)
    assert not s0.warm
    for t in np.arange(0.0, 5.0, 0.05):
        mon.push(t, t + 0.05)
    s = mon.update(4.0)
    assert s.warm and s.pdr > 0.95


def _mission():
    return Mission(home=np.array([0.0, 0.0, 1.2]),
                   waypoints=[np.array([5.0, 0.0, 1.5])])


def _run(ages, pdr=1.0, warm=True):
    sv = DegradationSupervisor(SupervisorParams())
    mission = _mission()
    out = []
    for k, age in enumerate(ages):
        from uav_link_aware_degradation.core.link import LinkStatus
        st = LinkStatus(t=k * 0.05, age=age, pdr=pdr, jitter=0.0,
                        silent_s=age, warm=warm)
        d = sv.update(k * 0.05, st, np.array([0.0, 0.0, 1.2]), mission)
        out.append((k * 0.05, d.mode))
    return sv, out


def test_degrade_then_return():
    ages = [0.05] * 10 + [0.05] * 40 + [0.6] * 10 + [2.0] * 40
    sv, out = _run(ages)
    modes = [m for _, m in out]
    assert Mode.NORMAL in modes[:10]
    assert Mode.DEGRADED in modes
    assert Mode.RETURN in modes
    assert sv.switches >= 2


def test_no_false_trigger_on_clean():
    ages = [0.05] * 100
    sv, out = _run(ages)
    assert all(m == Mode.NORMAL for _, m in out)
    assert sv.switches == 0


def test_hysteresis_recovery_needs_dwell():
    ages = [0.05] * 10 + [0.6] * 10 + [0.05] * 5 + [2.0] * 20
    sv, out = _run(ages)
    modes = [m for _, m in out]
    # 短暂恢复（0.25 s，不足 recover_hold 2 s）不应把状态切回 NORMAL
    first_deg = modes.index(Mode.DEGRADED)
    assert Mode.RETURN in modes
    assert Mode.NORMAL not in modes[first_deg:]
