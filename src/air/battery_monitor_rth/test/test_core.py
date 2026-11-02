#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""battery_monitor_rth 单元测试（不依赖 ROS）。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core import (  # noqa: E402
    BatterySim, RthMonitor, open_circuit_voltage, pack_voltage,
)


def test_ocv_range():
    """开路电压应在锂电池合理区间, 且随 SOC 单调上升。"""
    v100 = open_circuit_voltage(1.0)
    v0 = open_circuit_voltage(0.0)
    assert 4.0 < v100 <= 4.25
    assert 3.2 <= v0 < 3.4
    socs = [i / 100 for i in range(101)]
    volts = [open_circuit_voltage(s) for s in socs]
    assert all(volts[i] <= volts[i + 1] + 1e-9 for i in range(len(volts) - 1))


def test_pack_voltage_load_sag():
    """负载电流越大, 端电压越低。"""
    v_light = pack_voltage(0.8, 5.0)
    v_heavy = pack_voltage(0.8, 30.0)
    assert v_heavy < v_light


def test_battery_soc_decreases():
    sim = BatterySim(soc0=1.0)
    prev = sim.soc
    for _ in range(100):
        soc = sim.step(20.0, 1.0)
        assert soc < prev
        prev = soc


def test_soc_never_negative():
    sim = BatterySim(soc0=0.01)
    for _ in range(1000):
        sim.step(50.0, 1.0)
    assert sim.soc == 0.0


def test_rth_levels():
    """分级触发: 50%->INFO, 25%->RTH, 10%->LAND, 每级只触发一次。"""
    m = RthMonitor()
    assert m.check(0.6) is None
    assert m.check(0.48) == 'INFO'
    assert m.check(0.48) is None          # 不重复触发
    assert m.check(0.24) == 'RTH'
    assert m.check(0.09) == 'LAND'


def test_full_mission_rth_triggers():
    """5A 悬停电流仿真: 足够长时间后 RTH 应触发。"""
    sim = BatterySim(soc0=1.0)
    m = RthMonitor()
    events = []
    t = 0.0
    while t < 7200:  # 最多 2 小时
        soc = sim.step(5.0, 1.0)
        ev = m.check(soc)
        if ev:
            events.append((t, ev))
        t += 1.0
    names = [e for _, e in events]
    assert 'INFO' in names and 'RTH' in names
