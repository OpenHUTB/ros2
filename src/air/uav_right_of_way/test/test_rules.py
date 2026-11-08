#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""规则引擎（core.rules）单元测试：四条规则、动作词表与理由字符串。"""
import math

import numpy as np
import pytest

from uav_right_of_way.core.detect import AircraftState, Threat
from uav_right_of_way.core.rules import (ACTION_DESCEND, ACTION_MAINTAIN,
                                         ACTION_OVERTAKE_LEFT, ACTION_SLOW,
                                         ACTION_TURN_RIGHT, Action, RuleParams,
                                         classify, decide)

RP = RuleParams()


def state(uid, pos, vel, priority=0):
    return AircraftState(uid, np.asarray(pos, float),
                         np.asarray(vel, float), priority)


# ------------------------------------------------------------------ ① 优先级
def test_priority_low_yields_headon_turn_right():
    # 对头几何：低优先级 → 右转 30°（规则①）
    own = state("A", (0, 0, 15), (8, 0, 0), priority=0)
    other = state("B", (100, 0, 15), (-8, 0, 0), priority=10)
    dec = decide(own, other, Threat("B", 6.0, 5.0), 1.0, RP)
    assert dec.rule == "①优先级"
    assert dec.action.kind == ACTION_TURN_RIGHT
    assert dec.action.value == pytest.approx(30.0)
    assert "优先级" in dec.reason and "10" in dec.reason


def test_priority_low_yields_cross_descend():
    # 交叉几何：低优先级 → 降高 10 m（规则①）
    own = state("A", (0, 0, 15), (8, 0, 0), priority=0)
    other = state("B", (0, 100, 15), (0, -8, 0), priority=10)
    dec = decide(own, other, Threat("B", 6.0, 5.0), 1.0, RP)
    assert dec.action.kind == ACTION_DESCEND
    assert dec.action.value == pytest.approx(10.0)


def test_priority_high_stand_on():
    # 高优先级一方：保持航向航速，由对方让路
    own = state("A", (100, 0, 15), (-8, 0, 0), priority=10)
    other = state("B", (0, 0, 15), (8, 0, 0), priority=0)
    dec = decide(own, other, Threat("B", 6.0, 5.0), 1.0, RP)
    assert dec.rule == "①优先级"
    assert dec.action.kind == ACTION_MAINTAIN


# ------------------------------------------------------------------ ③ 对头
def test_headon_both_turn_right():
    a = state("A", (0, 0, 15), (8, 0, 0))
    b = state("B", (100, 0, 15), (-8, 0, 0))
    da = decide(a, b, Threat("B", 6.0, 5.0), 1.0, RP)
    db = decide(b, a, Threat("A", 6.0, 5.0), 1.0, RP)
    assert da.rule == "③对头" and db.rule == "③对头"
    assert da.action.kind == ACTION_TURN_RIGHT
    assert db.action.kind == ACTION_TURN_RIGHT
    assert "右转" in da.reason


def test_headon_classification():
    a = state("A", (0, 0, 15), (8, 0, 0))
    b = state("B", (100, 5, 15), (-8, 0, 0))   # 接近正前方迎面
    assert classify(a, b, RP) == "headon"
    # 大航向差但对方在侧后方 → 不算对头
    c = state("C", (-50, -60, 15), (-8, 0, 0))
    assert classify(a, c, RP) != "headon"


# ------------------------------------------------------------------ ② 左让右
def test_crossing_give_way_to_right():
    # 对方在本机右侧（叉积 < 0）→ 本机右转让行
    own = state("A", (0, 0, 15), (8, 0, 0))            # 东行
    other = state("B", (0, -80, 15), (0, 8, 0))        # 南边北行 → 在右侧
    dec = decide(own, other, Threat("B", 5.0, 10.0), 1.0, RP)
    assert dec.rule == "②左让右"
    assert dec.action.kind == ACTION_TURN_RIGHT
    assert "右侧" in dec.reason


def test_crossing_stand_on_when_on_right():
    # 本机在对方右侧 → 对方让路，本机保持
    own = state("A", (0, 0, 15), (0, 8, 0))            # 北行
    other = state("B", (-80, 0, 15), (8, 0, 0))        # 西边东行 → 在本机左侧
    dec = decide(own, other, Threat("B", 5.0, 10.0), 1.0, RP)
    assert dec.rule == "②左让右"
    assert dec.action.kind == ACTION_MAINTAIN


# ------------------------------------------------------------------ ④ 追越
def test_overtake_front_maintains():
    front = state("A", (50, 0, 15), (5, 0, 0))
    rear = state("B", (0, 0, 15), (9, 0, 0))           # 正后方快机
    dec = decide(front, rear, Threat("B", 8.0, 30.0), 1.0, RP)
    assert dec.rule == "④追越"
    assert dec.action.kind == ACTION_MAINTAIN
    assert "前机" in dec.reason and "保" in dec.reason


def test_overtake_rear_directly_behind_goes_left():
    front = state("A", (40, 0, 15), (5, 0, 0))
    rear = state("B", (0, 0, 15), (9, 0, 0))
    dec = decide(rear, front, Threat("A", 8.0, 30.0), 1.0, RP)
    assert dec.rule == "④追越"
    assert dec.action.kind == ACTION_OVERTAKE_LEFT
    assert "左侧" in dec.reason


def test_overtake_rear_on_left_keeps_overtaking():
    front = state("A", (40, 0, 15), (5, 0, 0))
    rear = state("B", (0, 10, 15), (9, 0, 0))          # 已在左侧
    dec = decide(rear, front, Threat("A", 8.0, 30.0), 1.0, RP)
    assert dec.action.kind == ACTION_OVERTAKE_LEFT
    assert "已具左侧几何" in dec.reason


def test_overtake_rear_on_right_slows():
    front = state("A", (40, 0, 15), (5, 0, 0))
    rear = state("B", (0, -10, 15), (9, 0, 0))         # 在右侧，不能从左侧追越
    dec = decide(rear, front, Threat("A", 8.0, 30.0), 1.0, RP)
    assert dec.action.kind == ACTION_SLOW
    assert dec.action.value == pytest.approx(0.30)
    assert "右侧" in dec.reason


def test_overtake_classification():
    a = state("A", (0, 0, 15), (8, 0, 0))
    b = state("B", (60, 3, 15), (8.2, 0, 0))
    assert classify(a, b, RP) == "overtake"


# ------------------------------------------------------------------ 审计
def test_decision_reason_and_dict_audit():
    own = state("A", (0, 0, 15), (8, 0, 0))
    other = state("B", (100, 0, 15), (-8, 0, 0))
    dec = decide(own, other, Threat("B", 6.25, 2.0), 12.5, RP)
    d = dec.as_dict()
    assert d["uid"] == "A" and d["other"] == "B"
    assert d["rule"] and d["reason"]
    assert d["t"] == pytest.approx(12.5)
    assert d["cpa_dist"] == pytest.approx(2.0)
    assert d["action"]["kind"] == ACTION_TURN_RIGHT
    # 理由字符串包含机号与规则标签（可审计）
    assert "B" in dec.reason and dec.rule in dec.reason


def test_action_describe_vocabulary():
    assert "30" in Action(ACTION_TURN_RIGHT, 30).describe()
    assert "10" in Action(ACTION_DESCEND, 10).describe()
    assert "30" in Action(ACTION_SLOW, 0.30).describe()
    assert "保" in Action(ACTION_MAINTAIN).describe()
    assert "20" in Action(ACTION_OVERTAKE_LEFT, 20).describe()


def test_low_speed_maintains():
    own = state("A", (0, 0, 15), (0.0, 0.0, 0.0))
    other = state("B", (100, 0, 15), (-8, 0, 0))
    dec = decide(own, other, Threat("B", 8.0, 10.0), 1.0, RP)
    assert dec.action.kind == ACTION_MAINTAIN
    assert "速度过低" in dec.reason
