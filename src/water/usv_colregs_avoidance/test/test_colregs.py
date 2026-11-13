"""COLREGs 判定、CPA 与场景测试（对应 review 修正后的行为）。"""

from __future__ import annotations

import math

import pytest

from usv_colregs_avoidance.colregs import (
    Encounter,
    ShipState,
    Thresholds,
    apply_course_offset,
    decide,
    relative_geometry,
    wrap_to_pi,
)
from usv_colregs_avoidance.simulation import default_scenarios, run_scenario

OWN = ShipState(0.0, 0.0, 0.0, 5.0)
TH = Thresholds()


def test_wrap_to_pi():
    assert wrap_to_pi(0.0) == pytest.approx(0.0)
    assert wrap_to_pi(math.pi) == pytest.approx(math.pi)
    assert wrap_to_pi(3 * math.pi) == pytest.approx(math.pi)
    assert wrap_to_pi(-3 * math.pi / 2) == pytest.approx(math.pi / 2)


def test_head_on_detected():
    enc = relative_geometry(OWN, ShipState(1000.0, 0.0, math.pi, 5.0), TH)
    assert enc.aspect == "head_on"
    assert enc.cpa_m < 5.0
    assert 90.0 < enc.tcpa_s < 110.0


def test_crossing_give_way_on_starboard():
    enc = relative_geometry(OWN, ShipState(800.0, 600.0, -math.pi / 2, 5.0), TH)
    assert enc.aspect == "crossing_give_way"
    assert enc.bearing_deg > 0.0


def test_crossing_stand_on_on_port():
    enc = relative_geometry(OWN, ShipState(800.0, -600.0, math.pi / 2, 5.0), TH)
    assert enc.aspect == "crossing_stand_on"
    assert enc.bearing_deg < 0.0


def test_overtaken_when_target_astern():
    enc = relative_geometry(OWN, ShipState(-800.0, 0.0, 0.0, 8.0), TH)
    assert enc.aspect == "overtaken"


def test_opening_is_not_dangerous():
    """目标同向更快、正在拉开：不应判 danger，也不应机动。"""
    enc = relative_geometry(OWN, ShipState(1000.0, 0.0, 0.0, 10.0), TH)
    assert enc.aspect == "opening"
    assert enc.opening is True
    d = decide(enc, TH)
    assert d.action == "hold"
    assert d.urgency == "none"


def test_far_fast_head_on_is_not_clear():
    """3 km 外 30 m/s 对开：TCPA 只有 ~90 s，不能判 clear。"""
    enc = relative_geometry(OWN, ShipState(3200.0, 0.0, math.pi, 30.0), TH)
    assert enc.aspect != "clear"
    assert enc.tcpa_s < TH.tcpa_warn
    d = decide(enc, TH)
    assert d.action == "alter_starboard"


def test_own_overtaking_uses_rule_13():
    """正前方同向、目标更慢：本船是追越船，应按 Rule 13 让路。"""
    enc = relative_geometry(OWN, ShipState(1000.0, 0.0, 0.0, 2.0), TH)
    assert enc.aspect == "overtaking_own"
    d = decide(enc, TH)
    assert d.rule == "Rule 13"
    assert d.action == "alter_starboard"
    assert d.course_offset_deg > 0.0


def test_same_course_same_speed_is_clear():
    enc = relative_geometry(OWN, ShipState(600.0, 0.0, 0.0, 5.0), TH)
    assert enc.aspect == "clear"
    assert enc.tcpa_s == float("inf")
    d = decide(enc, TH)
    assert d.action == "hold"


def test_give_way_monitors_when_no_risk():
    """右舷目标但 CPA/TCPA 充裕：只监视，不立即 +45°。"""
    enc = Encounter(30.0, "crossing_give_way", 2000.0, 300.0, 2500.0)
    d = decide(enc, TH)
    assert d.action == "hold"
    assert "监视" in d.rule


def test_decide_head_on_when_danger():
    enc = Encounter(0.0, "head_on", 10.0, 30.0, 300.0)
    d = decide(enc, TH)
    assert d.rule == "Rule 14"
    assert d.course_offset_deg > 0.0
    assert d.urgency == "danger"


def test_decide_stand_on_holds_when_safe():
    enc = Encounter(-30.0, "crossing_stand_on", TH.cpa_safe + 100, 300.0, 2000.0)
    d = decide(enc, TH)
    assert d.action == "hold"
    assert d.rule == "Rule 17(a)"


def test_decide_stand_on_acts_when_danger():
    enc = Encounter(-30.0, "crossing_stand_on", TH.cpa_warn - 50, 30.0, 300.0)
    d = decide(enc, TH)
    assert d.action == "emergency_starboard"
    assert d.rule == "Rule 17(b)"


def test_apply_course_offset_wraps():
    s = apply_course_offset(ShipState(0, 0, math.radians(170), 5.0), 30.0)
    assert s.heading == pytest.approx(math.radians(-160.0))


def test_scenarios_improve_clearance():
    results = {r.name: r for r in default_scenarios()}
    assert results["head_on"].min_distance_compliant > 100.0
    assert results["head_on"].min_distance_inaction < 5.0
    assert results["crossing_give_way"].min_distance_compliant > \
        results["crossing_give_way"].min_distance_inaction
    assert results["overtaken"].min_distance_compliant > 100.0
    assert results["overtaken"].min_distance_inaction < 10.0


def test_overtaking_scenario_intervenes():
    results = {r.name: r for r in default_scenarios()}
    over = results["overtaking_own"]
    assert over.intervened is True
    assert over.rule == "Rule 13"


def test_opening_scenario_does_not_intervene():
    results = {r.name: r for r in default_scenarios()}
    opening = results["opening"]
    assert opening.intervened is False
    assert opening.action == "hold"
    assert opening.min_distance_compliant == pytest.approx(
        opening.min_distance_inaction)


def test_scenarios_record_setup():
    for r in default_scenarios():
        assert r.initial_distance > 0
        assert r.duration_s > 0
        assert r.dt > 0


def test_all_give_way_actions_are_starboard():
    for r in default_scenarios():
        if r.action in ("alter_starboard", "emergency_starboard"):
            assert r.course_offset_deg > 0.0


def test_high_speed_closing_scenario_is_handled():
    """3200 m 外 30 m/s 对开：闭环判定必须介入。"""
    own = ShipState(0.0, 0.0, 0.0, 5.0)
    r = run_scenario("fast_head_on", own, ShipState(3200.0, 0.0, math.pi, 30.0),
                     duration=200.0)
    assert r.intervened is True
    assert r.min_distance_compliant > r.min_distance_inaction
