#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""流水线（core.pipeline）测试：--no-sim 路径、JSON 结构与汇总表。"""
import json
import math

import numpy as np
import pytest

from uav_right_of_way.core.detect import DetectParams
from uav_right_of_way.core.pipeline import (format_report, format_table,
                                            run_case, to_json)


@pytest.fixture(scope="module")
def headon_nosim():
    return run_case("headon", sim=False)


def test_nosim_runs_both_modes(headon_nosim):
    r = headon_nosim
    assert r["scenario"] == "headon"
    assert r["closed_loop"] is False
    assert r["baseline"]["decisions"] == 0
    assert r["rules"]["decisions"] >= 2
    # 规则启用显著改善最小间距
    assert r["rules"]["min_separation_m"] > r["baseline"]["min_separation_m"]
    assert r["rules"]["min_separation_m"] > 5.0


def test_result_payload_structure(headon_nosim):
    r = headon_nosim
    for key in ("scenario", "title", "description", "params", "plans",
                "baseline", "rules", "trajectories", "wall_time_s"):
        assert key in r
    assert r["params"]["horizon_s"] == 8.0
    assert r["params"]["cpa_threshold_m"] == 40.0
    b = r["baseline"]
    assert b["min_separation_m"] > 0
    assert len(b["min_pair"]) == 2
    rl = r["rules"]
    assert isinstance(rl["decision_log"], list) and rl["decision_log"]
    d0 = rl["decision_log"][0]
    for key in ("t", "uid", "other", "rule", "action", "reason",
                "cpa_t", "cpa_dist"):
        assert key in d0
    # 轨迹数据可供配图
    tr = r["trajectories"]
    assert set(tr) == {"baseline", "rules"}
    assert set(tr["rules"]["agents"]) == {"UAV-1", "UAV-2", "UAV-3", "UAV-4"}


def test_nosim_all_scenarios_rules_improve():
    for name in ("cross", "overtake"):
        r = run_case(name, sim=False)
        assert r["rules"]["min_separation_m"] > r["baseline"]["min_separation_m"]
        assert r["rules"]["decisions"] > 0
        # 决策理由全部非空（可审计）
        assert all(d["reason"] for d in r["rules"]["decision_log"])


def test_to_json_roundtrip(headon_nosim, tmp_path):
    path = str(tmp_path / "out.json")
    assert to_json([headon_nosim], path) == path
    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    assert isinstance(data, list) and len(data) == 1
    assert data[0]["scenario"] == "headon"
    assert data[0]["rules"]["min_separation_m"] == \
        headon_nosim["rules"]["min_separation_m"]


def test_reports_contain_key_numbers(headon_nosim):
    rep = format_report(headon_nosim)
    assert "headon" in rep
    assert "%.2f" % headon_nosim["baseline"]["min_separation_m"] in rep
    table = format_table([headon_nosim])
    assert "headon" in table
    assert "%d" % headon_nosim["rules"]["decisions"] in table


def test_unknown_scenario_rejected():
    with pytest.raises(KeyError):
        run_case("no_such_scenario", sim=False)
