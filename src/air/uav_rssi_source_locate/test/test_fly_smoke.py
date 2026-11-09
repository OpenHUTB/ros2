#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Closed-loop smoke tests: real MuJoCo flight searching each scenario."""

import numpy as np
import pytest

from uav_rssi_source_locate.core.cases import make_scenario
from uav_rssi_source_locate.core.flyer import (REF_LEASH_M, Beacon,
                                               locate_beacon)
from uav_rssi_source_locate.core.rssi import RssiCfg

RC = RssiCfg()

pytest.importorskip("mujoco", reason="closed-loop tests need mujoco")


def test_center_scenario_converges_to_metre_level():
    sc = make_scenario("center")
    res = locate_beacon(Beacon(sc.beacon_xy), sc.cfg, RC)
    assert res.outcome == "ok"
    assert res.detected_t is not None and res.detected_t > 0.0
    assert res.error_m < 2.0
    # 复核点的实测 RSSI 必须与"估计点即信标"自洽
    assert res.verified
    assert res.n_measurements >= 8
    # 全程不超过时域上限，且飞行高度没有掉高（参考拴绳在线性区工作）
    assert res.sim_s < 900.0


def test_northwest_mirror_case_does_not_keep_the_ghost_solution():
    # 西北角的首圈只剩单侧弧：最小二乘在主轴另一侧有一个残差更小的
    # 幽灵解（实测偏 57 m、残差 0.05 m）。镜像 + 出界判据必须把它翻
    # 回来，否则定位误差是几十米而不是米级。
    sc = make_scenario("northwest")
    res = locate_beacon(Beacon(sc.beacon_xy), sc.cfg, RC)
    assert res.outcome == "ok"
    assert res.error_m < 3.0
    # 估计点必须仍在搜索区内
    assert 0.0 <= res.estimate[0] <= sc.cfg.area[0]
    assert 0.0 <= res.estimate[1] <= sc.cfg.area[1]


def test_flight_stays_at_search_altitude():
    sc = make_scenario("southeast")
    res = locate_beacon(Beacon(sc.beacon_xy), sc.cfg, RC)
    z = res.pos[:, 2]
    assert abs(float(np.median(z[len(z) // 4:])) - sc.cfg.alt_m) < 1.0
    # 追踪参考点时不追到掉高：全称最低高度不低于搜索高度的一半
    assert float(np.min(z[len(z) // 4:])) > 0.5 * sc.cfg.alt_m
    assert REF_LEASH_M > 0.0
