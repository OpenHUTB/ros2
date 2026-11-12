#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rov_depth_hold 的离线回归测试（不依赖 ROS / Gazebo / 网络）。

实现文件位于 ``scripts/rov_depth_hold.py``；按路径加载，避免被测到别的模块
的同名文件。
"""
import importlib.util
import os
import sys
import tempfile

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
IMPL = os.path.join(HERE, "..", "scripts", "rov_depth_hold.py")


def load_impl():
    spec = importlib.util.spec_from_file_location("rov_depth_impl", IMPL)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["rov_depth_impl"] = mod
    spec.loader.exec_module(mod)
    return mod


impl = load_impl()
TIMES, DEPTHS, TARGETS, THRUSTS, VELS = impl.simulate()
M = impl.evaluate(TIMES, DEPTHS, TARGETS, THRUSTS)


def test_depth_profile_steps():
    assert impl.depth_profile(0.5) == 0.0
    assert impl.depth_profile(2.0) == 2.0


def test_disturbance_windows():
    assert impl.buoyancy_disturbance(6.0) == pytest.approx(-1.5)
    assert impl.buoyancy_disturbance(4.0) == 0.0
    # 波浪段为往复：12 s 后开始但初值为 0，14 s 附近不为 0
    assert impl.buoyancy_disturbance(12.2) != 0.0


def test_rise_time():
    assert M["rise_time"] is not None
    assert 1.0 < M["rise_time"] < 4.0


def test_overshoot_small():
    assert abs(M["overshoot"]) < 0.1


def test_step_steady_error():
    assert M["steady_err"] < 0.05


def test_disturbance_peak_and_recovery():
    assert 0.2 < M["dist_peak"] < 0.8
    assert M["recovery_time"] is not None
    assert M["recovery_time"] < 3.0


def test_wave_residual_and_final_depth():
    assert M["wave_err"] < 0.1
    assert abs(M["final_depth"] - 2.0) < 0.25


def test_thrust_limit():
    ctrl = impl.PIDDepthController()
    for _ in range(500):
        u = ctrl.update(10.0, 0.0, 0.0)
        assert abs(u) <= ctrl.u_limit + 1e-9
    assert abs(ctrl.integral) <= ctrl.i_limit + 1e-9


def test_cli_writes_outputs():
    with tempfile.TemporaryDirectory() as tmp:
        impl.main(["--out-dir", tmp])
        limits = {"depth_response.png": 20000, "depth_demo.gif": 50000,
                  "summary.json": 100}
        for name, limit in limits.items():
            path = os.path.join(tmp, name)
            assert os.path.isfile(path), name
            assert os.path.getsize(path) > limit, name


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
