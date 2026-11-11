#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ugv_cruise_pid 的离线回归测试（不依赖 ROS / Gazebo / 网络）。

实现文件位于 ``scripts/cruise_pid_node.py``，仓库里同名 ``main.py`` 很多，
因此按路径加载，避免被测到别的模块。
"""
import importlib.util
import os
import sys
import tempfile

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
IMPL = os.path.join(HERE, "..", "scripts", "cruise_pid_node.py")


def load_impl():
    spec = importlib.util.spec_from_file_location("ugv_cruise_impl", IMPL)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["ugv_cruise_impl"] = mod
    spec.loader.exec_module(mod)
    return mod


impl = load_impl()
TIMES, VS, VTS, XS, ACCELS = impl.simulate()
M = impl.evaluate(TIMES, VS, VTS, XS)


def test_profile_steps():
    assert impl.speed_profile(0.0) == 0.0
    assert impl.speed_profile(5.0) == 5.0
    assert impl.speed_profile(12.0) == 8.0
    assert impl.speed_profile(16.0) == 3.0


def test_gust_window():
    assert impl.wind_gust(13.0) == -1.0
    assert impl.wind_gust(11.0) == 0.0
    assert impl.wind_gust(15.0) == 0.0


def test_step_steady_error():
    # 8 m/s 阶跃段（阵风前）稳态误差应小于 0.2 m/s
    assert M["steady_err"] < 0.2


def test_step_overshoot():
    assert 0.0 <= M["overshoot"] < 0.3


def test_rise_time():
    assert M["rise_time"] is not None
    assert 0.1 < M["rise_time"] < 2.0


def test_gust_and_residual():
    # 阵风期间速度确实被压低，阵风结束后能拉回目标附近
    assert 0.2 < M["gust_peak"] < 1.5
    assert M["residual"] < 0.25


def test_final_speed_and_distance():
    assert abs(M["final_speed"] - 3.0) < 0.2
    assert 80.0 < M["distance"] < 110.0


def test_integral_anti_windup():
    ctrl = impl.PIDCruiseController()
    for _ in range(500):
        ctrl.update(20.0, 0.0)
    assert abs(ctrl.integral) <= ctrl.i_limit + 1e-9
    assert abs(ctrl.update(20.0, 0.0)) <= ctrl.a_limit + 1e-9


def test_cli_writes_outputs():
    with tempfile.TemporaryDirectory() as tmp:
        impl.main(["--out-dir", tmp])
        limits = {"cruise_response.png": 20000, "cruise_demo.gif": 50000,
                  "summary.json": 100}
        for name, limit in limits.items():
            path = os.path.join(tmp, name)
            assert os.path.isfile(path), name
            assert os.path.getsize(path) > limit, name


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
