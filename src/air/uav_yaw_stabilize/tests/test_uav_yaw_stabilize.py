#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_yaw_stabilize 的离线回归测试（不依赖 ROS / Gazebo / 网络）。

实现文件位于 ``scripts/uav_yaw_stabilize.py``；按路径加载，避免被测到别的
模块的同名文件。
"""
import importlib.util
import os
import sys
import tempfile

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
IMPL = os.path.join(HERE, "..", "scripts", "uav_yaw_stabilize.py")


def load_impl():
    spec = importlib.util.spec_from_file_location("uav_yaw_impl", IMPL)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["uav_yaw_impl"] = mod
    spec.loader.exec_module(mod)
    return mod


impl = load_impl()
TIMES, YAWS, TARGETS, TORQUES, RATES = impl.simulate()
M = impl.evaluate(TIMES, YAWS, TARGETS, TORQUES)


def test_wrap_pi():
    assert impl.wrap_pi(0.0) == pytest.approx(0.0)
    assert impl.wrap_pi(np.pi + 0.1) == pytest.approx(-np.pi + 0.1)
    assert impl.wrap_pi(-np.pi - 0.1) == pytest.approx(np.pi - 0.1)


def test_yaw_profile_steps():
    assert impl.yaw_profile(3.0) == pytest.approx(np.pi / 4)
    assert impl.yaw_profile(9.0) == pytest.approx(-np.pi / 6)
    # 标量与数组两种调用方式
    arr = impl.yaw_profile(np.array([2.0, 10.0]))
    assert arr.shape == (2,)


def test_wind_torque_windows():
    assert impl.wind_torque(5.5) == pytest.approx(0.06)
    assert impl.wind_torque(4.0) == 0.0
    assert impl.wind_torque(10.2) != 0.0


def test_first_step_behaviour():
    assert M["rise_time"] is not None
    assert 0.2 < M["rise_time"] < 1.5
    assert abs(M["overshoot"]) < 5.0
    assert M["steady_err"] < 0.5


def test_gust_response():
    # 阵风把航向推离指令，但幅度有限且能在 2 s 内拉回
    assert 0.5 < M["gust_peak"] < 10.0
    assert M["recovery_time"] is not None
    assert M["recovery_time"] < 2.0


def test_second_step_settle():
    assert M["settle_time"] is not None
    assert M["settle_time"] < 3.0


def test_wave_and_final_yaw():
    assert M["wave_dev"] < 2.0
    assert abs(M["final_yaw"] - (-30.0)) < 3.0


def test_torque_limit_and_error_wrap():
    ctrl = impl.PDYawController()
    big = np.pi - 0.01          # 越过 ±180° 的边界
    u = ctrl.update(big + 0.02, -np.pi + 0.005, 0.0)
    assert abs(u) <= ctrl.torque_limit + 1e-9


def test_cli_writes_outputs():
    with tempfile.TemporaryDirectory() as tmp:
        impl.main(["--out-dir", tmp])
        limits = {"yaw_response.png": 20000, "yaw_demo.gif": 50000,
                  "summary.json": 100}
        for name, limit in limits.items():
            path = os.path.join(tmp, name)
            assert os.path.isfile(path), name
            assert os.path.getsize(path) > limit, name


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
