#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_l1_altitude_stabilization 的离线回归测试（不依赖 ROS / PX4 / Gazebo / 网络）。

实现文件位于 ``scripts/l1_altitude_stab.py``；按路径加载，避免被测到别的模块的同名文件。
"""
import importlib.util
import json
import os
import sys
import tempfile

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
IMPL = os.path.join(HERE, "..", "scripts", "uav_l1_altitude_stabilization.py")


def load_impl():
    spec = importlib.util.spec_from_file_location("l1_alt_impl", IMPL)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["l1_alt_impl"] = mod
    spec.loader.exec_module(mod)
    return mod


impl = load_impl()
TIMES, ZS, TARGETS, THRUSTS, WINDS, SIGMAS = impl.simulate("l1")
M = impl.evaluate(TIMES, ZS, TARGETS, THRUSTS)
_, ZS_PD, _, THRUSTS_PD, _, _ = impl.simulate("pd")
M_PD = impl.evaluate(TIMES, ZS_PD, TARGETS, THRUSTS_PD)


def test_altitude_profile_steps():
    assert impl.altitude_profile(0.5) == pytest.approx(0.0)
    assert impl.altitude_profile(3.0) == pytest.approx(2.0)
    assert impl.altitude_profile(12.0) == pytest.approx(3.5)
    arr = impl.altitude_profile(np.array([0.5, 3.0, 12.0]))
    assert arr.shape == (3,)


def test_wind_windows():
    assert impl.wind(4.0) == 0.0
    assert impl.wind(6.0) == pytest.approx(-2.0)
    assert impl.wind(9.0) == 0.0
    # 往复风扰从 13 s 起，13 s 处正弦值为 0
    assert abs(impl.wind(13.0)) < 1e-9
    assert impl.wind(13.2) != 0.0


def test_plant_hover_equilibrium():
    """推力刚好等于自重、无风扰时，质点保持静止。"""
    plant = impl.UAVAltitudePlant(dt=0.001)
    for _ in range(1000):
        plant.step(impl.G)
    assert plant.z == pytest.approx(0.0, abs=1e-6)
    assert plant.v == pytest.approx(0.0, abs=1e-6)


def test_controller_initial_estimate():
    """扰动估计初值取 -g，表示自重已配平；否则起飞瞬间会掉高度。"""
    ctrl = impl.L1AltitudeController()
    ctrl.reset()
    assert ctrl.sigma_hat == pytest.approx(-impl.G)
    assert ctrl.sigma_f == pytest.approx(-impl.G)


def test_estimate_is_clipped():
    """投影限幅：即使自适应律被灌入大误差，估计量也不超过 ±sigma_limit。"""
    ctrl = impl.L1AltitudeController(sigma_limit=25.0)
    ctrl.reset()
    for _ in range(2000):
        ctrl.update(10.0, 0.0, 50.0)  # 虚假的大高度误差与速度
        assert abs(ctrl.sigma_hat) <= 25.0 + 1e-9


def test_thrust_limit_respected():
    assert np.max(np.abs(THRUSTS)) <= 20.0 + 1e-9
    assert np.max(np.abs(THRUSTS_PD)) <= 20.0 + 1e-9


def test_step_response():
    assert M["rise_time"] is not None
    assert 1.0 < M["rise_time"] < 2.2
    assert M["overshoot"] < 0.05
    assert M["steady_err"] < 0.05


def test_gust_rejection_beats_pd_baseline():
    """同样的增益与限幅，仅多了自适应补偿：阵风峰值偏差应显著更小。"""
    assert M["gust_peak"] < 0.15
    assert M["gust_peak"] < 0.5 * M_PD["gust_peak"]
    assert M["recovery_time"] is not None and M["recovery_time"] < 3.0
    assert M_PD["recovery_time"] is None  # 基线在阵风窗口内没回到 ±0.02 m


def test_wave_disturbance_and_final_altitude():
    assert M["wave_err"] < 0.05
    assert M["final_alt"] == pytest.approx(3.5, abs=0.1)


def test_main_writes_artifacts():
    """main() 在 --out-dir 下产出 PNG / GIF / summary.json 三件套。"""
    with tempfile.TemporaryDirectory() as tmp:
        impl.main(["--out-dir", tmp])
        for name, least in (("altitude_response.png", 10240),
                            ("altitude_demo.gif", 102400),
                            ("summary.json", 200)):
            path = os.path.join(tmp, name)
            assert os.path.exists(path), name
            assert os.path.getsize(path) > least, name
        with open(os.path.join(tmp, "summary.json"), encoding="utf-8") as f:
            data = json.load(f)
        assert data["module"] == "uav_l1_altitude_stabilization"
        assert "gust_peak" in data["metrics"]
