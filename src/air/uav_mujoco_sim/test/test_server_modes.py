#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RPC 控制模式回归：``[0.0]*4`` 是 falsy，零值指令不得被当作“没给指令”丢弃。

    python -m pytest test/test_server_modes.py -v     # 服务端无头启动，不需 ROS/显示器
"""
import os
import subprocess
import sys

import numpy as np
import pytest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_ROOT)

from uav_mujoco_sim.sim.client import wait_for_server       # noqa: E402

PORT = 41527
HOVER4 = [2.33, 2.33, 2.33, 2.33]      # 0.95 kg 的四桨悬停推力


@pytest.fixture(scope="module")
def server():
    env = dict(os.environ, PYTHONPATH=PKG_ROOT)
    p = subprocess.Popen(
        [sys.executable, "-m", "uav_mujoco_sim.sim", "--port", str(PORT),
         "--scene", "open_ground", "--headless"],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, cwd=PKG_ROOT)
    try:
        wait_for_server("127.0.0.1", PORT, timeout=60)
        yield p
    finally:
        p.terminate()
        try:
            p.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()


def test_optional_vector_keeps_zeros():
    from uav_mujoco_sim.sim.server import _optional_vector
    assert _optional_vector(None) is None
    z = _optional_vector([0.0, 0.0, 0.0, 0.0])
    assert z is not None and z.shape == (4,) and not z.any()
    assert _optional_vector([1, 2, 3]).tolist() == [1.0, 2.0, 3.0]


def test_zero_rotor_actually_stops_thrust(server):
    """四个桨置 0 后无人机开始下坠（修复前 rotor 被解析成 None，仍沿用上一次推力）。"""
    c = wait_for_server("127.0.0.1", PORT, timeout=30)
    assert c.reset(pos=(0.0, 0.0, 3.0))["ok"]
    c.frame(n=50, rotor=HOVER4, want=("odom",))          # 先稳到悬停
    vz0 = float(c.frame(n=1, rotor=HOVER4, want=("odom",))["odom"]["vel"][2])
    vz = float(c.frame(n=40, rotor=[0.0, 0.0, 0.0, 0.0],
                       want=("odom",))["odom"]["vel"][2])
    c.close()
    assert abs(vz0) < 0.3, "起始未稳在悬停：vz0=%.3f" % vz0
    assert vz < -2.0, ("零推力 0.4 s 后垂直速度 %.2f m/s，"
                       "说明零推力指令被当成『没给指令』丢掉了" % vz)


def test_rotor_mode_reports_status_and_is_sticky(server):
    """rotor 直通模式状态为 ROTOR，给零值也不退出该模式。"""
    c = wait_for_server("127.0.0.1", PORT, timeout=30)
    c.reset(pos=(0.0, 0.0, 3.0))
    st_full = c.frame(n=1, rotor=[6.0, 6.0, 6.0, 6.0], want=("odom",))["status"]
    st_zero = c.frame(n=1, rotor=[0.0, 0.0, 0.0, 0.0], want=("odom",))["status"]
    c.close()
    assert st_full == "ROTOR", st_full
    assert st_zero == "ROTOR", st_zero


def test_zero_cmd_vel_is_still_a_command(server):
    """速度指令全零表示“停下”（修复前 cmd_vel 被解析成 None，无人机以 1 m/s 持续前飞）。"""
    c = wait_for_server("127.0.0.1", PORT, timeout=30)
    c.reset(pos=(0.0, 0.0, 3.0))
    c.frame(n=80, cmd_vel=[1.0, 0.0, 0.0, 0.0], want=("odom",))
    v_moving = float(np.linalg.norm(
        c.frame(n=1, cmd_vel=[1.0, 0.0, 0.0, 0.0], want=("odom",))["odom"]["vel"]))
    for _ in range(100):
        r = c.frame(n=1, cmd_vel=[0.0, 0.0, 0.0, 0.0], want=("odom",))
    v_stopped = float(np.linalg.norm(r["odom"]["vel"]))
    c.close()
    assert v_moving > 0.5, "速度指令没生效：v=%.3f" % v_moving
    assert v_stopped < 0.25, \
        "零速度指令 1 s 后仍在运动：%.3f → %.3f m/s" % (v_moving, v_stopped)
