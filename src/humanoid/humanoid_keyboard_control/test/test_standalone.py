# -*- coding: utf-8 -*-
"""
人形机器人仿真与控制自动化测试
==============================
覆盖（课程任务①"物理仿真 + 键盘运动控制"的核心指标）：
  1. 站姿稳定性：4 s 站立质心零漂移、根高保持；
  2. 前进驱动：位移 ≈ v × t（沿机器人朝向）；
  3. 转向后沿朝向移动：先转向再前进，位移方向与偏航角一致；
  4. 姿态级转向：偏航角 ≈ w × t 且全程保持站立；
  5. 相机渲染可用；
  6. 仿真性能：实时因子（每秒物理步数）满足演示要求。

运行方式（包根目录）：
    python3 test/test_standalone.py        # 独立运行，失败时退出码为 1
    python3 -m pytest test/ -v             # pytest / colcon 收集
"""

import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from humanoid_keyboard_control.sim_core import HumanoidSim

try:
    import pytest

    @pytest.fixture(name="sim")
    def _sim_fixture():
        """pytest 用的仿真实例。

        各测试函数的第一参数 ``sim`` 在脚本模式下由 ``main()`` 显式传入；
        用 pytest/colcon 收集时必须由 fixture 提供，否则会被当成未定义的
        fixture 而在 setup 阶段直接报错（测试根本没机会执行）。
        """
        return HumanoidSim()

except ImportError:                       # 未安装 pytest（直接跑脚本）时不需要 fixture
    pass


def test_standing(sim, seconds=4.0):
    """站姿稳定性：质心零漂移、根高保持。"""
    sim.reset()
    com0 = sim.data.subtree_com[1].copy()
    z0 = sim.read_odom()['z']
    steps = int(seconds / sim.dt)
    for _ in range(steps):
        sim.step()
    com = sim.data.subtree_com[1]
    z = sim.read_odom()['z']
    assert abs(com[0] - com0[0]) < 0.4 and abs(com[2] - com0[2]) < 0.4, "站姿质心漂移"
    assert z > 1.0 and abs(z - z0) < 0.3, "站姿高度异常"
    print(f"[PASS] 站姿稳定性: {seconds:.0f}s 站立，质心位移 "
          f"({com[0]-com0[0]:+.3f}, {com[2]-com0[2]:+.3f})，根高 {z:.3f} m")


def test_drive(sim, v=0.3, t_forward=1.5):
    """前进驱动：位移 ≈ v × t（初始朝向即世界系 +x）。"""
    sim.reset()
    pos0 = sim.read_odom()
    steps = int(t_forward / sim.dt)
    for _ in range(steps):
        sim.step(v_lin=v)
    pos = sim.read_odom()
    dx = pos['x'] - pos0['x']
    expected = v * t_forward
    assert abs(dx - expected) < 0.15, f"前进位移误差过大: {dx:.3f} vs {expected:.3f}"
    assert pos['z'] > 0.9, "前进过程中倾倒"
    print(f"[PASS] 前进驱动: {t_forward:.1f}s 位移 {dx:.3f} m (期望 ≈{expected:.3f} m)")


def test_drive_heading(sim, w=0.9, t_turn=1.0, v=0.3, t_drive=1.0):
    """转向后沿朝向移动：速度按偏航角分解，位移方向应与偏航角一致。"""
    sim.reset()
    # 1) 先转向（姿态级）
    steps_turn = int(t_turn / sim.dt)
    for _ in range(steps_turn):
        sim.step(w_yaw=w)
    yaw = sim.read_odom()['yaw']
    pos0 = sim.read_odom()
    # 2) 再沿当前朝向"前进"
    steps_drive = int(t_drive / sim.dt)
    for _ in range(steps_drive):
        sim.step(v_lin=v)
    pos = sim.read_odom()
    dx = pos['x'] - pos0['x']
    dy = pos['y'] - pos0['y']
    moved = math.hypot(dx, dy)
    expected = v * t_drive
    assert abs(moved - expected) < 0.15, f"位移距离误差过大: {moved:.3f} vs {expected:.3f}"
    # 方向一致性：位移矢量与期望朝向 (cos yaw, sin yaw) 夹角 < 10°
    if moved > 1e-6:
        cos_ang = (dx * math.cos(yaw) + dy * math.sin(yaw)) / moved
        assert cos_ang > math.cos(math.radians(10)), \
            f"位移方向偏离朝向: cos夹角={cos_ang:.3f} (期望 > {math.cos(math.radians(10)):.3f})"
    assert pos['z'] > 0.9, "前进过程中倾倒"
    print(f"[PASS] 转向后沿朝向移动: yaw={math.degrees(yaw):.0f}° 位移 "
          f"({dx:.3f}, {dy:.3f}) m (距离 {moved:.3f} m，期望 ≈{expected:.3f} m)")


def test_turn(sim, w=0.5, t_turn=2.0):
    """姿态级转向：偏航角 ≈ w × t 且全程站立。"""
    sim.reset()
    yaw0 = sim.read_odom()['yaw']
    min_z = 9.0
    steps = int(t_turn / sim.dt)
    for _ in range(steps):
        sim.step(w_yaw=w)
        min_z = min(min_z, sim.read_odom()['z'])
    yaw = sim.read_odom()['yaw']
    dyaw = abs(yaw - yaw0)
    expected = w * t_turn
    assert abs(dyaw - expected) < 0.3, f"转向角误差过大: {math.degrees(dyaw):.1f}° vs {math.degrees(expected):.1f}°"
    assert min_z > 0.9, f"转向过程中倾倒 (min_z={min_z:.3f})"
    print(f"[PASS] 姿态级转向: {t_turn:.1f}s 转角 {math.degrees(dyaw):.1f}° "
          f"(期望 ≈{math.degrees(expected):.1f}°)，全程站立 (min_z={min_z:.3f} m)")


def test_render(sim):
    """相机渲染可用性。"""
    img = sim.render(camera='front_camera')
    assert img.shape == (480, 640, 3), f"渲染尺寸异常: {img.shape}"
    print(f"[PASS] 前视相机渲染: {img.shape[1]}x{img.shape[0]} RGB")


def test_performance(sim, seconds=1.0):
    """仿真性能（每秒物理步数）。"""
    sim.reset()
    start = time.time()
    count = 0
    while time.time() - start < seconds:
        sim.step()
        count += 1
    sps = count / (time.time() - start)
    print(f"[PASS] 仿真性能: {sps:.0f} step/s (实时因子 {sps * sim.dt:.2f}x)")
    assert sps > 5000, "仿真性能过低"


def main():
    print("=" * 56)
    print("  人形机器人 MuJoCo 仿真与运动控制自动化测试")
    print("=" * 56)
    sim = HumanoidSim()
    test_standing(sim)
    test_drive(sim)
    test_drive_heading(sim)
    test_turn(sim)
    test_render(sim)
    test_performance(sim)
    print("=" * 56)
    print("  全部测试通过 ✅")
    print("=" * 56)


if __name__ == '__main__':
    main()
