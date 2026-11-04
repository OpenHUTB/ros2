#!/usr/bin/env python3
"""任务①测试：MuJoCo 差速车物理仿真、键盘映射和基础运动。"""

import math
import os
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from mujoco_auto_parking.keyboard_teleop_node import (  # noqa: E402
    command_for_key,
    is_quit_key,
)
from mujoco_auto_parking.kinematics import CommandWatchdog  # noqa: E402
from mujoco_auto_parking.simulator import MujocoSimulation  # noqa: E402


MODEL_PATH = PACKAGE_ROOT / 'models' / 'parking_lot.xml'


def check_model_objects() -> None:
    simulation = MujocoSimulation(MODEL_PATH)
    for name in ('chassis', 'chassis_geom'):
        assert simulation.has_object(
            mujoco.mjtObj.mjOBJ_BODY if name == 'chassis'
            else mujoco.mjtObj.mjOBJ_GEOM,
            name,
        ), f'缺少对象 {name}'
    for name in MujocoSimulation.ACTUATOR_NAMES:
        assert simulation.has_object(mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    for name in ('front_camera', 'overview_camera'):
        assert simulation.has_object(mujoco.mjtObj.mjOBJ_CAMERA, name)
    for name in ('imu_site', 'lidar_site', 'target_slot'):
        assert simulation.has_object(mujoco.mjtObj.mjOBJ_SITE, name)
    print('[PASS] 模型对象：差速车、四轮执行器、相机、激光雷达安装点、目标车位')


def check_forward_motion() -> None:
    simulation = MujocoSimulation(MODEL_PATH)
    simulation.reset()
    start = simulation.planar_state()
    simulation.apply_wheel_speeds(4.0, 4.0)
    for _ in range(100):
        simulation.step()
    end = simulation.planar_state()
    assert end.x - start.x > 0.05, (start.x, end.x)
    assert abs(end.y - start.y) < 0.03, (start.y, end.y)
    assert abs(end.yaw) < 0.05
    print(f'[PASS] 前进：x {start.x:.3f} -> {end.x:.3f} m')


def check_backward_motion() -> None:
    simulation = MujocoSimulation(MODEL_PATH)
    simulation.reset()
    start = simulation.planar_state()
    simulation.apply_wheel_speeds(-4.0, -4.0)
    for _ in range(100):
        simulation.step()
    end = simulation.planar_state()
    assert end.x - start.x < -0.05, (start.x, end.x)
    print(f'[PASS] 后退：x {start.x:.3f} -> {end.x:.3f} m')


def check_turn_motion() -> None:
    simulation = MujocoSimulation(MODEL_PATH)
    simulation.reset()
    simulation.apply_wheel_speeds(-3.0, 3.0)
    for _ in range(100):
        simulation.step()
    yaw = simulation.planar_state().yaw
    assert yaw > 0.1, yaw
    simulation.apply_wheel_speeds(3.0, -3.0)
    for _ in range(100):
        simulation.step()
    yaw_after = simulation.planar_state().yaw
    assert yaw_after < yaw, (yaw, yaw_after)
    print(f'[PASS] 转向：yaw {yaw:.3f} -> {yaw_after:.3f} rad')


def check_pose_reset() -> None:
    simulation = MujocoSimulation(MODEL_PATH)
    simulation.set_pose(1.0, 2.0, math.pi / 2.0)
    state = simulation.planar_state()
    assert abs(state.x - 1.0) < 1e-6
    assert abs(state.y - 2.0) < 1e-6
    assert abs(state.yaw - math.pi / 2.0) < 1e-6
    print('[PASS] set_pose：位置和航向可精确设置')


def check_keyboard_mapping() -> None:
    linear, angular = 0.35, 0.9
    assert command_for_key('w', linear, angular) == (linear, 0.0)
    assert command_for_key('s', linear, angular) == (-linear, 0.0)
    assert command_for_key('a', linear, angular) == (0.0, angular)
    assert command_for_key('d', linear, angular) == (0.0, -angular)
    assert command_for_key('KEY_UP', linear, angular) == (linear, 0.0)
    assert command_for_key('KEY_LEFT', linear, angular) == (0.0, angular)
    assert command_for_key(' ', linear, angular) == (0.0, 0.0)
    assert command_for_key('x', linear, angular) is None
    assert is_quit_key('q')
    assert is_quit_key('ESC')
    assert not is_quit_key('w')
    print('[PASS] 键盘映射：W/S/A/D/方向键/空格/退出键')


def check_watchdog() -> None:
    now = [10.0]
    watchdog = CommandWatchdog(0.5, clock=lambda: now[0])
    watchdog.update(0.3, 0.1)
    assert watchdog.current() == (0.3, 0.1)
    now[0] = 10.51
    assert watchdog.current() == (0.0, 0.0)
    print('[PASS] Watchdog：超时后速度归零')


def main() -> None:
    os.environ.setdefault('MUJOCO_GL', 'osmesa')
    check_model_objects()
    check_forward_motion()
    check_backward_motion()
    check_turn_motion()
    check_pose_reset()
    check_keyboard_mapping()
    check_watchdog()
    print('\n任务①测试全部通过。')


if __name__ == '__main__':
    main()
