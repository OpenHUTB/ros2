#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_env.py - 运行环境自检脚本

提交运行效果图前，先运行本脚本确认环境就绪:

    python3 check_env.py            # 基础检查: Python 版本 / ROS2 / airsim 模块
    python3 check_env.py --sim      # 额外检查: 连接 OpenHUTB 模拟器并读取无人机状态

全部通过后，按 docs/air/hutb_drone_keyboard_control.md 第 8 节的清单录制运行效果图。
"""

import argparse
import os
import sys

OK = '[OK]  '
FAIL = '[FAIL]'
WARN = '[WARN]'


def check_python():
    v = sys.version_info
    print(f'{OK if (v.major, v.minor) >= (3, 10) else FAIL} '
          f'Python 版本: {v.major}.{v.minor}.{v.micro} (Humble 要求 >= 3.10)')
    return (v.major, v.minor) >= (3, 10)


def check_ros2():
    distro = os.environ.get('ROS_DISTRO', '')
    if distro:
        print(f'{OK if distro == "humble" else WARN} ROS_DISTRO={distro}'
              + ('' if distro == 'humble' else ' (本模块按 Humble 开发)'))
        try:
            import rclpy  # noqa: F401
            print(f'{OK} rclpy 可导入')
            return True
        except ImportError as e:
            print(f'{FAIL} rclpy 导入失败: {e}')
            return False
    print(f'{FAIL} 未检测到 ROS 2 环境, 请先 source /opt/ros/humble/setup.bash')
    return False


def check_airsim():
    try:
        import airsim  # noqa: F401
        print(f'{OK} airsim 模块可导入 (OpenHUTB Python 客户端已安装)')
        return True
    except ImportError:
        print(f'{FAIL} airsim 模块未安装, 请执行: pip install hutb-*.whl')
        return False


def check_simulator(host: str, port: int):
    try:
        import airsim
        client = airsim.MultirotorClient(ip=host, port=port)
        client.confirmConnection()
        state = client.getMultirotorState()
        p = state.kinematics_estimated.position
        print(f'{OK} 已连接模拟器 {host}:{port}')
        print(f'      无人机位置 (NED): x={p.x_val:.2f}, y={p.y_val:.2f}, z={p.z_val:.2f}')
        print(f'      状态: {"已解锁" if state.landed_state == airsim.LandedState.Landed else "空中"}')
        return True
    except Exception as e:  # noqa: BLE001
        print(f'{FAIL} 无法连接模拟器 {host}:{port}: {e}')
        print('      请确认已用 AIR 模式启动: python config.py --map Town10HD?GAME=AIR')
        return False


def main():
    parser = argparse.ArgumentParser(description='hutb_drone_keyboard_control 环境自检')
    parser.add_argument('--sim', action='store_true', help='额外检查模拟器连接')
    parser.add_argument('--host', default='127.0.0.1', help='模拟器地址')
    parser.add_argument('--port', type=int, default=41451, help='AirSim RPC 端口')
    args = parser.parse_args()

    print('=' * 52)
    print('  hutb_drone_keyboard_control 环境自检')
    print('=' * 52)

    results = [check_python(), check_ros2(), check_airsim()]
    if args.sim:
        results.append(check_simulator(args.host, args.port))

    passed = sum(results)
    total = len(results)
    print('-' * 52)
    print(f'检查完成: {passed}/{total} 通过'
          + ('  ✓ 环境就绪, 可以开始运行与录屏' if passed == total else '  ✗ 请先修复 FAIL 项'))
    sys.exit(0 if passed == total else 1)


if __name__ == '__main__':
    main()
