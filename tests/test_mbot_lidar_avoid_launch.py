#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mbot 雷达避障模块的仿真集成测试（需要 ROS Noetic + Gazebo）。

默认只做静态校验（无需 ROS）：
- 每个 ``<plugin>`` 必须带 ``filename`` 属性
  （历史上漏写 filename 导致 Gazebo 解析 URDF->SDF 失败、spawn 不成功）
- 有 ``check_urdf`` 时校验 URDF 结构
- 有 ``roslaunch`` 且包已 source 时，解析 launch 的节点列表

设置 ``MBOT_LAUNCH_TEST=1`` 时真正拉起 Gazebo（headless），
检查 ``/scan``、``/odom``、``/cmd_vel`` 是否有发布者并能收到数据，然后关闭。

用法:
    python3 -m pytest tests/test_mbot_lidar_avoid_launch.py -v
    MBOT_LAUNCH_TEST=1 python3 -m pytest tests/test_mbot_lidar_avoid_launch.py -v
    python3 tests/test_mbot_lidar_avoid_launch.py
"""
from __future__ import annotations

import os
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
PKG = os.path.join(ROOT, 'src', 'ground', 'mbot_gazebo_lidar_avoid')
URDF = os.path.join(PKG, 'urdf', 'mbot_lidar.urdf')
LAUNCH = os.path.join(PKG, 'launch', 'main.launch')


class _Skip(Exception):
    """standalone 运行时的跳过信号。"""


def skip(reason):
    # 在 pytest 下用 pytest.skip，直接运行时用自定义异常，避免 traceback
    if 'PYTEST_CURRENT_TEST' in os.environ:
        try:
            import pytest
            pytest.skip(reason)
            return
        except ImportError:
            pass
    raise _Skip(reason)


def _run(cmd, timeout=30):
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, timeout=timeout)


def _ros_available():
    return os.environ.get('ROS_DISTRO') is not None


# ---------------------------------------------------------------- 静态校验
def test_urdf_plugins_have_filename():
    """所有 gazebo 插件都必须声明 filename，否则 URDF->SDF 转换会报错。"""
    tree = ET.parse(URDF)
    plugins = list(tree.getroot().iter('plugin'))

    assert plugins, 'URDF 中没有任何 gazebo 插件'
    for plugin in plugins:
        filename = plugin.get('filename')
        assert filename, '插件 %r 缺少 filename 属性' % plugin.get('name')
        assert filename.endswith('.so'), '插件 filename 应为 .so：%r' % filename


def test_diff_drive_plugin_is_gazebo_ros_impl():
    tree = ET.parse(URDF)
    names = [p.get('filename') for p in tree.getroot().iter('plugin')]
    assert 'libgazebo_ros_diff_drive.so' in names
    assert 'libgazebo_ros_laser.so' in names
    # 不能出现旧式 type="gazebo_ros_diff_drive" 的写法
    for p in tree.getroot().iter('plugin'):
        assert p.get('type') is None, '插件应使用 filename 而不是 type 属性'


def test_launch_refers_to_existing_files():
    tree = ET.parse(LAUNCH)
    root = tree.getroot()
    text = ET.tostring(root, encoding='unicode')

    for rel in ('urdf/mbot_lidar.urdf', 'config/lidar_avoid.yaml',
                'worlds/lidar_avoid_world.world'):
        assert rel in text, 'launch 未引用 %s' % rel
        assert os.path.isfile(os.path.join(PKG, rel)), '缺少 %s' % rel


def test_urdf_valid_with_check_urdf():
    if shutil.which('check_urdf') is None:
        skip('未安装 check_urdf（ros-noetic-urdfdom）')
    result = _run(['check_urdf', URDF])
    assert result.returncode == 0, result.stdout
    assert 'Successfully Parsed XML' in result.stdout


def test_launch_nodes_resolvable():
    if shutil.which('roslaunch') is None or not _ros_available():
        skip('当前环境没有 roslaunch / 未 source ROS')
    result = _run(['roslaunch', '--nodes', 'mbot_gazebo_lidar_avoid',
                   'main.launch'], timeout=60)
    out = result.stdout
    if result.returncode != 0:
        skip('包未加入 ROS_PACKAGE_PATH：%s' % out.strip()[:200])
    assert 'spawn_mbot' in out
    assert 'mbot_lidar_avoid_node' in out
    assert 'robot_state_publisher' in out


# ---------------------------------------------------------------- 端到端
def _topic_exists(topic):
    result = _run(['rostopic', 'list'], timeout=15)
    return topic in result.stdout.split()


def _wait_topics(topics, timeout=90):
    deadline = time.time() + timeout
    missing = list(topics)
    while time.time() < deadline:
        missing = [t for t in topics if not _topic_exists(t)]
        if not missing:
            return []
        time.sleep(3)
    return missing


def test_gazebo_topics_end_to_end():
    """真实启动 Gazebo，确认机器人被生成且雷达/里程计有数据。"""
    if os.environ.get('MBOT_LAUNCH_TEST') != '1':
        skip('未设置 MBOT_LAUNCH_TEST=1，跳过真实仿真')
    if shutil.which('roslaunch') is None or not _ros_available():
        skip('当前环境没有 roslaunch / 未 source ROS')

    kwargs = {}
    if os.name != 'nt':
        kwargs['start_new_session'] = True

    proc = subprocess.Popen(
        ['roslaunch', 'mbot_gazebo_lidar_avoid', 'main.launch', 'gui:=false'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, **kwargs)

    try:
        missing = _wait_topics(['/scan', '/odom', '/cmd_vel'], timeout=120)
        assert not missing, '以下话题无发布者：%s' % ', '.join(missing)

        scan = _run(['rostopic', 'echo', '-n', '1', '/scan'], timeout=30)
        assert scan.returncode == 0, '未收到 /scan 数据'
        assert 'frame_id' in scan.stdout

        odom = _run(['rostopic', 'echo', '-n', '1', '/odom'], timeout=30)
        assert odom.returncode == 0, '未收到 /odom 数据'

        vel = _run(['rostopic', 'echo', '-n', '1', '/cmd_vel'], timeout=30)
        assert vel.returncode == 0, '未收到 /cmd_vel 数据'
    finally:
        if os.name != 'nt' and proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        else:
            proc.terminate()
        try:
            proc.wait(timeout=25)
        except subprocess.TimeoutExpired:
            proc.kill()
        if os.name != 'nt':
            _run(['pkill', '-f', 'gzserver'], timeout=10)


# ---------------------------------------------------------------- 直接运行
def _run_standalone():
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith('test_') and callable(o)]
    failed = 0
    skipped = 0
    for name, fn in tests:
        try:
            fn()
            print('PASS  %s' % name)
        except _Skip as exc:
            skipped += 1
            print('SKIP  %s: %s' % (name, exc))
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print('FAIL  %s: %s' % (name, exc))
    print('\n结果: %d 通过, %d 跳过, %d 失败'
          % (len(tests) - failed - skipped, skipped, failed))
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(_run_standalone())
