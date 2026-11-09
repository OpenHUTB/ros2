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


def _run(cmd, timeout=30, env=None):
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, timeout=timeout, env=env)


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
def _topic_exists(topic, env=None):
    result = _run(['rostopic', 'list'], timeout=15, env=env)
    return topic in result.stdout.split()


def _wait_topics(topics, timeout=90, env=None):
    deadline = time.time() + timeout
    missing = list(topics)
    while time.time() < deadline:
        missing = [t for t in topics if not _topic_exists(t, env=env)]
        if not missing:
            return []
        time.sleep(3)
    return missing


def _free_port():
    """找一个空闲端口，给测试专用的 rosmaster / gzserver 用，避免撞上已有的 ROS/Gazebo。"""
    import socket
    sock = socket.socket()
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def _stop_process_tree(proc):
    """只清理本测试拉起的进程组，不要 pkill 同机其他 Gazebo。"""
    if proc.poll() is None:
        if os.name != 'nt':
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
        else:
            proc.terminate()
    try:
        proc.wait(timeout=30)
        return
    except subprocess.TimeoutExpired:
        pass
    if os.name != 'nt':
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    else:
        proc.kill()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass


def test_gazebo_topics_end_to_end():
    """真实启动 Gazebo，确认机器人被生成、/odom 有数据，且发 cmd_vel 后位移增长。

    用独立的 ROS_MASTER_URI / GAZEBO_MASTER_URI 跑，互不干扰：
    同机上已有的 rosmaster、Gazebo 不会被影响，测试也不会被它们的同名话题带偏。
    """
    if os.environ.get('MBOT_LAUNCH_TEST') != '1':
        skip('未设置 MBOT_LAUNCH_TEST=1，跳过真实仿真')
    if shutil.which('roslaunch') is None or not _ros_available():
        skip('当前环境没有 roslaunch / 未 source ROS')

    env = dict(os.environ)
    env['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % _free_port()
    env['GAZEBO_MASTER_URI'] = 'http://127.0.0.1:%d' % _free_port()

    kwargs = {'env': env}
    if os.name != 'nt':
        # 独立进程组：结束后整组回收，不会误杀同机其他 Gazebo
        kwargs['start_new_session'] = True

    proc = subprocess.Popen(
        ['roslaunch', 'mbot_gazebo_lidar_avoid', 'main.launch', 'gui:=false'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, **kwargs)

    try:
        missing = _wait_topics(['/scan', '/odom', '/cmd_vel'], timeout=180, env=env)
        assert not missing, '以下话题无发布者：%s' % ', '.join(missing)

        # 确认 /odom 确实有数据（不是空话题）
        odom0 = _run(['rostopic', 'echo', '-n', '1', '/odom'], timeout=30, env=env)
        assert odom0.returncode == 0, '未收到 /odom 数据'

        scan = _run(['rostopic', 'echo', '-n', '1', '/scan'], timeout=30, env=env)
        assert scan.returncode == 0, '未收到 /scan 数据'
        assert 'frame_id' in scan.stdout

        # 发 cmd_vel 让机器人直行，等待 3 秒
        pub = subprocess.Popen(
            ['rostopic', 'pub', '-r', '10', '/cmd_vel',
             'geometry_msgs/Twist',
             '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        try:
            time.sleep(3)
        finally:
            pub.terminate()
            pub.wait(timeout=5)

        # 再取一次 /odom，检查位移
        odom1 = _run(['rostopic', 'echo', '-n', '1', '/odom'], timeout=30, env=env)
        assert odom1.returncode == 0, '未收到第二次 /odom 数据'

        # 解析两次 odom 的 pose.position.x，确认位移增长
        import re
        def _extract_x(text):
            m = re.search(r'x:\s*([-+]?\d*\.?\d+)', text)
            return float(m.group(1)) if m else None

        x0 = _extract_x(odom0.stdout)
        x1 = _extract_x(odom1.stdout)
        assert x0 is not None and x1 is not None, '无法从 /odom 解析 x'
        moved = abs(x1 - x0)
        assert moved > 0.01, '发 cmd_vel 后位移仅 %.4f m，说明 /odom 或驱动插件异常' % moved

        vel = _run(['rostopic', 'echo', '-n', '1', '/cmd_vel'], timeout=30, env=env)
        assert vel.returncode == 0, '未收到 /cmd_vel 数据'
    finally:
        _stop_process_tree(proc)


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
