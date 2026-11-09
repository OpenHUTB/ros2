#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mbot 雷达避障模块的仓库级测试（桩化 rospy，不需要 ROS / Gazebo）.

覆盖范围：
A. 扇区换算：360 束 / angle_min=0；720 束 / angle_min=-π（不因束数不同而停车）
B. 扫描回调与发布链路：真实的 _scan_callback -> Publisher.publish
C. 异常输入：NaN/Inf 过滤、全 NaN、空 ranges、angle_increment 非法
D. 控制行为：滞回不抖动、脱困倒车、后方受阻原地旋转、侧向让开
E. 安全保护：/scan 超时停车、从未收到扫描停车、shutdown 停车
F. 工程完整性：launch 内容、CMakeLists 安装规则、package.xml 与 LICENSE

用法:
    python3 -m pytest tests/test_mbot_lidar_avoid.py -v
    python3 tests/test_mbot_lidar_avoid.py
"""
from __future__ import annotations

import importlib.util
import math
import os
import sys
import time
import types
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
PKG = os.path.join(ROOT, 'src', 'ground', 'mbot_gazebo_lidar_avoid')
SCRIPTS = os.path.join(PKG, 'scripts')
MAIN_PY = os.path.join(SCRIPTS, 'main.py')

# 测试可改写的参数覆盖表
PARAMS = {}
PUBLISHERS = []
SUBSCRIBERS = []
TIMERS = []
SHUTDOWN_HOOKS = []


# ------------------------------------------------------------------ 消息桩
class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class LaserScan:
    def __init__(self, ranges=None, angle_min=0.0, angle_increment=0.0,
                 range_min=0.1, range_max=3.5):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_max = 2.0 * math.pi
        self.angle_increment = angle_increment
        self.range_min = range_min
        self.range_max = range_max


# ------------------------------------------------------------------ rospy 桩
class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)

    @property
    def last(self):
        return self.messages[-1] if self.messages else None


def _install_stubs():
    rospy = types.ModuleType('rospy')
    rospy.init_node = lambda *a, **k: None
    rospy.spin = lambda *a, **k: None
    rospy.is_shutdown = lambda: False
    rospy.Duration = lambda secs: secs
    rospy.loginfo = lambda *a, **k: None
    rospy.logwarn_throttle = lambda *a, **k: None
    rospy.ROSInterruptException = type('ROSInterruptException', (Exception,), {})

    def _get_param(name, default=None):
        key = name.lstrip('~')
        return PARAMS.get(key, default)

    rospy.get_param = _get_param

    def _publisher(*a, **k):
        pub = _Publisher()
        PUBLISHERS.append(pub)
        return pub

    rospy.Publisher = _publisher

    def _subscriber(*a, **k):
        callback = a[2] if len(a) > 2 else k.get('callback')
        SUBSCRIBERS.append(callback)
        return types.SimpleNamespace(callback=callback)

    rospy.Subscriber = _subscriber

    def _timer(period, callback, *a, **k):
        TIMERS.append(callback)
        return types.SimpleNamespace(callback=callback, period=period)

    rospy.Timer = _timer

    def _on_shutdown(cb):
        SHUTDOWN_HOOKS.append(cb)

    rospy.on_shutdown = _on_shutdown

    geo_msg = types.ModuleType('geometry_msgs.msg')
    geo_msg.Twist = Twist
    geo_pkg = types.ModuleType('geometry_msgs')
    geo_pkg.msg = geo_msg

    sen_msg = types.ModuleType('sensor_msgs.msg')
    sen_msg.LaserScan = LaserScan
    sen_pkg = types.ModuleType('sensor_msgs')
    sen_pkg.msg = sen_msg

    sys.modules['rospy'] = rospy
    sys.modules['geometry_msgs'] = geo_pkg
    sys.modules['geometry_msgs.msg'] = geo_msg
    sys.modules['sensor_msgs'] = sen_pkg
    sys.modules['sensor_msgs.msg'] = sen_msg


def load_main():
    """导入被测模块（只导入一次）。"""
    if 'mbot_main_under_test' in sys.modules:
        return sys.modules['mbot_main_under_test']
    _install_stubs()
    spec = importlib.util.spec_from_file_location(
        'mbot_main_under_test', MAIN_PY)
    module = importlib.util.module_from_spec(spec)
    sys.modules['mbot_main_under_test'] = module
    spec.loader.exec_module(module)
    return module


def make_node(**overrides):
    """创建节点实例，可覆盖任意参数。"""
    module = load_main()
    PARAMS.clear()
    PARAMS.update({k: float(v) for k, v in overrides.items()})
    PUBLISHERS.clear()
    SUBSCRIBERS.clear()
    TIMERS.clear()
    SHUTDOWN_HOOKS.clear()
    node = module.LidarAvoidNode()
    node._publishers = PUBLISHERS
    node._subscribers = SUBSCRIBERS
    node._timers = TIMERS
    node._shutdown_hooks = SHUTDOWN_HOOKS
    return node


def make_scan(n=360, angle_min=0.0, default=3.0, overrides=None):
    """构造 LaserScan；overrides 为 {索引: 距离}。"""
    inc = 2.0 * math.pi / n
    ranges = [default] * n
    for idx, value in (overrides or {}).items():
        ranges[idx] = value
    return LaserScan(ranges=ranges, angle_min=angle_min, angle_increment=inc)


def last_pub(node):
    assert node._publishers, '未创建 Publisher'
    return node._publishers[0].last


def feed(node, scan):
    """直接驱动真实回调。"""
    node._subscribers[0](scan)
    return last_pub(node)


# ------------------------------------------------------------------ A. 扇区换算
def test_sector_360_angle_min_zero():
    node = make_node()
    sec = node._sector_mins(make_scan(360, 0.0, overrides={0: 0.5}))

    assert sec is not None
    assert abs(sec['front'] - 0.5) < 1e-9
    assert sec['valid_count'] == 360


def test_sector_720_angle_min_minus_pi_does_not_park():
    """720 束且 angle_min=-π 时不能触发“异常长度停车”。"""
    node = make_node()
    # 720 束：索引 360 对应 0 rad
    scan = make_scan(720, -math.pi, overrides={360: 0.5})
    sec = node._sector_mins(scan)

    assert sec is not None
    assert abs(sec['front'] - 0.5) < 1e-9

    twist = feed(node, scan)
    assert node.state == 'avoid'
    assert twist.linear.x > 0.0


def test_sector_left_right_not_swapped_at_minus_pi():
    """angle_min=-π 时左右扇区不能错位。"""
    node = make_node()
    # 索引 420：角度 = -π + 420*(2π/720) = +30° -> 左侧
    # 索引 300：角度 = -π + 300*(2π/720) = -30° -> 右侧
    scan = make_scan(720, -math.pi, overrides={420: 0.20, 300: 0.20})
    sec = node._sector_mins(scan)

    assert abs(sec['left'] - 0.20) < 1e-9
    assert abs(sec['right'] - 0.20) < 1e-9
    assert sec['front'] >= 3.0


# ------------------------------------------------------------------ B. 回调链路
def test_callback_publishes_to_publisher():
    node = make_node()
    twist = feed(node, make_scan(360, 0.0))

    assert len(node._publishers[0].messages) == 1
    assert twist.linear.x > 0.0


def test_timer_watchdog_registered():
    node = make_node()
    assert node._timers, '未注册 watchdog Timer'
    assert node._shutdown_hooks, '未注册 shutdown 回调'


# ------------------------------------------------------------------ C. 异常输入
def test_nan_inf_filtered():
    node = make_node()
    scan = make_scan(360, 0.0)
    scan.ranges = [float('nan')] * 360
    scan.ranges[0] = float('inf')
    scan.ranges[5] = 0.6

    twist = feed(node, scan)
    assert node.state == 'avoid'
    assert twist.linear.x > 0.0


def test_all_nan_parks():
    node = make_node()
    scan = make_scan(360, 0.0)
    scan.ranges = [float('nan')] * 360

    twist = feed(node, scan)
    assert twist.linear.x == 0.0
    assert twist.angular.z == 0.0
    assert node.state == 'cruise'


def test_empty_ranges_parks():
    node = make_node()
    twist = feed(node, LaserScan(ranges=[], angle_min=0.0,
                                angle_increment=0.01745))
    assert twist.linear.x == 0.0
    assert twist.angular.z == 0.0


def test_invalid_angle_increment_parks():
    node = make_node()
    twist = feed(node, LaserScan(ranges=[1.0] * 360, angle_min=0.0,
                                angle_increment=0.0))
    assert twist.linear.x == 0.0


# ------------------------------------------------------------------ D. 控制行为
def test_hysteresis_does_not_chatter():
    node = make_node()
    # 0.75 < safe(0.80) -> 进入避障
    t1 = feed(node, make_scan(360, 0.0, default=0.75))
    assert node.state == 'avoid'
    assert t1.angular.z != 0.0

    # 0.90 介于 safe(0.80) 与 exit(1.00) 之间 -> 保持避障，不来回切换
    t2 = feed(node, make_scan(360, 0.0, default=0.90))
    assert node.state == 'avoid'
    assert t2.angular.z != 0.0

    # 1.10 > exit(1.00) -> 回到巡航
    t3 = feed(node, make_scan(360, 0.0, default=1.10))
    assert node.state == 'cruise'
    assert t3.angular.z == 0.0


def test_danger_triggers_reverse():
    """紧急时应有倒车分支，而不是零速度停死。"""
    node = make_node()
    # 只有正前方近（0.20 m），侧后方空旷 -> 可以倒车
    twist = feed(node, make_scan(360, 0.0, overrides={0: 0.20}))

    assert node.state == 'escape'
    assert twist.linear.x < 0.0


def test_rear_blocked_rotates_in_place():
    node = make_node()
    # 前方 0.20 -> 脱困；后方两个扇区也近 -> 不能倒车，改原地旋转
    scan = make_scan(360, 0.0, default=3.0,
                     overrides={0: 0.20, 120: 0.20, 240: 0.20})
    twist = feed(node, scan)

    assert node.state == 'escape'
    assert twist.linear.x == 0.0
    assert abs(twist.angular.z) > 0.0


def test_side_obstacle_steers_away():
    """侧向障碍也要参与决策，不能只管 ±15°。"""
    node = make_node()
    # 左侧 30° 处 0.20 m，前方与右侧空旷
    scan = make_scan(360, 0.0, overrides={30: 0.20})
    twist = feed(node, scan)

    assert node.state == 'cruise'
    assert twist.angular.z < 0.0   # 向右让开


# ------------------------------------------------------------------ E. 安全保护
def test_scan_timeout_parks():
    node = make_node()
    node._last_scan_time = time.time() - 3.0

    node._timers[0]()
    twist = last_pub(node)
    assert twist.linear.x == 0.0
    assert twist.angular.z == 0.0


def test_no_scan_yet_parks():
    node = make_node()
    assert node._last_scan_time is None

    node._timers[0]()
    twist = last_pub(node)
    assert twist.linear.x == 0.0


def test_shutdown_parks():
    node = make_node()
    node._shutdown_hooks[0]()

    twist = last_pub(node)
    assert twist.linear.x == 0.0
    assert twist.angular.z == 0.0


# ------------------------------------------------------------------ F. 工程完整性
def test_required_assets_exist():
    for rel in ('README.md', 'LICENSE', 'CMakeLists.txt', 'package.xml',
                'config/lidar_avoid.yaml', 'urdf/mbot_lidar.urdf',
                'worlds/lidar_avoid_world.world', 'launch/main.launch',
                'scripts/main.py'):
        assert os.path.isfile(os.path.join(PKG, rel)), f'缺少 {rel}'


def test_main_py_is_executable_script():
    with open(MAIN_PY, 'r', encoding='utf-8') as f:
        first = f.readline().strip()
    assert first.startswith('#!'), 'main.py 缺少 shebang'


def test_launch_spawns_mbot_and_loads_params():
    tree = ET.parse(os.path.join(PKG, 'launch', 'main.launch'))
    text = ET.tostring(tree.getroot(), encoding='unicode')

    assert 'robot_description' in text
    assert 'spawn_model' in text
    assert 'lidar_avoid.yaml' in text
    assert 'mbot_lidar.urdf' in text
    assert 'turtlebot3' not in text, 'launch 不应再依赖 turtlebot3'


def test_urdf_has_diff_drive_and_laser():
    tree = ET.parse(os.path.join(PKG, 'urdf', 'mbot_lidar.urdf'))
    text = ET.tostring(tree.getroot(), encoding='unicode')

    assert 'gazebo_ros_diff_drive' in text
    assert 'libgazebo_ros_laser.so' in text


def test_cmakelists_installs_launch_and_config():
    with open(os.path.join(PKG, 'CMakeLists.txt'), 'r', encoding='utf-8') as f:
        content = f.read()

    assert 'catkin_install_python' in content
    assert 'install(DIRECTORY' in content
    for name in ('launch', 'config', 'urdf', 'worlds'):
        assert name in content, f'CMakeLists 未安装 {name}'


def test_package_xml_metadata_complete():
    tree = ET.parse(os.path.join(PKG, 'package.xml'))
    root = tree.getroot()

    assert root.findtext('license') == 'MIT'

    maintainer = root.find('maintainer')
    assert maintainer is not None
    email = maintainer.get('email', '')
    assert 'todo' not in email.lower()
    assert '@' in email

    assert os.path.isfile(os.path.join(PKG, 'LICENSE')), '声明 MIT 但缺少 LICENSE'


def test_parameters_come_from_rosparam():
    """同名参数通过 rosparam 覆盖后应生效，验证没有硬编码残留。"""
    node = make_node(cruise_speed=0.55, safe_distance=1.20)

    assert abs(node.cfg['cruise_speed'] - 0.55) < 1e-9
    assert abs(node.cfg['safe_distance'] - 1.20) < 1e-9

    twist = feed(node, make_scan(360, 0.0, default=1.5))
    assert abs(twist.linear.x - 0.55) < 1e-9


# ------------------------------------------------------------------ 直接运行
def _run_standalone():
    tests = [(name, obj) for name, obj in sorted(globals().items())
             if name.startswith('test_') and callable(obj)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print('PASS  %s' % name)
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print('FAIL  %s: %s' % (name, exc))
    print('\n结果: %d/%d 通过' % (len(tests) - failed, len(tests)))
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(_run_standalone())
