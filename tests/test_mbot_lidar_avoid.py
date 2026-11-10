#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mbot 雷达避障模块的仓库级测试（桩化 rospy，不需要 ROS / Gazebo）.

覆盖范围：
A. 扇区换算：360 束 / angle_min=0；720 束 / angle_min=-π（不因束数不同而停车）
B. 扫描回调与发布链路：真实的 _scan_callback -> Publisher.publish
C. 异常输入：NaN/Inf 过滤、全 NaN、空 ranges、angle_increment 非法
D. 控制行为：滞回不抖动、脱困倒车、后方受阻原地旋转、侧向让开
E. 安全保护：/scan 超时停车（墙钟、与 /clock 解耦）、从未收到扫描停车、shutdown 停车
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
    SHUTDOWN_HOOKS.clear()
    node = module.LidarAvoidNode()
    node._publishers = PUBLISHERS
    node._subscribers = SUBSCRIBERS
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


def test_watchdog_is_wall_clock_thread():
    """watchdog 必须是墙钟线程：rospy.Timer 跟随 /clock，仿真暂停后就不再触发。"""
    with open(MAIN_PY, 'r', encoding='utf-8') as f:
        src = f.read()
    assert 'rospy.Timer(' not in src, 'watchdog 不能用 rospy.Timer（受 /clock 影响）'
    assert 'threading.Thread' in src, 'watchdog 应改为墙钟线程'

    node = make_node(control_rate=50.0)
    node.start()
    thread = node._watchdog_thread
    assert thread is not None, 'start() 未创建 watchdog 线程'
    assert thread.daemon, 'watchdog 线程应为 daemon'

    node._on_shutdown()
    thread.join(2.0)
    assert not thread.is_alive(), 'shutdown 后 watchdog 线程应退出'


def test_timeout_parks_without_sim_clock():
    """仿真时钟停走（时钟冻结）时，按墙钟仍要停车。"""
    node = make_node()
    node._last_scan_time = time.time() - 3.0

    # 只依赖 time.time()，不读 rospy.get_rostime()，因此 /clock 冻结不影响判断
    node._tick()
    twist = last_pub(node)
    assert twist.linear.x == 0.0
    assert twist.angular.z == 0.0


def test_state_machine_is_guarded_by_lock():
    """callback 与 watchdog 都会改 state，两条路径都必须持同一把锁。"""
    node = make_node()
    assert hasattr(node, '_state_lock'), '缺少保护 self.state 的锁'

    real = node._state_lock
    depth = {'enter': 0}

    class _Recording:
        def __enter__(self):
            depth['enter'] += 1
            return real.__enter__()

        def __exit__(self, *exc):
            return real.__exit__(*exc)

    node._state_lock = _Recording()

    feed(node, make_scan(360, 0.0, default=0.75))
    after_callback = depth['enter']
    assert after_callback >= 1, '扫描回调未在锁内更新状态'

    node._last_scan_time = time.time() - 3.0
    node._tick()
    assert depth['enter'] > after_callback, 'watchdog 未在锁内复位状态/停车'


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


def test_all_inf_means_open_space_and_keeps_cruising():
    """空旷场地雷达全是 inf（无回波）时不能判为“数据不可用”而停车。"""
    node = make_node()
    scan = make_scan(360, 0.0)
    scan.ranges = [float('inf')] * 360

    sec = node._sector_mins(scan)
    assert sec is not None
    assert sec['valid_count'] == 360
    assert abs(sec['front'] - node.cfg['range_max']) < 1e-9

    twist = feed(node, scan)
    assert node.state == 'cruise'
    assert twist.linear.x > 0.0


def test_negative_inf_still_filtered():
    """-inf 不是“无回波”，应按无效点丢弃。"""
    node = make_node()
    scan = make_scan(360, 0.0)
    scan.ranges = [float('-inf')] * 360

    twist = feed(node, scan)
    assert twist.linear.x == 0.0


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
def test_no_scan_yet_parks():
    node = make_node()
    assert node._last_scan_time is None

    node._tick()
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
    plugins = list(tree.getroot().iter('plugin'))

    # 插件必须用 filename（缺 filename 会让 Gazebo 解析 URDF->SDF 失败、spawn 不成功）
    filenames = []
    for plugin in plugins:
        filename = plugin.get('filename')
        assert filename, '插件 %r 缺少 filename' % plugin.get('name')
        filenames.append(filename)

    assert 'libgazebo_ros_diff_drive.so' in filenames
    assert 'libgazebo_ros_laser.so' in filenames


def test_urdf_diff_drive_uses_ros1_camel_case():
    """ROS 1 的 libgazebo_ros_diff_drive.so 只认驼峰参数，ROS 2 的 snake_case 会静默失败。"""
    tree = ET.parse(os.path.join(PKG, 'urdf', 'mbot_lidar.urdf'))
    diff = None
    for p in tree.getroot().iter('plugin'):
        if p.get('filename') == 'libgazebo_ros_diff_drive.so':
            diff = p
            break
    assert diff is not None, '找不到 diff_drive 插件'

    # 必须出现的 ROS 1 驼峰参数
    ros1_keys = {
        'leftJoint', 'rightJoint', 'wheelSeparation', 'wheelDiameter',
        'wheelTorque', 'wheelAcceleration', 'updateRate', 'odometrySource',
        'commandTopic', 'odometryTopic', 'odometryFrame', 'robotBaseFrame',
        'publishOdomTF', 'publishWheelTF', 'publishTf',
        'publishWheelJointState', 'rosDebugLevel',
    }
    for key in ros1_keys:
        assert diff.find(key) is not None, 'diff_drive 缺少 ROS 1 参数 <%s>' % key

    # 参数值必须真的写进 URDF，否则插件会退回默认值（扭矩 5 / 加速度 0）
    assert diff.find('wheelTorque').text == '10'
    assert diff.find('wheelAcceleration').text == '3.0'
    assert diff.find('updateRate').text == '100'
    assert diff.find('odometrySource').text == 'world'

    # 里程计以 base_footprint 为基座，轮子关节状态要发布，
    # 否则 /joint_states 无发布者、robot_state_publisher 发不出轮子 TF
    assert diff.find('robotBaseFrame').text == 'base_footprint'
    assert diff.find('publishWheelJointState').text == 'true'
    # 轮子 TF 已由 robot_state_publisher 根据 /joint_states 发布，
    # 插件自己再发一份就会重复，实测刷 TF_REPEATED_DATA
    assert diff.find('publishWheelTF').text == 'false'

    # 禁止出现的 ROS 2 snake_case 参数
    ros2_keys = {
        'left_joint', 'right_joint', 'wheel_separation', 'wheel_diameter',
        'command_topic', 'odometry_topic', 'odometry_frame', 'robot_base_frame',
        'publish_odom', 'publish_odom_tf', 'publish_wheel_tf',
        'max_wheel_torque', 'max_wheel_acceleration', 'update_rate',
        'odometry_source',
    }
    for key in ros2_keys:
        assert diff.find(key) is None, 'diff_drive 不应使用 ROS 2 参数 <%s>' % key

    # 这些不是 ROS 1 插件的合法参数，写进去会被静默忽略
    invalid_keys = {'maxWheelTorque', 'maxWheelAcceleration', 'publishOdom'}
    for key in invalid_keys:
        assert diff.find(key) is None, 'diff_drive 含无效参数 <%s>' % key


def test_urdf_has_base_footprint():
    """根链接应为 dummy base_footprint，避免 KDL 警告。"""
    tree = ET.parse(os.path.join(PKG, 'urdf', 'mbot_lidar.urdf'))
    links = [link.get('name') for link in tree.getroot().iter('link')]
    assert 'base_footprint' in links, '缺少 base_footprint 根链接'

    # 必须有一个 joint 把 base_footprint 连接到 base_link
    joints = list(tree.getroot().iter('joint'))
    found = None
    for j in joints:
        parent = j.find('parent')
        child = j.find('child')
        if (parent is not None and child is not None and
                parent.get('link') == 'base_footprint' and
                child.get('link') == 'base_link'):
            found = j
            break
    assert found is not None, '缺少 base_footprint -> base_link 的固定关节'

    # base_footprint 必须落在地面：轮子最低点在 base_link 下方 0.11 m，
    # 偏移为 0 会让 /odom 的 z 停在 0.11，接导航栈时多出 10 cm 偏置
    origin = found.find('origin')
    assert origin is not None, 'base_footprint -> base_link 关节缺少 origin'
    z = float(origin.get('xyz').split()[2])
    assert abs(z - 0.11) < 1e-9, '/odom 会带 %.2f m 的 z 偏置，应为 0.11' % z


def test_readme_does_not_build_repo_root():
    """README 不能推荐用仓库根目录直接 catkin_make（会把 ROS2/CARLA/MuJoCo 包一起编译）。"""
    with open(os.path.join(PKG, 'README.md'), 'r', encoding='utf-8') as f:
        readme = f.read()

    assert '不要直接用仓库根目录' in readme
    assert 'catkin_make --only-pkg-with-deps' in readme
    assert '~/mbot_ws' in readme


def test_readme_and_yaml_params_consistent():
    """README 参数表与 config/lidar_avoid.yaml 的键和默认值必须一一对应。"""
    import yaml

    with open(os.path.join(PKG, 'config', 'lidar_avoid.yaml'), 'r', encoding='utf-8') as f:
        yaml_cfg = yaml.safe_load(f)
    node_cfg = yaml_cfg['mbot_lidar_avoid_node']

    with open(os.path.join(PKG, 'README.md'), 'r', encoding='utf-8') as f:
        readme = f.read()

    # 从 README 参数表提取 "参数 | 默认值" 行
    import re
    pattern = re.compile(r'\|\s*`([a-z_]+)`\s*\|\s*([\d.\-]+)\s*\|')
    readme_params = {}
    for m in pattern.finditer(readme):
        key = m.group(1)
        value = float(m.group(2))
        readme_params[key] = value

    # 键集合一致
    yaml_keys = set(node_cfg.keys())
    readme_keys = set(readme_params.keys())
    assert yaml_keys == readme_keys, 'README 与 yaml 参数不一致: %s vs %s' % (
        sorted(readme_keys - yaml_keys), sorted(yaml_keys - readme_keys))

    # 默认值一致
    for key in yaml_keys:
        yv = float(node_cfg[key])
        rv = readme_params[key]
        assert abs(yv - rv) < 1e-9, '参数 %s 不一致: yaml=%.4f, README=%.4f' % (
            key, yv, rv)


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

    # 两个 gazebo 插件 .so 来自 gazebo_plugins 包，必须在依赖里声明
    deps = {e.text.strip() for e in root.findall('exec_depend')}
    assert 'gazebo_plugins' in deps, 'package.xml 缺少 gazebo_plugins 依赖'


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
