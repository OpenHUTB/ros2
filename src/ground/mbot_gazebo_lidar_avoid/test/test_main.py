#!/usr/bin/env python3
"""main.py 的离线单元测试。

不依赖 ROS 运行时，直接导入 LidarAvoidNode 的静态方法与 _decide/_valid_min 等
纯逻辑函数进行覆盖。运行：

    python3 test/test_main.py
"""

import math
import os
import sys

# 允许直接在仓库根目录或 test/ 目录下运行
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))   # .../mbot_gazebo_lidar_avoid/test
PKG_ROOT = os.path.dirname(SCRIPT_DIR)                     # .../mbot_gazebo_lidar_avoid
SCRIPTS_DIR = os.path.join(PKG_ROOT, 'scripts')            # .../mbot_gazebo_lidar_avoid/scripts
sys.path.insert(0, SCRIPTS_DIR)

# 只导入纯逻辑函数所需的依赖 —— main.py 模块本身会 import rospy，
# 所以这里用自定义 stub 把 rospy / geometry_msgs / sensor_msgs 先塞进 sys.modules。
import types

# ---- 最小 rospy stub，让 main.py 可以被 import ----
rospy_stub = types.ModuleType('rospy')
rospy_stub.init_node = lambda *a, **k: None
rospy_stub.Publisher = lambda *a, **k: types.SimpleNamespace(publish=lambda *a, **k: None)
rospy_stub.Subscriber = lambda *a, **k: None
rospy_stub.on_shutdown = lambda cb: None
rospy_stub.logwarn_throttle = lambda *a, **k: None
rospy_stub.Rate = lambda hz: types.SimpleNamespace(sleep=lambda: None)
rospy_stub.is_shutdown = lambda: False
rospy_stub.ROSInterruptException = type('ROSInterruptException', (Exception,), {})

gmsg_stub = types.ModuleType('geometry_msgs')
gmsg_stub_msg = types.ModuleType('geometry_msgs.msg')

class _Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

gmsg_stub_msg.Twist = _Twist
gmsg_stub.msg = gmsg_stub_msg

smsg_stub = types.ModuleType('sensor_msgs')
smsg_stub_msg = types.ModuleType('sensor_msgs.msg')

class _LaserScan:
    def __init__(self):
        self.ranges = None

smsg_stub_msg.LaserScan = _LaserScan
smsg_stub.msg = smsg_stub_msg

sys.modules['rospy'] = rospy_stub
sys.modules['geometry_msgs'] = gmsg_stub
sys.modules['geometry_msgs.msg'] = gmsg_stub_msg
sys.modules['sensor_msgs'] = smsg_stub
sys.modules['sensor_msgs.msg'] = smsg_stub_msg

from main import LidarAvoidNode, DANGER_DISTANCE, SAFE_DISTANCE  # noqa: E402


# ---- 测试 ----
def test_valid_min_normal():
    assert LidarAvoidNode._valid_min([1.0, 2.0, 3.0]) == 1.0
    assert LidarAvoidNode._valid_min([0.5]) == 0.5


def test_valid_min_filter_nan_inf():
    assert LidarAvoidNode._valid_min([float('nan'), 2.0, float('inf')]) == 2.0
    assert LidarAvoidNode._valid_min([-float('inf'), 0.3, float('nan')]) == 0.3


def test_valid_min_all_invalid():
    assert LidarAvoidNode._valid_min([float('nan'), float('inf'), -float('inf')]) is None
    assert LidarAvoidNode._valid_min([]) is None


def test_valid_min_clipped_to_range_max():
    # 内部用 min(..., RANGE_MAX_CLIP=3.5)
    assert LidarAvoidNode._valid_min([99.0, 100.0]) == 3.5


def test_decide_cruise():
    t = LidarAvoidNode._decide(None, front=1.0, left=2.0, right=2.0)
    assert t.linear.x > 0
    assert t.angular.z == 0.0


def test_decide_avoid_turn_left():
    t = LidarAvoidNode._decide(None, front=0.6, left=2.0, right=0.5)
    assert t.linear.x > 0
    assert t.angular.z > 0   # 左边宽，左转


def test_decide_avoid_turn_right():
    t = LidarAvoidNode._decide(None, front=0.6, left=0.5, right=2.0)
    assert t.linear.x > 0
    assert t.angular.z < 0   # 右边宽，右转


def test_decide_danger_stop():
    t = LidarAvoidNode._decide(None, front=0.1, left=2.0, right=2.0)
    assert t.linear.x == 0.0
    assert t.angular.z == 0.0


# ---- 汇总 ----
def main():
    tests = [
        ('test_valid_min_normal', test_valid_min_normal),
        ('test_valid_min_filter_nan_inf', test_valid_min_filter_nan_inf),
        ('test_valid_min_all_invalid', test_valid_min_all_invalid),
        ('test_valid_min_clipped_to_range_max', test_valid_min_clipped_to_range_max),
        ('test_decide_cruise', test_decide_cruise),
        ('test_decide_avoid_turn_left', test_decide_avoid_turn_left),
        ('test_decide_avoid_turn_right', test_decide_avoid_turn_right),
        ('test_decide_danger_stop', test_decide_danger_stop),
    ]

    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f'PASS  {name}')
        except AssertionError as e:
            failed += 1
            print(f'FAIL  {name}: {e}')

    print(f'\n结果: {len(tests) - failed}/{len(tests)} 通过')
    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
