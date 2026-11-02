# -*- coding: utf-8 -*-
"""ROS 侧测试：用 test/fake_ros.py 的严格替身把三个节点真接起来跑。

替身只在**没有真实 rclpy** 时启用；装了 ROS 2 的机器上这些用例会整体跳过，
改由 `.github/workflows/ros2_ugv_visual_slam_nav.yml` 在 ros:humble 容器里
用真的 `ros2 launch` 验证。
"""

import importlib.util
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import fake_ros  # noqa: E402


def _install(share_root=None):
    try:
        fake_ros.install(share_root=share_root)
    except RuntimeError:
        pytest.skip('本机已安装真实 rclpy，请用 ros2 launch 做端到端验证')


@pytest.fixture()
def ros():
    _install()
    fake_ros.reset()
    yield fake_ros
    fake_ros.reset()


def _close():
    import rclpy
    try:
        rclpy.shutdown()
    except Exception:
        pass


# ------------------------------------------------------------------ 仿真节点
def test_sim_node_publishes_sensor_topics(ros):
    ros.set_param_overrides({'ugv_sim': {'scene': 'corridor', 'seed': 0}})
    import rclpy
    rclpy.init()
    from ugv_visual_slam_nav.sim_node import SimNode

    node = SimNode()
    assert node.unused_overrides() == [], 'launch 传了但节点没声明的参数'
    rig = ros.Rig(dt=0.05)
    rig.run(30)

    assert node.published_on('/ugv/odom') >= 3
    assert node.published_on('/ugv/camera/image_raw') >= 3
    assert node.published_on('/ugv/camera/depth') >= 3
    assert node.published_on('/ugv/ground_truth') >= 3
    assert node.steps >= 3
    _close()


# --------------------------------------------------------------- 三节点闭环
def test_slam_and_nav_closed_loop(ros):
    ros.set_param_overrides({'ugv_sim': {'scene': 'corridor', 'seed': 0},
                             'ugv_slam': {'start_x': 2.0, 'start_y': 2.0},
                             'ugv_nav': {'goal_x': 17.6, 'goal_y': 11.6}})
    import rclpy
    from geometry_msgs.msg import Twist
    rclpy.init()

    from ugv_visual_slam_nav.nav_node import NavNode
    from ugv_visual_slam_nav.sim_node import SimNode
    from ugv_visual_slam_nav.slam_node import SlamNode

    class Sink(rclpy.node.Node):
        def __init__(self):
            super().__init__('cmd_sink')
            self.cmds = []
            self.create_subscription(Twist, '/ugv/cmd_vel',
                                     lambda m: self.cmds.append(m), 10)

    sim = SimNode()
    slam = SlamNode()
    nav = NavNode()
    sink = Sink()

    rig = ros.Rig(dt=0.05)
    rig.run(300)          # 15 s 仿真

    assert sim.published_on('/ugv/odom') >= 50
    assert slam.published_on('/ugv/map') >= 3, 'SLAM 应周期性发布栅格'
    assert slam.n_frames > 50, 'SLAM 应该吃到了图像'
    assert len(slam.slam.ids) >= 1, '应该至少关联出一个路标'
    assert nav.published_on('/ugv/cmd_vel') >= 100
    assert sink.cmds, '导航节点应发布过速度指令'
    assert max(m.linear.x for m in sink.cmds) > 0.05, '速度指令不能一直是 0'
    _close()


def test_topic_type_mismatch_is_rejected(ros):
    """替身应像真 ROS 一样拒绝话题类型不一致。"""
    import rclpy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Image
    rclpy.init()

    class A(rclpy.node.Node):
        def __init__(self):
            super().__init__('a')
            self.pub = self.create_publisher(Image, '/ugv/cmd_vel', 10)

    class B(rclpy.node.Node):
        def __init__(self):
            super().__init__('b')
            self.create_subscription(Twist, '/ugv/cmd_vel', lambda m: None, 10)

    a, b = A(), B()
    with pytest.raises(TypeError):
        a.pub.publish(Image())
    _close()


# ------------------------------------------------------------------ launch
def _load_launch(name):
    path = os.path.join(PKG_ROOT, 'launch', name)
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_main_launch_declares_three_nodes(ros):
    mod = _load_launch('main.launch.py')
    desc = mod.generate_launch_description()
    ctx, nodes = ros.expand_launch(desc, {'scene': 'cluttered', 'seed': '2'})

    assert ctx['scene'] == 'cluttered'
    assert len(nodes) == 3
    names = {n.kwargs['name'] for n in nodes}
    assert names == {'ugv_sim', 'ugv_slam', 'ugv_nav'}
    params = {n.kwargs['name']: n.kwargs['parameters'] for n in nodes}
    assert params['ugv_slam'][0]['occ_threshold'] == pytest.approx(0.62)
    assert params['ugv_nav'][0]['goal_x'] == pytest.approx(17.6)


def test_teleop_and_e2e_launch(ros):
    for name, expect in (('teleop.launch.py', {'ugv_sim'}),
                         ('e2e.launch.py', {'ugv_sim', 'ugv_e2e'})):
        mod = _load_launch(name)
        _ctx, nodes = ros.expand_launch(mod.generate_launch_description(), {})
        assert {n.kwargs['name'] for n in nodes} == expect
