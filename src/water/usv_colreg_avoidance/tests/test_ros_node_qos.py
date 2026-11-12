"""节点侧回归：轨迹的 QoS 与 replay 重发。

背景（评审意见）：`/usv_colreg/track_a` 原先是默认的 VOLATILE QoS，而且只在
节点初始化时发一次。DDS 不会为 VOLATILE 的晚到订阅者补发历史样本，所以
RViz 晚启动、重连或切换 Fixed Frame 之后就永远收不到那条 Path，表现为话题
超时、主船轨迹空白；调 `/usv_colreg/replay` 也不会重发。

本文件把 rclpy / 消息包换成一圈最小桩，在没有 ROS 的机器上真把
`UsvColregNode` 构造起来，然后断言两件事：

1. 轨迹与 Marker 的发布端声明的是 TRANSIENT_LOCAL + RELIABLE + KEEP_LAST(1)；
2. `_on_replay` 会把完整 Path 重新发一次，而不只是把 index 归零。

第 1 条锁的是"声明"，DDS 实际的补发行为只能在真 ROS 环境里验（见 README
「验证」一节）。第 2 条是纯 Python 逻辑，与 DDS 无关。
"""
import os
import sys
import types

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


# --------------------------------------------------------------- rclpy 桩
class _Msg:
    """够用的消息基类：任意字段名都能直接赋值/读取。"""

    def __init__(self, **fields):
        for key, value in fields.items():
            setattr(self, key, value)


class _Header(_Msg):
    def __init__(self, frame_id='', stamp=None):
        super().__init__(frame_id=frame_id, stamp=stamp)


class _Vector3(_Msg):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        super().__init__(x=x, y=y, z=z)


class _Quaternion(_Msg):
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        super().__init__(x=x, y=y, z=z, w=w)


class _Pose(_Msg):
    def __init__(self):
        super().__init__(position=_Vector3(), orientation=_Quaternion())


class _PoseStamped(_Msg):
    def __init__(self):
        super().__init__(header=_Header(), pose=_Pose())


class _Path(_Msg):
    def __init__(self):
        super().__init__(header=_Header(), poses=[])


class _String(_Msg):
    def __init__(self):
        super().__init__(data='')


class _Marker(_Msg):
    ADD = 0
    DELETE = 2
    DELETEALL = 3
    CUBE = 1
    LINE_STRIP = 4
    TEXT_VIEW_FACING = 9

    def __init__(self):
        super().__init__(
            header=_Header(), ns='', id=0, type=0, action=0,
            pose=_Pose(), scale=_Vector3(), color=_Msg(r=0.0, g=0.0, b=0.0, a=1.0),
            points=[], text='')


class _MarkerArray(_Msg):
    def __init__(self):
        super().__init__(markers=[])


class _Duration:
    def to_msg(self):
        return _Msg(sec=0, nanosec=0)


class _Clock:
    def now(self):
        return _Duration()


class _Logger:
    def info(self, *args, **kwargs):
        pass


class _Parameter:
    def __init__(self, value):
        self.value = value


class _QoSProfile:
    def __init__(self, depth=10, history=None, reliability=None, durability=None):
        self.depth = depth
        self.history = history
        self.reliability = reliability
        self.durability = durability


class _Policy:
    pass


class _Publisher:
    def __init__(self, msg_type, topic, qos):
        self.msg_type = msg_type
        self.topic = topic
        self.qos = qos
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Node:
    def __init__(self, name):
        self.name = name
        self._params = {}
        self.publishers = {}
        self.services = {}
        self.timers = []

    def declare_parameter(self, name, default):
        self._params.setdefault(name, default)

    def get_parameter(self, name):
        return _Parameter(self._params[name])

    def create_publisher(self, msg_type, topic, qos):
        pub = _Publisher(msg_type, topic, qos)
        self.publishers[topic] = pub
        return pub

    def create_service(self, srv_type, name, callback):
        self.services[name] = callback

    def create_timer(self, period, callback):
        self.timers.append((period, callback))
        return _Msg()

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return _Logger()

    def destroy_node(self):
        pass


def _module(name, **attrs):
    mod = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(mod, key, value)
    return mod


STUB_NAMES = [
    'rclpy', 'rclpy.node', 'rclpy.qos',
    'geometry_msgs', 'geometry_msgs.msg',
    'nav_msgs', 'nav_msgs.msg',
    'std_msgs', 'std_msgs.msg',
    'std_srvs', 'std_srvs.srv',
    'visualization_msgs', 'visualization_msgs.msg',
]


def _build_stubs():
    qos = _module(
        'rclpy.qos',
        QoSProfile=_QoSProfile,
        HistoryPolicy=_Policy(),
        ReliabilityPolicy=_Policy(),
        DurabilityPolicy=_Policy(),
    )
    qos.HistoryPolicy.KEEP_LAST = 'KEEP_LAST'
    qos.ReliabilityPolicy.RELIABLE = 'RELIABLE'
    qos.DurabilityPolicy.TRANSIENT_LOCAL = 'TRANSIENT_LOCAL'

    rclpy = _module('rclpy', init=lambda *a, **k: None, spin=lambda *a, **k: None,
                    try_shutdown=lambda: None)
    rclpy.node = _module('rclpy.node', Node=_Node)
    rclpy.qos = qos

    return {
        'rclpy': rclpy,
        'rclpy.node': rclpy.node,
        'rclpy.qos': qos,
        'geometry_msgs': _module('geometry_msgs'),
        'geometry_msgs.msg': _module('geometry_msgs.msg', Point=_Vector3,
                                     PoseStamped=_PoseStamped),
        'nav_msgs': _module('nav_msgs'),
        'nav_msgs.msg': _module('nav_msgs.msg', Path=_Path),
        'std_msgs': _module('std_msgs'),
        'std_msgs.msg': _module('std_msgs.msg', String=_String),
        'std_srvs': _module('std_srvs'),
        'std_srvs.srv': _module('std_srvs.srv', Trigger=object),
        'visualization_msgs': _module('visualization_msgs'),
        'visualization_msgs.msg': _module('visualization_msgs.msg',
                                          Marker=_Marker, MarkerArray=_MarkerArray),
    }


@pytest.fixture(scope='module')
def ros_node():
    """装好桩再导入节点模块，导入完立刻把桩撤掉，不污染其它测试。"""
    saved = {name: sys.modules.get(name) for name in STUB_NAMES}
    for name, mod in _build_stubs().items():
        sys.modules[name] = mod
    try:
        sys.modules.pop('usv_colreg_avoidance.ros_node', None)
        import importlib
        module = importlib.import_module('usv_colreg_avoidance.ros_node')
    finally:
        for name, old in saved.items():
            if old is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old
    return module


@pytest.fixture(scope='module')
def node(ros_node):
    return ros_node.UsvColregNode()


def test_path_publisher_is_latched(node):
    """轨迹发布端必须是 TRANSIENT_LOCAL，晚到的订阅者才拿得到整条 Path。"""
    pub = node.publishers['/usv_colreg/track_a']
    assert pub.qos.durability == 'TRANSIENT_LOCAL'
    assert pub.qos.reliability == 'RELIABLE'
    assert pub.qos.history == 'KEEP_LAST'
    # 只留最后一条就够：轨迹每次都是整条重发，不需要队列深度
    assert pub.qos.depth == 1


def test_marker_publisher_is_latched(node):
    pub = node.publishers['/usv_colreg/markers']
    assert pub.qos.durability == 'TRANSIENT_LOCAL'


def test_path_is_published_at_startup(node):
    pub = node.publishers['/usv_colreg/track_a']
    assert len(pub.sent) == 1
    path = pub.sent[0]
    assert path.header.frame_id == 'map'
    assert len(path.poses) == len(node.result.frames[::4])
    assert all(p.header.frame_id == 'map' for p in path.poses)


def test_replay_republishes_full_path(node):
    """replay 必须重发完整 Path，只归零 index 的话订阅端还是看不到轨迹。"""
    pub = node.publishers['/usv_colreg/track_a']
    before = len(pub.sent)

    response = _Msg(success=False, message='')
    node.index = 17
    result = node._on_replay(_Msg(), response)

    assert result.success is True
    assert node.index == 0
    assert len(pub.sent) == before + 1
    resent = pub.sent[-1]
    assert len(resent.poses) == len(pub.sent[0].poses)
    assert resent.poses[0].pose.position.x == pub.sent[0].poses[0].pose.position.x


def test_rviz_config_subscribes_transient_local():
    """RViz 订阅端也得是 Transient Local，否则发布端 latch 了也配不上。"""
    rviz = os.path.join(ROOT, 'rviz', 'usv_colreg_avoidance.rviz')
    text = open(rviz, encoding='utf-8').read()
    for topic in ('/usv_colreg/track_a', '/usv_colreg/markers'):
        idx = text.index('Value: ' + topic)
        window = text[max(0, idx - 400):idx]
        assert 'Durability Policy: Transient Local' in window, topic
    assert 'Durability Policy: Volatile' not in text
