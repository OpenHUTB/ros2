# -*- coding: utf-8 -*-
"""fake_ros.py —— 无 ROS 环境下的 rclpy / 消息 / ament / launch 替身。

用途：在 Windows 或没装 ROS 的机器上，把 sim / detector / controller 三个节点
按真实话题接线跑起来，验证四件事：

  1. 节点构造、参数声明、消息字段赋值是否与真实 ROS 2 接口一致；
  2. 「图像 -> 检测 -> 速度指令 -> 动力学」这条闭环在 ROS 侧也能收敛
     （与离线 `sim.rollout` 的结果对上，说明两条链路实现一致）；
  3. install 之后的 share 目录布局能否被 `model_io` 正确定位；
  4. launch 文件是否把参数真正传给了三个节点。

设计原则：**宁可严格，不要宽容**。以下三件事真实 ROS 2 会报错、而宽容的假实现
会悄悄放过，所以这里一律照真实行为抛异常：

  * 给消息的**不存在字段**赋值（如 `msg.header.sec` 应为 `msg.header.stamp.sec`）；
  * 读取**未声明**的参数、重复声明同一个参数；
  * 未调用 `rclpy.init()` 就构造节点。

刻意宽松的地方：数值类型不做强校验。真实 rclpy 允许把 `numpy.int64` 赋给
uint32 字段（序列化时隐式转换），假实现若在这里报错就是**假阳性**，会误导。

只在**没有真实 rclpy**时启用：若机器已装 ROS 2，`install()` 直接抛错，让调用方跳过
测试——真环境当然要用真的跑。
"""
import os
import sys
import types

_CTX = {
    'installed': False,
    'initialized': False,
    't': 0.0,
    'nodes': [],
    'topics': {},        # topic -> [(node, callback, msg_type)]
    'queue': [],         # [(callback, msg, topic)]
    'overrides': {},     # node_name -> {param: value}
    'share_root': None,  # 模拟 install/<ws>/share 的父目录
}


# ======================================================================
# 消息
# ======================================================================
class _Msg:
    """以真实消息定义为准的最小实现：字段名固定，赋值即校验。"""

    _defaults = {}

    def __init__(self, **kwargs):
        object.__setattr__(self, '_d', {})
        for name, default in type(self)._defaults.items():
            self._d[name] = default() if callable(default) else default
        for name, value in kwargs.items():
            setattr(self, name, value)

    def __getattr__(self, name):
        try:
            return object.__getattribute__(self, '_d')[name]
        except KeyError:
            raise AttributeError('%s 没有字段 %r（真实 ROS 同样会抛 AttributeError）'
                                 % (type(self).__name__, name))

    def __setattr__(self, name, value):
        d = object.__getattribute__(self, '_d')
        if name not in d:
            raise AttributeError('%s 没有字段 %r（真实 ROS 同样会抛 AttributeError）'
                                 % (type(self).__name__, name))
        if name == 'data' and not isinstance(value, (bytes, bytearray)):
            raise TypeError('uint8[] 字段 data 必须是 bytes，收到 %s'
                            % type(value).__name__)
        d[name] = value

    def __repr__(self):
        inner = ', '.join('%s=%r' % (k, v) for k, v in self._d.items())
        return '%s(%s)' % (type(self).__name__, inner)


class Time(_Msg):
    _defaults = {'sec': 0, 'nanosec': 0}


class Header(_Msg):
    _defaults = {'stamp': Time, 'frame_id': ''}


class Point(_Msg):
    _defaults = {'x': 0.0, 'y': 0.0, 'z': 0.0}


class Vector3(_Msg):
    _defaults = {'x': 0.0, 'y': 0.0, 'z': 0.0}


class Quaternion(_Msg):
    _defaults = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}


class PointStamped(_Msg):
    _defaults = {'header': Header, 'point': Point}


class Twist(_Msg):
    _defaults = {'linear': Vector3, 'angular': Vector3}


class Pose(_Msg):
    _defaults = {'position': Point, 'orientation': Quaternion}


class PoseWithCovariance(_Msg):
    _defaults = {'pose': Pose, 'covariance': lambda: [0.0] * 36}


class TwistWithCovariance(_Msg):
    _defaults = {'twist': Twist, 'covariance': lambda: [0.0] * 36}


class Odometry(_Msg):
    _defaults = {'header': Header, 'child_frame_id': '',
                 'pose': PoseWithCovariance, 'twist': TwistWithCovariance}


class Image(_Msg):
    _defaults = {'header': Header, 'height': 0, 'width': 0, 'encoding': '',
                 'is_bigendian': 0, 'step': 0, 'data': b''}


# ======================================================================
# 节点 / 发布 / 订阅 / 定时器
# ======================================================================
class Parameter:
    def __init__(self, value):
        self.value = value

    def __repr__(self):
        return 'Parameter(%r)' % (self.value,)


class Logger:
    def __init__(self, node):
        self._node = node

    def _log(self, level, msg):
        self._node.logs.append((level, msg))

    def debug(self, msg, **kw):
        self._log('debug', msg)

    def info(self, msg, **kw):
        self._log('info', msg)

    def warn(self, msg, **kw):
        self._log('warn', msg)

    warning = warn

    def error(self, msg, **kw):
        self._log('error', msg)


class Clock:
    def now(self):
        holder = self

        class _Now:
            def to_msg(self_inner):
                total = _CTX['t']
                return Time(sec=int(total), nanosec=int((total % 1.0) * 1e9))

        return _Now()


class Publisher:
    def __init__(self, node, msg_type, topic, qos):
        self.node, self.msg_type, self.topic, self.qos = node, msg_type, topic, qos
        self.count = 0

    def publish(self, msg):
        if not isinstance(msg, self.msg_type):
            raise TypeError('%s 发布到 %s 的消息类型是 %s，收到 %s'
                            % (self.node.node_name, self.topic,
                               self.msg_type.__name__, type(msg).__name__))
        self.count += 1
        for owner, cb, want in _CTX['topics'].get(self.topic, []):
            if want is not self.msg_type:
                raise TypeError('话题 %s 上发布 %s，但订阅端声明的是 %s（真实 ROS 会拒绝连接）'
                                % (self.topic, self.msg_type.__name__, want.__name__))
            _CTX['queue'].append((cb, msg))


class Subscription:
    def __init__(self, node, msg_type, topic, callback, qos):
        self.node, self.msg_type, self.topic = node, msg_type, topic
        self.callback, self.qos = callback, qos
        _CTX['topics'].setdefault(topic, []).append((node, callback, msg_type))


class Timer:
    def __init__(self, period, callback):
        self.period = float(period)
        self.callback = callback
        self.next_at = _CTX['t'] + self.period
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class Node:
    def __init__(self, node_name, *, parameter_overrides=None, **kwargs):
        if not _CTX['initialized']:
            raise RuntimeError('rclpy.init() 尚未调用，无法构造节点（真实 ROS 同样会失败）')
        self.node_name = node_name
        self._declared = {}
        self._used_overrides = set()
        self._logger = Logger(self)
        self._timers = []
        self._publishers = []
        self.logs = []
        self._overrides = dict(_CTX['overrides'].get(node_name, {}))
        if parameter_overrides:
            self._overrides.update(dict(parameter_overrides))
        _CTX['nodes'].append(self)

    # ---- 参数 ----
    def declare_parameter(self, name, value=None, **kwargs):
        if name in self._declared:
            raise RuntimeError('%s: 参数 %r 重复声明（真实 ROS 抛 '
                               'ParameterAlreadyDeclaredException）' % (self.node_name, name))
        if name in self._overrides:
            self._used_overrides.add(name)
            value = self._overrides[name]
        self._declared[name] = value
        return Parameter(value)

    def get_parameter(self, name):
        if name not in self._declared:
            raise RuntimeError('%s: 读取未声明参数 %r（真实 ROS 抛 '
                               'ParameterNotDeclaredException）' % (self.node_name, name))
        return Parameter(self._declared[name])

    def unused_overrides(self):
        """YAML / launch 传了、但节点没声明的参数——真实 ROS 会静默忽略。"""
        return sorted(set(self._overrides) - self._used_overrides)

    # ---- 通信 ----
    def create_publisher(self, msg_type, topic, qos_profile):
        pub = Publisher(self, msg_type, topic, qos_profile)
        self._publishers.append(pub)
        return pub

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        return Subscription(self, msg_type, topic, callback, qos_profile)

    def create_timer(self, timer_period_sec, callback, **kwargs):
        timer = Timer(timer_period_sec, callback)
        self._timers.append(timer)
        return timer

    # ---- 杂项 ----
    def get_logger(self):
        return self._logger

    def get_clock(self):
        return Clock()

    def destroy_node(self):
        for timer in self._timers:
            timer.cancelled = True

    def published_on(self, topic):
        for pub in self._publishers:
            if pub.topic == topic:
                return pub.count
        raise KeyError('%s 没有发布 %s' % (self.node_name, topic))


# ======================================================================
# 极简执行器
# ======================================================================
class Rig:
    """单线程执行器的近似：推进虚拟时钟 -> 触发到期定时器 -> 派发消息。

    真实 ROS 里定时器和消息回调谁先跑是不确定的；这里固定为「先按时钟顺序
    触发定时器、每触发一次就把消息队列排空」，是其中一种合法交错，足以
    暴露接线、字段、参数问题，也保证测试可复现。
    """

    def __init__(self, dt=0.05):
        self.dt = dt
        self.t = 0.0

    def pump(self, max_rounds=50):
        for _ in range(max_rounds):
            if not _CTX['queue']:
                return
            batch = _CTX['queue']
            _CTX['queue'] = []
            for cb, msg in batch:
                cb(msg)
        raise RuntimeError('消息队列在 %d 轮后仍未排空，疑似回调里无限自触发' % max_rounds)

    def step(self):
        _CTX['t'] += self.dt
        self.t = _CTX['t']
        for node in _CTX['nodes']:
            for timer in list(node._timers):
                if timer.cancelled:
                    continue
                if _CTX['t'] + 1e-9 < timer.next_at:
                    continue
                timer.next_at = _CTX['t'] + timer.period
                timer.callback()
                self.pump()

    def run(self, steps, stop_when=None):
        for i in range(steps):
            self.step()
            if stop_when is not None and stop_when():
                return i + 1
        return steps


# ======================================================================
# 安装替身
# ======================================================================
class _FakePackageNotFound(Exception):
    pass


def _make_ament_module():
    pkg = types.ModuleType('ament_index_python')
    packages = types.ModuleType('ament_index_python.packages')

    def get_package_share_directory(name):
        root = _CTX['share_root']
        if root:
            path = os.path.join(root, name)
            if os.path.isdir(path):
                return path
        raise _FakePackageNotFound(
            "Package '{}' not found（模拟未 install / 未 source 环境）".format(name))

    packages.get_package_share_directory = get_package_share_directory
    packages.PackageNotFoundError = _FakePackageNotFound
    pkg.packages = packages
    return {'ament_index_python': pkg,
            'ament_index_python.packages': packages}


def _make_launch_modules():
    launch = types.ModuleType('launch')
    actions = types.ModuleType('launch.actions')
    subst = types.ModuleType('launch.substitutions')
    launch_ros = types.ModuleType('launch_ros')
    ros_actions = types.ModuleType('launch_ros.actions')

    class LaunchConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            if self.name not in context:
                raise RuntimeError('launch 参数 %r 未声明' % self.name)
            return context[self.name]

        def __repr__(self):
            return 'LaunchConfiguration(%r)' % self.name

    class DeclareLaunchArgument:
        def __init__(self, name, default_value=None, description='', **kwargs):
            self.name = name
            self.default_value = default_value
            self.description = description

    class OpaqueFunction:
        def __init__(self, function, **kwargs):
            self.function = function

    class LaunchDescription:
        def __init__(self, entities=None, **kwargs):
            self.entities = list(entities or [])

    class Node:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

        def __repr__(self):
            return 'Node(%s)' % ', '.join('%s=%r' % kv for kv in self.kwargs.items())

    launch.LaunchDescription = LaunchDescription
    actions.DeclareLaunchArgument = DeclareLaunchArgument
    actions.OpaqueFunction = OpaqueFunction
    subst.LaunchConfiguration = LaunchConfiguration
    launch_ros.actions = ros_actions
    ros_actions.Node = Node
    launch.actions = actions
    launch.substitutions = subst
    return {'launch': launch, 'launch.actions': actions,
            'launch.substitutions': subst, 'launch_ros': launch_ros,
            'launch_ros.actions': ros_actions}


def install(share_root=None):
    """把替身塞进 sys.modules。若本机已有真实 rclpy，抛 RuntimeError 让调用方跳过。"""
    if _CTX['installed']:
        _CTX['share_root'] = share_root or _CTX['share_root']
        return
    try:
        import rclpy  # noqa: F401
    except ImportError:
        pass
    else:
        raise RuntimeError('本机已安装真实 rclpy，请直接跑 ros2 launch 验证，不使用替身')

    rclpy = types.ModuleType('rclpy')
    rclpy_node = types.ModuleType('rclpy.node')

    def init(args=None):
        _CTX['initialized'] = True
        _CTX['t'] = 0.0

    def shutdown():
        _CTX['initialized'] = False

    def spin(node):
        raise RuntimeError('替身不实现 spin；测试请用 fake_ros.Rig 驱动')

    def ok():
        return _CTX['initialized']

    rclpy.init = init
    rclpy.shutdown = shutdown
    rclpy.spin = spin
    rclpy.ok = ok
    rclpy_node.Node = Node
    rclpy.node = rclpy_node

    mods = {'rclpy': rclpy, 'rclpy.node': rclpy_node}
    for group, pairs in (
            ('sensor_msgs', [('Image', Image)]),
            ('geometry_msgs', [('PointStamped', PointStamped), ('Twist', Twist),
                               ('Point', Point), ('Vector3', Vector3),
                               ('Quaternion', Quaternion), ('Pose', Pose)]),
            ('nav_msgs', [('Odometry', Odometry)]),
            ('std_msgs', [('Header', Header)]),
    ):
        top = types.ModuleType(group)
        msg = types.ModuleType('%s.msg' % group)
        for name, cls in pairs:
            setattr(msg, name, cls)
        top.msg = msg
        mods[group] = top
        mods['%s.msg' % group] = msg

    mods.update(_make_ament_module())
    mods.update(_make_launch_modules())
    sys.modules.update(mods)
    _CTX['installed'] = True
    _CTX['share_root'] = share_root


def reset():
    """清空总线与时钟，供每个用例独立运行（模块本身不重复 install）。"""
    _CTX['t'] = 0.0
    _CTX['nodes'] = []
    _CTX['topics'] = {}
    _CTX['queue'] = []
    _CTX['overrides'] = {}
    if _CTX['initialized']:
        _CTX['initialized'] = False


def set_param_overrides(mapping):
    """模拟 launch 传参：{节点名: {参数: 值}}。"""
    _CTX['overrides'] = {k: dict(v) for k, v in mapping.items()}


def set_share_root(path):
    _CTX['share_root'] = path


def expand_launch(description, overrides=None):
    """执行 LaunchDescription 里的 OpaqueFunction，返回 (参数上下文, Node 列表)。

    overrides 用于模拟命令行 `ros2 launch ... scene:=occlusion` 的写法。
    """
    context = {}
    for entity in description.entities:
        if type(entity).__name__ == 'DeclareLaunchArgument':
            context[entity.name] = entity.default_value
    context.update(overrides or {})
    nodes = []
    for entity in description.entities:
        if type(entity).__name__ == 'OpaqueFunction':
            nodes.extend(entity.function(context))
    return context, nodes
