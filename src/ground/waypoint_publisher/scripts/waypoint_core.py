#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""waypoint_publisher 的核心逻辑（不依赖 ROS）。

把「路径点解析与校验」「当前目标点的推进 / 切换」「到达判定」「模拟车辆步进」
从 ROS 的发布订阅里拆出来，好处有三个：

1. 这一层不 import rospy，可以在没有 ROS、没有仿真器、没有图形界面的机器上
   做离线回归：``python3 tests/test_waypoint_publisher_local.py``；
2. 所有可调参数集中到 :class:`PublisherParams`，由 ``config/*.yaml`` 提供默认值、
   由 launch ``<arg>`` 覆盖，命令行即可调整，不必改代码；
3. 坐标系与角度约定集中在这里（世界坐标 x 向东、y 向北、yaw 逆时针为正），
   节点只负责收发消息。

同时兼容 Python 2.7（Melodic）与 Python 3.x（Noetic）：
不使用 f-string，也不引入 3.7+ 才有的 ``from __future__ import annotations``。
"""

from __future__ import division, print_function

import math
from collections import namedtuple

# ---------------------------------------------------------------- 默认参数
DEFAULT_PUBLISH_HZ = 10.0           # 目标点 / 状态的发布频率
DEFAULT_FRAME_ID = "map"            # 路径点所在坐标系
DEFAULT_MODE = "sequential"         # 推进模式：sequential / loop / manual
DEFAULT_TARGET_TOLERANCE = 1.0      # 到达判定阈值 m（<= 0 表示关闭自动推进）
DEFAULT_START_INDEX = 0             # 初始目标点下标
DEFAULT_AUTOSTART = True            # 启动后立即发布目标点
DEFAULT_USE_MOCK_ODOM = False       # 是否内置模拟车辆（无需 CARLA）
DEFAULT_MOCK_SPEED = 2.0            # 模拟车辆速度 m/s

DEFAULT_WAYPOINTS_TOPIC = "/waypoints"
DEFAULT_TARGET_TOPIC = "/waypoint/target"
DEFAULT_INDEX_TOPIC = "/waypoint/current_index"
DEFAULT_SET_INDEX_TOPIC = "/waypoint/set_index"
DEFAULT_ODOM_TOPIC = "/odom"

# 推进模式
MODE_SEQUENTIAL = "sequential"      # 顺序推进，到最后一个后结束
MODE_LOOP = "loop"                  # 循环推进，到最后一个后回到第一个
MODE_MANUAL = "manual"              # 只响应外部切换指令，不自动推进
MODES = (MODE_SEQUENTIAL, MODE_LOOP, MODE_MANUAL)

# 路径点：世界坐标 (x, y) 与朝向 yaw（弧度，逆时针为正，车头朝 +x 时为 0）
# 用 namedtuple 既支持 wp.x 也支持 wp[0]，写代码和写测试都方便。
Waypoint = namedtuple("Waypoint", "x y yaw")


def make_waypoint(x, y, yaw=0.0):
    """构造路径点，顺带把数值统一成 float（YAML 里可能写成整数）。"""
    return Waypoint(float(x), float(y), float(yaw))


# ---------------------------------------------------------------- 解析与校验
def parse_route_text(text, source=""):
    """解析路径点文本：每行一个点，写成 ``x y yaw``（yaw 可省略，默认 0）。

    用纯文本而不是 YAML，是为了让「读路线」不依赖 PyYAML —— 离线测试和手改路线
    都更省事。分隔符可以是空格、制表符或逗号，``#`` 之后是注释，空行忽略。

    ``source`` 只用于报错信息（通常是文件名）。格式不对时抛 ``ValueError``，
    消息里带行号，便于照着改。
    """
    points = []
    for lineno, raw_line in enumerate(text.splitlines(), 1):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.replace(",", " ").split()
        if len(fields) < 2:
            raise _route_error(source, lineno,
                               "至少需要 x、y 两个数，实际是 %r" % raw_line.strip())
        if len(fields) > 3:
            raise _route_error(source, lineno,
                               "每行最多 3 个数（x y yaw），实际有 %d 个" % len(fields))
        try:
            numbers = [float(field) for field in fields]
        except ValueError:
            raise _route_error(source, lineno, "坐标不是数字：%r" % raw_line.strip())
        yaw = numbers[2] if len(numbers) == 3 else 0.0
        points.append(make_waypoint(numbers[0], numbers[1], yaw))
    if not points:
        raise _route_error(source, 0, "没有解析到任何路径点（是不是全是注释或空行？）")
    return points


def _route_error(source, lineno, message):
    """拼一条带位置信息的报错，统一 parse_route_text 的错误格式。"""
    where = source or "路径点文本"
    if lineno:
        return ValueError("%s 第 %d 行：%s" % (where, lineno, message))
    return ValueError("%s：%s" % (where, message))


def validate_waypoints(points):
    """检查一条路线是否可用，返回问题列表（空列表表示通过）。

    只做「离线就能判断」的检查：数量、NaN/Inf、相邻点重合。
    路面是否在车道上这类需要地图信息的检查不在这里做。
    """
    problems = []
    if not points:
        problems.append("路径点列表为空")
        return problems
    if len(points) < 2:
        problems.append("至少需要 2 个路径点才能构成路线，当前只有 %d 个" % len(points))
    for i, p in enumerate(points):
        if _not_finite(p.x) or _not_finite(p.y) or _not_finite(p.yaw):
            problems.append("第 %d 个路径点含 NaN/Inf" % (i + 1))
    for i in range(1, len(points)):
        d = distance(points[i - 1], points[i])
        if d <= 1e-9:
            problems.append("第 %d 与第 %d 个路径点重合（相距 %.3f m）" % (i, i + 1, d))
    return problems


def _not_finite(value):
    return math.isnan(value) or math.isinf(value)


def demo_route():
    """内置示例路线：10 m 见方的矩形环路，8 个点，不需要任何外部文件。

    用矩形是因为四角都是 90° 弯，能直观看出「目标点依次切换」和「弯道附近
    到达判定」是否符合预期；点数少也方便在 rviz 里一眼数清。
    """
    return [
        make_waypoint(0.0, 0.0),
        make_waypoint(5.0, 0.0),
        make_waypoint(10.0, 0.0),
        make_waypoint(10.0, 5.0),
        make_waypoint(10.0, 10.0),
        make_waypoint(5.0, 10.0),
        make_waypoint(0.0, 10.0),
        make_waypoint(0.0, 5.0),
    ]


# ---------------------------------------------------------------- 几何
def distance(a, b):
    """两点（只取 x、y）的平面距离。参数可以是 Waypoint，也可以是 (x, y, ...) 元组。"""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def reached(pose, target, tolerance):
    """位姿是否已到达目标点。

    ``tolerance <= 0`` 视为**关闭自动到达判定**：返回 False，此时只能通过外部
    指令切换目标点（对应 ``control/target_tolerance: 0`` 的用法）。
    """
    if tolerance <= 0.0:
        return False
    return distance(pose, target) <= tolerance


def nearest_index(pose, points):
    """返回离 ``pose`` 最近的路径点下标；列表为空时返回 -1。"""
    if not points:
        return -1
    best_index = 0
    best_distance = distance(pose, points[0])
    for i in range(1, len(points)):
        d = distance(pose, points[i])
        if d < best_distance:
            best_index, best_distance = i, d
    return best_index


def step_towards(pose, target, step):
    """把位姿沿直线朝目标点推进 ``step`` 米，返回新的 (x, y, yaw)。

    不会越过目标点（最后一步正好落在目标上）；``step <= 0`` 或已经重合时原样
    返回。航向取「当前位置指向目标」的方向，因此模拟车辆看起来总是朝着目标开。

    这是给内置模拟车辆用的最小运动学，故意做得可离线断言 —— 真车 / CARLA 的
    位姿由 ``/odom`` 提供，与这个函数无关。
    """
    x, y, yaw = pose[0], pose[1], pose[2]
    dx, dy = target[0] - x, target[1] - y
    d = math.hypot(dx, dy)
    if d <= 1e-9 or step <= 0.0:
        return (x, y, yaw)
    k = min(float(step), d) / d
    return (x + dx * k, y + dy * k, math.atan2(dy, dx))


def yaw_to_quaternion(yaw):
    """把偏航角换成绕 z 轴的四元数分量 ``(z, w)``（x、y 恒为 0）。

    ROS 的 Pose / PoseStamped / PoseArray 都要求四元数，平面小车只有 z 分量，
    单独抽出来便于离线断言「朝向没写错轴」。
    """
    return (math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(x, y, z, w):
    """从四元数取出平面偏航角（弧度）。

    与 :func:`yaw_to_quaternion` 互为逆运算：订阅 ``/odom`` 时用它把车辆姿态
    转回 yaw，再做到达判定。
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ---------------------------------------------------------------- 目标点游标
class TargetCursor(object):
    """当前目标点的游标：负责推进、循环回绕、手动切换与结束判定。

    这个类是「动态目标点切换」的核心：无论什么模式，都可以用 :meth:`set_index`
    直接跳到任意路径点，或用 :meth:`force_next` 往前走一格。
    """

    def __init__(self, count, mode=DEFAULT_MODE, start_index=DEFAULT_START_INDEX):
        count = int(count)
        if count <= 0:
            raise ValueError("路径点数量必须大于 0，实际是 %d" % count)
        if mode not in MODES:
            raise ValueError("不支持的推进模式 %r（可选：%s）" % (mode, ", ".join(MODES)))
        self.count = count
        self.mode = mode
        self.finished = False
        self.index = self._clamp(int(start_index))

    def _clamp(self, index):
        if index < 0:
            return 0
        if index >= self.count:
            return self.count - 1
        return index

    def current(self):
        """当前目标点下标。"""
        return self.index

    def set_index(self, index):
        """动态切换目标点。越界或不是整数时返回 False，且**不改变**当前下标。"""
        try:
            value = int(index)
        except (TypeError, ValueError):
            return False
        if value < 0 or value >= self.count:
            return False
        self.index = value
        self.finished = False
        return True

    def advance(self):
        """到达当前目标后推进一格；``manual`` 模式下不自动推进。"""
        if self.mode == MODE_MANUAL:
            return self.index
        return self.force_next()

    def force_next(self):
        """不管什么模式都往前走一格（``manual`` 模式下的手动「下一个」也用它）。

        ``loop`` 模式在末尾回绕到 0；其余模式在末尾置 ``finished`` 并停在最后一点。
        """
        if self.index + 1 < self.count:
            self.index += 1
            self.finished = False
        elif self.mode == MODE_LOOP:
            self.index = 0
            self.finished = False
        else:
            self.finished = True
        return self.index

    def jump_to_nearest(self, pose, points):
        """把目标点切到离 ``pose`` 最近的那个路径点；成功返回 True。"""
        index = nearest_index(pose, points)
        if index < 0:
            return False
        return self.set_index(index)

    def at_last(self):
        """当前目标点是否已经是最后一个路径点。"""
        return self.index == self.count - 1

    def is_finished(self):
        """``sequential`` 模式走完最后一个点后为 True。"""
        return self.finished

    def __repr__(self):
        return "TargetCursor(index=%d, count=%d, mode=%s, finished=%s)" % (
            self.index, self.count, self.mode, self.finished)


# ---------------------------------------------------------------- 参数
class PublisherParams(object):
    """一次运行使用的全部可调参数。

    默认值集中在文件顶部常量里；``from_params`` 从 ROS 参数服务器读取
    ``rate/*`` / ``frame/*`` / ``control/*`` / ``route/*`` / ``mock/*`` / ``topic/*``，
    读不到就用默认值，所以不传任何参数时也能直接跑起来。
    """

    __slots__ = (
        "publish_hz", "frame_id", "mode", "target_tolerance", "start_index",
        "autostart", "route_file", "use_mock_odom", "mock_speed",
        "waypoints_topic", "target_topic", "index_topic", "set_index_topic",
        "odom_topic",
    )

    def __init__(
        self,
        publish_hz=DEFAULT_PUBLISH_HZ,
        frame_id=DEFAULT_FRAME_ID,
        mode=DEFAULT_MODE,
        target_tolerance=DEFAULT_TARGET_TOLERANCE,
        start_index=DEFAULT_START_INDEX,
        autostart=DEFAULT_AUTOSTART,
        route_file="",
        use_mock_odom=DEFAULT_USE_MOCK_ODOM,
        mock_speed=DEFAULT_MOCK_SPEED,
        waypoints_topic=DEFAULT_WAYPOINTS_TOPIC,
        target_topic=DEFAULT_TARGET_TOPIC,
        index_topic=DEFAULT_INDEX_TOPIC,
        set_index_topic=DEFAULT_SET_INDEX_TOPIC,
        odom_topic=DEFAULT_ODOM_TOPIC,
    ):
        self.publish_hz = float(publish_hz)
        self.frame_id = str(frame_id)
        self.mode = str(mode)
        if self.mode not in MODES:
            raise ValueError("不支持的推进模式 %r（可选：%s）"
                             % (self.mode, ", ".join(MODES)))
        self.target_tolerance = float(target_tolerance)
        self.start_index = int(start_index)
        self.autostart = as_bool(autostart)
        self.route_file = str(route_file) if route_file else ""
        self.use_mock_odom = as_bool(use_mock_odom)
        self.mock_speed = float(mock_speed)
        self.waypoints_topic = str(waypoints_topic)
        self.target_topic = str(target_topic)
        self.index_topic = str(index_topic)
        self.set_index_topic = str(set_index_topic)
        self.odom_topic = str(odom_topic)

    @classmethod
    def from_params(cls, get_param):
        """从 ROS 参数读取；``get_param(name, default)`` 由调用方传入（rospy.get_param）。

        相对名（如 ``rate/publish_hz``）在节点处于全局命名空间时解析到全局命名空间，
        与 launch 里顶层的 ``<rosparam>`` / ``<param>`` 对应。
        """
        return cls(
            publish_hz=get_param("rate/publish_hz", DEFAULT_PUBLISH_HZ),
            frame_id=get_param("frame/id", DEFAULT_FRAME_ID),
            mode=get_param("control/mode", DEFAULT_MODE),
            target_tolerance=get_param("control/target_tolerance", DEFAULT_TARGET_TOLERANCE),
            start_index=get_param("control/start_index", DEFAULT_START_INDEX),
            autostart=get_param("control/autostart", DEFAULT_AUTOSTART),
            route_file=get_param("route/file", ""),
            use_mock_odom=get_param("mock/enabled", DEFAULT_USE_MOCK_ODOM),
            mock_speed=get_param("mock/speed", DEFAULT_MOCK_SPEED),
            waypoints_topic=get_param("topic/waypoints", DEFAULT_WAYPOINTS_TOPIC),
            target_topic=get_param("topic/target", DEFAULT_TARGET_TOPIC),
            index_topic=get_param("topic/current_index", DEFAULT_INDEX_TOPIC),
            set_index_topic=get_param("topic/set_index", DEFAULT_SET_INDEX_TOPIC),
            odom_topic=get_param("topic/odom", DEFAULT_ODOM_TOPIC),
        )


try:  # Python 2：文本类型是 unicode，str 是字节串
    _STRING_TYPES = (str, unicode)  # noqa: F821
except NameError:  # Python 3 只有 str
    _STRING_TYPES = (str,)


def as_bool(value):
    """兼容 YAML / rosparam 传进来的 bool、字符串与数字。"""
    if isinstance(value, bool):
        return value
    if isinstance(value, _STRING_TYPES):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)
