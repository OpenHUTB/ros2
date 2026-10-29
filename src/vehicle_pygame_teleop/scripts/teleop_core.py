#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vehicle_pygame_teleop 布局无关的核心逻辑（不依赖 pygame / ROS）。

把「按键 → 控制意图」和「控制意图 → 车辆运动学积分」从 pygame 渲染与
rospy 发布中拆出来，好处有两个：

1. 这一层没有 pygame / rospy 依赖，可以在没有 ROS、没有显示器、没装图形库
   的机器上跑离线回归：
   ``python3 tests/test_vehicle_pygame_teleop_local.py``
2. 所有可调参数集中到 :class:`TeleopParams`，由 ``config/*.yaml`` 提供默认值、
   由 launch ``<arg>`` 覆盖；每一项默认值都等于改造前的硬编码常量，因此
   「不传参数 = 默认行为不变」。

坐标系约定：世界坐标 x 向东、y 向北；yaw 为绕 z 轴的转角，逆时针为正，
车头朝向 +x 时为 0。俯视画面把世界坐标映射到屏幕（y 轴向下），见节点的
``world_to_screen``。
"""

# 本模块同时兼容 Python 2.7 与 Python 3.x：
# Noetic（Ubuntu 20.04）用 Python 3，而 Melodic（Ubuntu 18.04）的 rospy 仍是
# Python 2.7（REP 151：Noetic 才是第一个完全支持 Python 3 的 ROS 1 发行版）。
# 因此这里只使用两个大版本都支持的语法：不用 f-string，也不引入 3.7+ 的
# ``from __future__ import annotations``。
from __future__ import division, print_function

import math

# ---------------------------------------------------------------- 默认参数
# 与原实现里的模块级常量一一对应；改这里等于改默认行为，
# 请同步 config/vehicle_pygame_teleop.yaml。
DEFAULT_MAX_SPEED = 1.5             # 最大前进速度 m/s
DEFAULT_MAX_REVERSE = 0.8           # 最大倒车速度 m/s
DEFAULT_ACCEL = 2.2                 # 油门 / 倒车加速度 m/s^2
DEFAULT_BRAKE = 4.5                 # 刹车减速度 m/s^2
DEFAULT_HANDBRAKE = 8.0             # 手刹减速度 m/s^2
DEFAULT_DRAG = 1.2                  # 无按键时滑行阻力减速度 m/s^2
DEFAULT_MAX_YAW_RATE = 1.4          # 最大偏航角速度 rad/s
DEFAULT_YAW_RESPONSE = 8.0          # 转向响应角加速度 rad/s^2
DEFAULT_REVERSE_STEER_INVERT = True  # 倒车时转向效果反向
DEFAULT_CMD_VEL_TOPIC = "/cmd_vel"  # 速度指令话题
DEFAULT_PUBLISH_HZ = 30.0           # 发布频率 Hz

# 倒车判定阈值：speed 低于该值视为正在倒车。
# 抽成常量而不是散在代码里的魔数，方便离线测试引用同一来源。
REVERSE_EPS = -0.02

# 控制意图的五个分量
CONTROL_ACTIONS = ("gas", "brake", "left", "right", "handbrake")

# 按键 → 控制意图的映射（单一事实来源：节点、文档、离线测试都引用它）。
# 每项为 (控制意图, 文档中的按键写法, 键名元组)。
KEY_BINDINGS = (
    ("gas", "W / ↑", ("w", "up")),
    ("brake", "S / ↓", ("s", "down")),
    ("left", "A / ←", ("a", "left")),
    ("right", "D / →", ("d", "right")),
    ("handbrake", "空格", ("space",)),
)

# 便于按控制意图取键名；与 KEY_BINDINGS 内容保持一致。
KEY_BINDINGS_MAP = dict((action, names) for action, _label, names in KEY_BINDINGS)


# ---------------------------------------------------------------- 参数
class TeleopParams(object):
    """一次运行使用的全部可调参数。

    默认值与改造前的硬编码常量完全一致；``from_params`` 从 ROS 参数服务器
    读取 ``vehicle/*`` / ``rate/publish_hz`` / ``topic/cmd_vel``，读不到就用
    默认值，所以没有 launch 参数时行为也不变。
    """

    __slots__ = (
        "max_speed", "max_reverse", "accel", "brake", "handbrake", "drag",
        "max_yaw_rate", "yaw_response", "reverse_steer_invert",
        "cmd_vel_topic", "publish_hz",
    )

    def __init__(
        self,
        max_speed=DEFAULT_MAX_SPEED,
        max_reverse=DEFAULT_MAX_REVERSE,
        accel=DEFAULT_ACCEL,
        brake=DEFAULT_BRAKE,
        handbrake=DEFAULT_HANDBRAKE,
        drag=DEFAULT_DRAG,
        max_yaw_rate=DEFAULT_MAX_YAW_RATE,
        yaw_response=DEFAULT_YAW_RESPONSE,
        reverse_steer_invert=DEFAULT_REVERSE_STEER_INVERT,
        cmd_vel_topic=DEFAULT_CMD_VEL_TOPIC,
        publish_hz=DEFAULT_PUBLISH_HZ,
    ):
        self.max_speed = float(max_speed)
        self.max_reverse = float(max_reverse)
        self.accel = float(accel)
        self.brake = float(brake)
        self.handbrake = float(handbrake)
        self.drag = float(drag)
        self.max_yaw_rate = float(max_yaw_rate)
        self.yaw_response = float(yaw_response)
        self.reverse_steer_invert = _as_bool(reverse_steer_invert)
        self.cmd_vel_topic = str(cmd_vel_topic)
        self.publish_hz = float(publish_hz)

    @classmethod
    def from_params(cls, get_param):
        """从 ROS 参数读取；``get_param(name, default)`` 由调用方传入（rospy.get_param）。

        这里用相对名（如 ``vehicle/max_speed``）：节点名在全局命名空间下时，
        相对名解析到全局命名空间，与 launch 里顶层的 ``<param>``/``<rosparam>`` 对应。
        """
        return cls(
            max_speed=get_param("vehicle/max_speed", DEFAULT_MAX_SPEED),
            max_reverse=get_param("vehicle/max_reverse", DEFAULT_MAX_REVERSE),
            accel=get_param("vehicle/accel", DEFAULT_ACCEL),
            brake=get_param("vehicle/brake", DEFAULT_BRAKE),
            handbrake=get_param("vehicle/handbrake", DEFAULT_HANDBRAKE),
            drag=get_param("vehicle/drag", DEFAULT_DRAG),
            max_yaw_rate=get_param("vehicle/max_yaw_rate", DEFAULT_MAX_YAW_RATE),
            yaw_response=get_param("vehicle/yaw_response", DEFAULT_YAW_RESPONSE),
            reverse_steer_invert=get_param(
                "vehicle/reverse_steer_invert", DEFAULT_REVERSE_STEER_INVERT
            ),
            cmd_vel_topic=get_param("topic/cmd_vel", DEFAULT_CMD_VEL_TOPIC),
            publish_hz=get_param("rate/publish_hz", DEFAULT_PUBLISH_HZ),
        )


try:  # Python 2 里 str 是字节串、unicode 才是文本
    _STRING_TYPES = (str, unicode)  # noqa: F821
except NameError:  # Python 3 只有 str
    _STRING_TYPES = (str,)


def _as_bool(value):
    """兼容 YAML/rosparam 传进来的 bool、字符串与数字。"""
    if isinstance(value, bool):
        return value
    if isinstance(value, _STRING_TYPES):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


# ---------------------------------------------------------------- 控制输入
class ControlInput(object):
    """某一帧的控制意图（与 pygame 的按键对象解耦，便于离线构造）。

    五个分量互斥语义由 :func:`step_vehicle` 决定：``handbrake`` 优先级最高，
    然后 ``gas``、``brake``；``left``/``right`` 只影响转向。
    """

    __slots__ = CONTROL_ACTIONS

    def __init__(self, gas=False, brake=False, left=False, right=False, handbrake=False):
        self.gas = bool(gas)
        self.brake = bool(brake)
        self.left = bool(left)
        self.right = bool(right)
        self.handbrake = bool(handbrake)

    def as_tuple(self):
        return tuple(getattr(self, name) for name in CONTROL_ACTIONS)

    def __repr__(self):
        return "ControlInput(%s)" % ", ".join(
            "%s=%s" % (name, getattr(self, name)) for name in CONTROL_ACTIONS
        )


def control_from_predicate(pressed):
    """按 :data:`KEY_BINDINGS` 从 ``pressed(键名) -> bool`` 构造 :class:`ControlInput`。

    节点把 pygame 的按键状态包装成 ``pressed``：``lambda n: keys[_KEYCODE[n]]``。
    """
    return ControlInput(
        gas=any(pressed(name) for name in KEY_BINDINGS_MAP["gas"]),
        brake=any(pressed(name) for name in KEY_BINDINGS_MAP["brake"]),
        left=any(pressed(name) for name in KEY_BINDINGS_MAP["left"]),
        right=any(pressed(name) for name in KEY_BINDINGS_MAP["right"]),
        handbrake=any(pressed(name) for name in KEY_BINDINGS_MAP["handbrake"]),
    )


# ---------------------------------------------------------------- 车辆状态
class VehicleState(object):
    """车辆运动学状态（世界坐标，单位 m / rad / (m/s) / (rad/s)）。"""

    __slots__ = ("x", "y", "yaw", "speed", "yaw_rate")

    def __init__(self, x=0.0, y=0.0, yaw=0.0, speed=0.0, yaw_rate=0.0):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.speed = float(speed)
        self.yaw_rate = float(yaw_rate)


# ---------------------------------------------------------------- 运动学
def step_vehicle(state, control, dt, params):
    """推进一帧差速底盘运动学，就地更新并返回 ``state``。

    与改造前的 ``TeleopCar.update_physics`` 逐句等价，只是把 self 上的状态
    换成显式传入的 :class:`VehicleState`、把 pygame 按键换成 :class:`ControlInput`：

    1. 手刹优先：以 ``handbrake`` 减速到 0，且不响应油门/刹车；
    2. 否则纵向：``gas`` 加速（倒车中先制动到 0 再前进）、``brake`` 先刹车后倒车、
       无按键按 ``drag`` 自然滑行；
    3. 转向：``left`` 给正角速度、``right`` 给负角速度，按 ``yaw_response``
       平滑逼近目标值；
    4. 倒车时用更新后的速度判定（``speed < REVERSE_EPS``），若
       ``reverse_steer_invert`` 为真则转向效果反向（差速底盘特性）；
    5. 按 yaw/yaw_rate 积分航向与位置。
    """
    speed = state.speed

    if control.handbrake:
        # 手刹：快速减速到零，且不响应转向键
        if speed > 0.0:
            speed = max(0.0, speed - params.handbrake * dt)
        elif speed < 0.0:
            speed = min(0.0, speed + params.handbrake * dt)
        target_yaw = 0.0
    else:
        # 纵向控制
        if control.gas:
            if speed < 0.0:
                speed = min(0.0, speed + params.brake * dt)      # 倒车中踩油门先制动
            else:
                speed = min(params.max_speed, speed + params.accel * dt)
        elif control.brake:
            if speed > 0.0:
                speed = max(0.0, speed - params.brake * dt)      # 前进中踩刹车
            else:
                speed = max(-params.max_reverse, speed - params.accel * dt)
        else:
            # 无按键时滑行阻力自然减速
            if speed > 0.0:
                speed = max(0.0, speed - params.drag * dt)
            elif speed < 0.0:
                speed = min(0.0, speed + params.drag * dt)

        # 横向（转向）控制
        target_yaw = 0.0
        if control.left:
            target_yaw = params.max_yaw_rate
        elif control.right:
            target_yaw = -params.max_yaw_rate

    # 倒车时转向效果反向（差速底盘特性）；判定用的是本帧更新后的速度
    if params.reverse_steer_invert and speed < REVERSE_EPS:
        target_yaw = -target_yaw

    # 转向响应平滑
    if state.yaw_rate < target_yaw:
        state.yaw_rate = min(target_yaw, state.yaw_rate + params.yaw_response * dt)
    else:
        state.yaw_rate = max(target_yaw, state.yaw_rate - params.yaw_response * dt)

    state.speed = speed

    # 运动学积分
    state.yaw += state.yaw_rate * dt
    state.x += speed * math.cos(state.yaw) * dt
    state.y += speed * math.sin(state.yaw) * dt
    return state


def twist_fields(state):
    """把车辆状态映射成 ``geometry_msgs/Twist`` 的两个有效字段。

    返回 ``(linear.x, angular.z)``：其余分量（linear.y/z、angular.x/y）恒为 0。
    """
    return state.speed, state.yaw_rate
