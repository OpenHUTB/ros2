#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Twist 到车辆控制量转换的纯逻辑实现。

本文件**不 import rospy**、不使用任何 ROS 消息类型、也不依赖 numpy，
节点（twist_node.py）只负责收发布与日志，所有换算、限幅、积分都在这里，
因此可以在没有 ROS、没有 CARLA、没有显示器的机器上做离线回归测试。

同时兼容 Python 2.7（Melodic 自带的 rospy）与 Python 3.x（Noetic）：
只使用 __future__ 兼容导入、% 格式化、class X(object)、math 标准库。
"""
from __future__ import division
from __future__ import print_function

import math

# ---------------------------------------------------------------- 默认值
# 所有默认值都与 config/twist_to_control.yaml 保持一致；
# 不传任何参数时行为固定，便于复现。

DEFAULT_MODEL = "bicycle"
MODELS = ("bicycle", "ackermann", "diff_drive")

DEFAULT_WHEELBASE = 2.7          # 轴距 L（米），Carla 默认车辆量级
DEFAULT_TRACK_WIDTH = 1.5        # 轮距 b（米），用于阿克曼内外轮差与差速分配

DEFAULT_MAX_STEERING = 0.55      # 前轮最大转角（弧度，约 31.5 度）
DEFAULT_MAX_CURVATURE = 1.0      # 最大曲率（1/米），对应最小转弯半径 1 米
DEFAULT_MAX_SPEED = 5.0          # 最大速度（米/秒），正反向都限
DEFAULT_MAX_ACCEL = 2.0          # 纵向加速度上限（米/秒^2），用于速度平滑
DEFAULT_MAX_STEERING_RATE = 2.0  # 转向角变化率上限（弧度/秒）
DEFAULT_MIN_SPEED = 0.05         # 曲率计算死区：|v| 小于它时曲率无意义
DEFAULT_TIMEOUT = 0.5            # /cmd_vel 超时（秒），超时按零速度处理
DEFAULT_PUBLISH_HZ = 20.0        # 控制量发布频率（赫兹）
DEFAULT_USE_MOCK_ODOM = False    # 是否内置运动学模拟并发 /odom

DEFAULT_CMD_VEL_TOPIC = "/cmd_vel"
DEFAULT_CONTROL_TOPIC = "/vehicle/control"
DEFAULT_WHEEL_TOPIC = "/vehicle/wheel_speed"
DEFAULT_INFO_TOPIC = "/vehicle/control_info"
DEFAULT_ODOM_TOPIC = "/odom"

# 数值保护：避免除零与分母接近零造成的巨大转角
_EPS = 1e-9
_MIN_DENOMINATOR = 1e-3
_MAX_ACCEL_EPS = 1e-9


# ---------------------------------------------------------------- 小工具
def as_bool(value):
    """把 launch / yaml 里可能出现的各种写法统一成 bool。"""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if value is None:
        return False
    text = "%s" % (value,)
    return text.strip().lower() in ("1", "true", "yes", "on")


def as_float(value, default):
    """把字符串或数字转成 float，失败时退回 default。"""
    if value is None:
        return float(default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def clamp(value, low, high):
    """把 value 限制在 [low, high] 内。"""
    if value != value:          # NaN 保护
        return low
    if value < low:
        return low
    if value > high:
        return high
    return value


def move_towards(current, target, max_delta):
    """把 current 朝 target 移动至多 max_delta，用于速度与转角的速率限制。"""
    if max_delta < 0.0:
        max_delta = -max_delta
    diff = target - current
    if diff > max_delta:
        return current + max_delta
    if diff < -max_delta:
        return current - max_delta
    return target


# ---------------------------------------------------------------- 核心换算
def curvature_from_twist(speed, yaw_rate, min_speed=DEFAULT_MIN_SPEED):
    """由 (线速度 v, 角速度 w) 求曲率 k = w / v。

    - 返回 None 表示速度落在死区内、曲率无意义（调用方应锁存上一帧曲率）；
    - speed 为负（倒车）时曲率自动反号，保证倒车打方向的方向正确。
    """
    if abs(speed) < abs(min_speed):
        return None
    return yaw_rate / speed


def steering_from_curvature(curvature, wheelbase=DEFAULT_WHEELBASE,
                            max_curvature=DEFAULT_MAX_CURVATURE,
                            max_steering=DEFAULT_MAX_STEERING):
    """由曲率求前轮转角（自行车模型）：delta = atan(k * L)。

    返回 (转角, 是否饱和)。先按 max_curvature 截断曲率，再按 max_steering 截断转角，
    两级限幅分别对应"路径本身太弯"和"方向盘转不到那么大的角度"。
    """
    limited = clamp(curvature, -max_curvature, max_curvature)
    saturated = abs(limited - curvature) > _EPS
    angle = math.atan(limited * wheelbase)
    if abs(angle) > max_steering:
        angle = clamp(angle, -max_steering, max_steering)
        saturated = True
    return angle, saturated


def ackermann_wheel_angles(curvature, wheelbase=DEFAULT_WHEELBASE,
                           track_width=DEFAULT_TRACK_WIDTH):
    """把曲率分配到左右前轮（真阿克曼转向几何）。

    内侧轮走的圆弧半径更小，因此转角更大：
        delta_in  = atan(L * k / (1 - b * k / 2))
        delta_out = atan(L * k / (1 + b * k / 2))
    返回 (左轮转角, 右轮转角)：k 为正（左转）时左轮是内侧，k 为负（右转）时
    右轮是内侧。实现上先按 |k| 求两侧角度幅值，再按转向号分配左右并加上符号，
    这样右转（k 小于 0）时内侧轮仍然是转角更大的一侧。
    """
    magnitude = abs(curvature)
    half = 0.5 * track_width * magnitude
    denom_inner = 1.0 - half
    denom_outer = 1.0 + half
    if abs(denom_inner) < _MIN_DENOMINATOR:
        denom_inner = _MIN_DENOMINATOR
    if abs(denom_outer) < _MIN_DENOMINATOR:
        denom_outer = _MIN_DENOMINATOR
    inner = math.atan(magnitude * wheelbase / denom_inner)
    outer = math.atan(magnitude * wheelbase / denom_outer)
    if curvature >= 0.0:
        return inner, outer
    return -outer, -inner


def diff_drive_wheel_speeds(speed, curvature, track_width=DEFAULT_TRACK_WIDTH):
    """差速底盘：由速度与曲率求左右轮线速度。

    w = k * v，故 v_left = v - w * b / 2、v_right = v + w * b / 2。
    左转（k 为正）时右轮更快，保证"两轮和 = 2v、两轮差 = w * b"。
    """
    yaw_rate = curvature * speed
    half = 0.5 * yaw_rate * track_width
    return speed - half, speed + half


# ---------------------------------------------------------------- 参数
class ControlParams(object):
    """转换参数。from_params 读取的键名与 launch 暴露的 <arg> 一一对应。"""

    __slots__ = (
        "model", "wheelbase", "track_width",
        "max_steering", "max_curvature", "max_speed", "max_accel",
        "max_steering_rate", "min_speed", "timeout", "publish_hz",
        "cmd_vel_topic", "control_topic", "wheel_topic", "info_topic",
        "odom_topic", "use_mock_odom",
    )

    def __init__(self, model=DEFAULT_MODEL,
                 wheelbase=DEFAULT_WHEELBASE,
                 track_width=DEFAULT_TRACK_WIDTH,
                 max_steering=DEFAULT_MAX_STEERING,
                 max_curvature=DEFAULT_MAX_CURVATURE,
                 max_speed=DEFAULT_MAX_SPEED,
                 max_accel=DEFAULT_MAX_ACCEL,
                 max_steering_rate=DEFAULT_MAX_STEERING_RATE,
                 min_speed=DEFAULT_MIN_SPEED,
                 timeout=DEFAULT_TIMEOUT,
                 publish_hz=DEFAULT_PUBLISH_HZ,
                 cmd_vel_topic=DEFAULT_CMD_VEL_TOPIC,
                 control_topic=DEFAULT_CONTROL_TOPIC,
                 wheel_topic=DEFAULT_WHEEL_TOPIC,
                 info_topic=DEFAULT_INFO_TOPIC,
                 odom_topic=DEFAULT_ODOM_TOPIC,
                 use_mock_odom=DEFAULT_USE_MOCK_ODOM):
        self.model = model if model in MODELS else DEFAULT_MODEL
        self.wheelbase = float(wheelbase)
        self.track_width = float(track_width)
        self.max_steering = float(max_steering)
        self.max_curvature = float(max_curvature)
        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)
        self.max_steering_rate = float(max_steering_rate)
        self.min_speed = float(min_speed)
        self.timeout = float(timeout)
        self.publish_hz = float(publish_hz)
        self.cmd_vel_topic = cmd_vel_topic
        self.control_topic = control_topic
        self.wheel_topic = wheel_topic
        self.info_topic = info_topic
        self.odom_topic = odom_topic
        self.use_mock_odom = bool(use_mock_odom)

    @classmethod
    def from_params(cls, get_param):
        """从 rosparam（launch 里的 <param>/<rosparam>）读取全部参数。"""
        model = get_param("model/type", DEFAULT_MODEL)
        if "%s" % (model,) not in MODELS:
            model = DEFAULT_MODEL
        return cls(
            model="%s" % (model,),
            wheelbase=as_float(get_param("model/wheelbase", DEFAULT_WHEELBASE),
                               DEFAULT_WHEELBASE),
            track_width=as_float(
                get_param("model/track_width", DEFAULT_TRACK_WIDTH),
                DEFAULT_TRACK_WIDTH),
            max_steering=as_float(
                get_param("limit/max_steering", DEFAULT_MAX_STEERING),
                DEFAULT_MAX_STEERING),
            max_curvature=as_float(
                get_param("limit/max_curvature", DEFAULT_MAX_CURVATURE),
                DEFAULT_MAX_CURVATURE),
            max_speed=as_float(get_param("limit/max_speed", DEFAULT_MAX_SPEED),
                               DEFAULT_MAX_SPEED),
            max_accel=as_float(get_param("limit/max_accel", DEFAULT_MAX_ACCEL),
                               DEFAULT_MAX_ACCEL),
            max_steering_rate=as_float(
                get_param("limit/max_steering_rate", DEFAULT_MAX_STEERING_RATE),
                DEFAULT_MAX_STEERING_RATE),
            min_speed=as_float(get_param("limit/min_speed", DEFAULT_MIN_SPEED),
                               DEFAULT_MIN_SPEED),
            timeout=as_float(get_param("control/timeout", DEFAULT_TIMEOUT),
                             DEFAULT_TIMEOUT),
            publish_hz=as_float(get_param("rate/publish_hz", DEFAULT_PUBLISH_HZ),
                                DEFAULT_PUBLISH_HZ),
            cmd_vel_topic=get_param("topic/cmd_vel", DEFAULT_CMD_VEL_TOPIC),
            control_topic=get_param("topic/control", DEFAULT_CONTROL_TOPIC),
            wheel_topic=get_param("topic/wheel_speed", DEFAULT_WHEEL_TOPIC),
            info_topic=get_param("topic/control_info", DEFAULT_INFO_TOPIC),
            odom_topic=get_param("topic/odom", DEFAULT_ODOM_TOPIC),
            use_mock_odom=as_bool(get_param("mock/enabled",
                                            DEFAULT_USE_MOCK_ODOM)),
        )


class TwistCommand(object):
    """一条速度指令：linear 为前进方向线速度，angular 为绕 z 轴角速度。"""

    __slots__ = ("linear", "angular")

    def __init__(self, linear=0.0, angular=0.0):
        self.linear = float(linear)
        self.angular = float(angular)

    def as_tuple(self):
        return self.linear, self.angular

    def is_zero(self, eps=1e-6):
        return abs(self.linear) < eps and abs(self.angular) < eps

    def __repr__(self):
        return "TwistCommand(linear=%.4f, angular=%.4f)" % (self.linear,
                                                            self.angular)


# ---------------------------------------------------------------- 状态
class ControlState(object):
    """控制器的跨帧状态。

    转角变化率限制需要上一帧转角，低速奇点需要锁存上一帧曲率，
    所以这些量必须跨帧保留，不能每帧清零。
    """

    __slots__ = ("speed", "steering", "curvature", "latched_curvature",
                 "requested_steering", "latched", "saturated", "rate_limited")

    def __init__(self, speed=0.0, steering=0.0, curvature=0.0,
                 latched_curvature=0.0):
        self.speed = float(speed)
        self.steering = float(steering)
        self.curvature = float(curvature)
        self.latched_curvature = float(latched_curvature)
        self.requested_steering = float(steering)
        self.latched = False
        self.saturated = False
        self.rate_limited = False

    def as_tuple(self):
        return self.steering, self.speed, self.curvature

    def reset(self):
        self.speed = 0.0
        self.steering = 0.0
        self.curvature = 0.0
        self.latched_curvature = 0.0
        self.requested_steering = 0.0
        self.latched = False
        self.saturated = False
        self.rate_limited = False

    def __repr__(self):
        return ("ControlState(speed=%.4f, steering=%.4f, curvature=%.4f, "
                "latched=%s, saturated=%s, rate_limited=%s)"
                % (self.speed, self.steering, self.curvature,
                   self.latched, self.saturated, self.rate_limited))


def step_control(state, command, dt, params):
    """推进一帧：速度平滑 -> 曲率（含死区锁存）-> 转角（含限幅与速率限制）。

    就地修改并返回 state。dt 小于等于 0 时不做任何推进（便于测试与首帧保护）。
    """
    if dt <= 0.0:
        return state

    # 1) 纵向速度：限速 + 加速度平滑
    target_speed = clamp(command.linear, -params.max_speed, params.max_speed)
    state.speed = move_towards(state.speed, target_speed,
                               params.max_accel * dt)

    # 2) 曲率：死区内锁存上一帧（避免 v -> 0 时 k 发散）
    raw = curvature_from_twist(command.linear, command.angular,
                               params.min_speed)
    if raw is None:
        state.curvature = state.latched_curvature
        state.latched = True
    else:
        state.curvature = clamp(raw, -params.max_curvature,
                                params.max_curvature)
        state.latched_curvature = state.curvature
        state.latched = False

    # 3) 转角：曲率 -> 前轮转角。请求值记录未限幅的角度（路径要求打多少），
    #    实发值再按最大转角与变化率限制，诊断话题里两者的差就是被砍掉的部分。
    raw_angle = math.atan(state.curvature * params.wheelbase)
    target_steering, saturated = steering_from_curvature(
        state.curvature, params.wheelbase, params.max_curvature,
        params.max_steering)
    state.requested_steering = raw_angle
    state.saturated = saturated

    # 4) 转角速率限制：方向盘不能瞬移
    applied = move_towards(state.steering, target_steering,
                           params.max_steering_rate * dt)
    state.rate_limited = abs(applied - target_steering) > _EPS
    state.steering = applied
    return state


def control_fields(state):
    """发布用的控制量三元组：(前轮转角, 速度, 曲率)。"""
    return state.steering, state.speed, state.curvature


def info_fields(state):
    """诊断用五元组：(请求转角, 实发转角, 是否饱和, 是否锁存, 是否速率受限)。"""
    return (state.requested_steering, state.steering,
            1.0 if state.saturated else 0.0,
            1.0 if state.latched else 0.0,
            1.0 if state.rate_limited else 0.0)


# ---------------------------------------------------------------- 模拟车辆
class Pose(object):
    """二维位姿，仅用于内置运动学模拟（mock）。"""

    __slots__ = ("x", "y", "yaw")

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)

    def as_tuple(self):
        return self.x, self.y, self.yaw

    def __repr__(self):
        return "Pose(x=%.4f, y=%.4f, yaw=%.4f)" % (self.x, self.y, self.yaw)


def mock_step(pose, state, dt, params):
    """内置运动学模拟：按当前控制量积分车辆位姿，就地更新并返回 pose。

    - bicycle / ackermann：w = v * tan(delta) / L（用**实发**转角，因此转角
      被速率限制时实际角速度会小于请求值，这一点与真实车辆一致）
    - diff_drive：w = v * k
    """
    if dt <= 0.0:
        return pose
    if params.model == "diff_drive":
        yaw_rate = state.speed * state.curvature
    else:
        yaw_rate = state.speed * math.tan(state.steering) / params.wheelbase
    pose.yaw += yaw_rate * dt
    pose.x += state.speed * math.cos(pose.yaw) * dt
    pose.y += state.speed * math.sin(pose.yaw) * dt
    return pose


def yaw_to_quaternion(yaw):
    """偏航角转四元数，返回 (x, y, z, w)。"""
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(x, y, z, w):
    """四元数转偏航角（与 yaw_to_quaternion 互逆）。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def steady_steering(speed, yaw_rate, params):
    """便捷函数：一步到位求稳态转角（不做速率限制），主要给文档与测试用。"""
    raw = curvature_from_twist(speed, yaw_rate, params.min_speed)
    if raw is None:
        return 0.0, False
    return steering_from_curvature(raw, params.wheelbase,
                                   params.max_curvature, params.max_steering)
