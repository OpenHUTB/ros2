#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""场景运行器的纯逻辑部分：场景解析、内置车辆、轨迹指标与断言。

本模块刻意不 import rospy、不 import PyYAML、不依赖 CARLA 或任何模拟器，
因此可以在没有 ROS 的机器上直接导入并回归测试；ROS 相关的收发放在
scenario_runner_node.py 里。
"""

from __future__ import division, print_function

import io
import math
import re

DEFAULT_PUBLISH_HZ = 20.0
DEFAULT_SIM_DT = 0.05
DEFAULT_USE_MOCK_ODOM = True

DEFAULT_MAX_SPEED = 5.0
DEFAULT_MAX_ACCEL = 1.5
DEFAULT_MAX_DECEL = 4.0
DEFAULT_MAX_YAW_RATE = 1.0

DEFAULT_DURATION = 20.0
DEFAULT_TOLERANCE = 0.05

EXPECT_TYPES = (
    "max_speed",
    "reach_speed",
    "final_speed",
    "distance",
    "position",
    "heading",
)

_NUMBER_RE = re.compile(r"^[+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?$")


class ScenarioError(Exception):
    """场景文件不合法。"""


class Pose(object):
    """二维位姿（位置 + 航向角，单位米 / 弧度）。"""

    __slots__ = ("x", "y", "yaw")

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)

    def as_tuple(self):
        return (self.x, self.y, self.yaw)

    def __repr__(self):
        return "Pose(x=%.3f, y=%.3f, yaw=%.3f)" % (self.x, self.y, self.yaw)


class Command(object):
    """一条速度指令，对应 geometry_msgs/Twist。"""

    __slots__ = ("linear", "angular")

    def __init__(self, linear=0.0, angular=0.0):
        self.linear = float(linear)
        self.angular = float(angular)

    def as_tuple(self):
        return (self.linear, self.angular)

    def __repr__(self):
        return "Command(linear=%.3f, angular=%.3f)" % (self.linear, self.angular)


class Step(object):
    """场景里的一个时间点：到 at 秒时切换成 command。"""

    __slots__ = ("at", "name", "command")

    def __init__(self, at, command, name=""):
        self.at = float(at)
        self.command = command
        self.name = name

    def __repr__(self):
        return "Step(at=%.2f, name=%r, command=%r)" % (self.at, self.name, self.command)


class Expect(object):
    """一条断言。字段随 type 不同而不同。"""

    __slots__ = ("type", "value", "tolerance", "min_value", "max_value", "x", "y")

    def __init__(self, raw):
        if not isinstance(raw, dict):
            raise ScenarioError("expects 的每一项都必须是映射")
        kind = raw.get("type")
        if not kind:
            raise ScenarioError("expects 缺 `type` 字段")
        kind = _as_text(kind)
        if kind not in EXPECT_TYPES:
            raise ScenarioError(
                "不支持的断言类型 %s，可选：%s" % (kind, ", ".join(EXPECT_TYPES))
            )
        self.type = kind
        self.value = _as_float(raw.get("value"), None, "expects.value")
        self.tolerance = _as_float(
            raw.get("tolerance"), DEFAULT_TOLERANCE, "expects.tolerance"
        )
        self.min_value = _as_float(raw.get("min"), None, "expects.min")
        self.max_value = _as_float(raw.get("max"), None, "expects.max")
        self.x = _as_float(raw.get("x"), None, "expects.x")
        self.y = _as_float(raw.get("y"), None, "expects.y")
        self._check_shape()

    def _check_shape(self):
        if self.type in ("max_speed", "reach_speed", "final_speed"):
            if self.value is None:
                raise ScenarioError("%s 断言必须给 value" % self.type)
            if self.tolerance < 0:
                raise ScenarioError("%s 的 tolerance 不能为负" % self.type)
        elif self.type == "distance":
            if self.min_value is None and self.max_value is None:
                raise ScenarioError("distance 断言至少要给 min 或 max")
            if (
                self.min_value is not None
                and self.max_value is not None
                and self.min_value > self.max_value
            ):
                raise ScenarioError("distance 的 min 不能大于 max")
        elif self.type == "position":
            if self.x is None or self.y is None:
                raise ScenarioError("position 断言必须同时给 x 和 y")
            if self.tolerance < 0:
                raise ScenarioError("position 的 tolerance 不能为负")
        elif self.type == "heading":
            if self.value is None:
                raise ScenarioError("heading 断言必须给 value（弧度）")
            if self.tolerance < 0:
                raise ScenarioError("heading 的 tolerance 不能为负")

    def describe(self):
        if self.type == "max_speed":
            return "最大速度不超过 %.2f m/s" % (self.value + self.tolerance)
        if self.type == "reach_speed":
            return "峰值速度达到 %.2f m/s（允许 %.2f）" % (self.value, self.tolerance)
        if self.type == "final_speed":
            return "结束速度等于 %.2f m/s（允许 %.2f）" % (self.value, self.tolerance)
        if self.type == "distance":
            return "行驶里程在 [%s, %s] 之间" % (
                "-" if self.min_value is None else "%.2f" % self.min_value,
                "-" if self.max_value is None else "%.2f" % self.max_value,
            )
        if self.type == "position":
            return "结束位置离 (%.2f, %.2f) 不超过 %.2f m" % (
                self.x,
                self.y,
                self.tolerance,
            )
        return "结束航向为 %.3f rad（允许 %.3f）" % (self.value, self.tolerance)

    def __repr__(self):
        return "Expect(%r)" % (self.type,)


class Scenario(object):
    """一份场景脚本。"""

    __slots__ = (
        "name",
        "duration",
        "initial",
        "initial_speed",
        "steps",
        "expects",
        "source",
    )

    def __init__(
        self, name, duration, initial, steps, expects, source="", initial_speed=0.0
    ):
        self.name = name
        self.duration = duration
        self.initial = initial
        self.initial_speed = float(initial_speed)
        self.steps = steps
        self.expects = expects
        self.source = source

    def command_at(self, elapsed):
        """返回 elapsed 秒时应当发布的速度指令。"""
        current = Command(0.0, 0.0)
        for step in self.steps:
            if elapsed + 1e-9 >= step.at:
                current = step.command
            else:
                break
        return current

    def __repr__(self):
        return "Scenario(name=%r, duration=%.2f, steps=%d, expects=%d)" % (
            self.name,
            self.duration,
            len(self.steps),
            len(self.expects),
        )


class Sample(object):
    """轨迹里的一个采样点。"""

    __slots__ = ("t", "x", "y", "yaw", "speed")

    def __init__(self, t, x, y, yaw, speed):
        self.t = float(t)
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.speed = float(speed)

    def as_tuple(self):
        return (self.t, self.x, self.y, self.yaw, self.speed)

    def __repr__(self):
        return "Sample(t=%.3f, x=%.3f, y=%.3f, yaw=%.3f, speed=%.3f)" % (
            self.t,
            self.x,
            self.y,
            self.yaw,
            self.speed,
        )


class Trajectory(object):
    """一次运行记录下来的轨迹。"""

    __slots__ = ("samples", "name")

    def __init__(self, samples, name=""):
        self.samples = list(samples)
        self.name = name

    def is_empty(self):
        return not self.samples

    def final(self):
        if self.is_empty():
            raise ScenarioError("轨迹为空，无法取末帧")
        return self.samples[-1]

    def duration(self):
        if self.is_empty():
            return 0.0
        return self.samples[-1].t - self.samples[0].t

    def peak_speed(self):
        if self.is_empty():
            return 0.0
        return max(sample.speed for sample in self.samples)

    def travelled(self):
        """累计里程（按采样点之间速度积分，倒退也计入）。"""
        total = 0.0
        for index in range(1, len(self.samples)):
            previous = self.samples[index - 1]
            current = self.samples[index]
            total += abs(current.speed) * (current.t - previous.t)
        return total

    def __repr__(self):
        return "Trajectory(%d samples, %.2f s)" % (
            len(self.samples),
            self.duration(),
        )


class VehicleProfile(object):
    """内置车辆的速度约束。"""

    __slots__ = ("max_speed", "max_accel", "max_decel", "max_yaw_rate")

    def __init__(
        self,
        max_speed=DEFAULT_MAX_SPEED,
        max_accel=DEFAULT_MAX_ACCEL,
        max_decel=DEFAULT_MAX_DECEL,
        max_yaw_rate=DEFAULT_MAX_YAW_RATE,
    ):
        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)
        self.max_yaw_rate = float(max_yaw_rate)
        if self.max_speed <= 0:
            raise ScenarioError("vehicle/max_speed 必须大于 0")
        if self.max_accel <= 0 or self.max_decel <= 0:
            raise ScenarioError("vehicle/max_accel 与 max_decel 必须大于 0")
        if self.max_yaw_rate <= 0:
            raise ScenarioError("vehicle/max_yaw_rate 必须大于 0")

    def as_tuple(self):
        return (self.max_speed, self.max_accel, self.max_decel, self.max_yaw_rate)

    def __repr__(self):
        return (
            "VehicleProfile(max_speed=%.2f, max_accel=%.2f, "
            "max_decel=%.2f, max_yaw_rate=%.2f)"
        ) % self.as_tuple()


class Result(object):
    """一条断言的判定结果。"""

    __slots__ = ("index", "expect", "passed", "detail")

    def __init__(self, index, expect, passed, detail):
        self.index = index
        self.expect = expect
        self.passed = passed
        self.detail = detail

    def __repr__(self):
        return "Result(#%d %s %s %s)" % (
            self.index,
            self.expect.type,
            "PASS" if self.passed else "FAIL",
            self.detail,
        )


# ---------------------------------------------------------------- 场景解析


def _strip_comment(line):
    """去掉行尾注释，但保留引号里的 #。"""
    out = []
    quote = None
    for char in line:
        if quote:
            out.append(char)
            if char == quote:
                quote = None
            continue
        if char in ('"', "'"):
            quote = char
            out.append(char)
            continue
        if char == "#":
            break
        out.append(char)
    return "".join(out).rstrip()


def _split_top_level(text):
    """按顶层逗号切分（忽略花括号、方括号与引号里的逗号）。"""
    items = []
    buffer = []
    depth = 0
    quote = None
    for char in text:
        if quote:
            buffer.append(char)
            if char == quote:
                quote = None
            continue
        if char in ('"', "'"):
            quote = char
            buffer.append(char)
            continue
        if char in "{[":
            depth += 1
            buffer.append(char)
            continue
        if char in "}]":
            depth -= 1
            buffer.append(char)
            continue
        if char == "," and depth == 0:
            items.append("".join(buffer).strip())
            buffer = []
            continue
        buffer.append(char)
    tail = "".join(buffer).strip()
    if tail:
        items.append(tail)
    return [item for item in items if item]


def _parse_scalar(text):
    """把一段文本解释成数字 / 布尔 / 空 / 字符串。"""
    text = text.strip()
    if not text:
        return None
    if len(text) >= 2 and text[0] == text[-1] and text[0] in ('"', "'"):
        return text[1:-1]
    lowered = text.lower()
    if lowered in ("true", "yes", "on"):
        return True
    if lowered in ("false", "no", "off"):
        return False
    if lowered in ("null", "none", "~"):
        return None
    if _NUMBER_RE.match(text):
        if "." in text or "e" in lowered:
            return float(text)
        return int(text)
    return text


def _parse_inline(text):
    """解析行内值：标量、{a: 1, b: 2} 或 [1, 2]。"""
    text = text.strip()
    if not text:
        return None
    if text[0] == "{" and text[-1] == "}":
        result = {}
        for item in _split_top_level(text[1:-1]):
            key, separator, value = item.partition(":")
            if not separator:
                raise ScenarioError("行内映射缺少冒号: %s" % item)
            result[_as_text(_parse_scalar(key))] = _parse_inline(value)
        return result
    if text[0] == "[" and text[-1] == "]":
        return [_parse_inline(item) for item in _split_top_level(text[1:-1])]
    return _parse_scalar(text)


def _tokenize(text):
    """把文本切成 (缩进, 内容, 行号) 的三元组，跳过空行与注释行。"""
    lines = []
    for number, raw in enumerate(text.splitlines(), 1):
        stripped = _strip_comment(raw.replace("\t", "  "))
        if not stripped.strip():
            continue
        if stripped.rstrip() != stripped.strip():
            pass
        indent = len(stripped) - len(stripped.lstrip(" "))
        lines.append((indent, stripped.strip(), number))
    return lines


def _parse_block(lines, index, indent):
    """从 index 处解析一个块（映射或列表），返回 (值, 下一个下标)。"""
    head = lines[index]
    is_list = head[1] == "-" or head[1].startswith("- ")
    if is_list:
        return _parse_list_block(lines, index, indent)
    return _parse_map_block(lines, index, indent)


def _parse_list_block(lines, index, indent):
    items = []
    while index < len(lines):
        current_indent, content, number = lines[index]
        if current_indent != indent:
            if current_indent > indent:
                raise ScenarioError("第 %d 行缩进比同级列表项更深" % number)
            break
        if content == "-":
            index += 1
            if index >= len(lines) or lines[index][0] <= indent:
                items.append(None)
                continue
            value, index = _parse_block(lines, index, lines[index][0])
            items.append(value)
            continue
        if not content.startswith("- "):
            break
        body = content[2:].strip()
        if body[:1] in ("{", "["):
            items.append(_parse_inline(body))
            index += 1
            continue
        if ":" in body:
            synthetic = [(indent + 2, body, number)]
            index += 1
            while index < len(lines) and lines[index][0] > indent:
                synthetic.append(lines[index])
                index += 1
            value, _ = _parse_map_block(synthetic, 0, indent + 2)
            items.append(value)
            continue
        items.append(_parse_inline(body))
        index += 1
    return items, index


def _parse_map_block(lines, index, indent):
    result = {}
    while index < len(lines):
        current_indent, content, number = lines[index]
        if current_indent < indent:
            break
        if current_indent > indent:
            raise ScenarioError("第 %d 行缩进异常" % number)
        if content == "-" or content.startswith("- "):
            break
        key_text, separator, rest = content.partition(":")
        if not separator:
            raise ScenarioError("第 %d 行不是 `键: 值` 形式: %s" % (number, content))
        key = _as_text(_parse_scalar(key_text))
        index += 1
        rest = rest.strip()
        if rest:
            result[key] = _parse_inline(rest)
            continue
        if index < len(lines) and lines[index][0] > indent:
            value, index = _parse_block(lines, index, lines[index][0])
            result[key] = value
        else:
            result[key] = None
    return result, index


def _as_text(value):
    if isinstance(value, bytes):
        return value.decode("utf-8")
    if value is None:
        return u""
    if isinstance(value, bool):
        return u"true" if value else u"false"
    return u"%s" % (value,)


def _as_float(value, fallback, field):
    if value is None:
        return fallback
    if isinstance(value, bool):
        raise ScenarioError("%s 不能是布尔值" % field)
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ScenarioError("%s 必须是数字，收到 %r" % (field, value))
    if math.isnan(result) or math.isinf(result):
        raise ScenarioError(u"%s 不能是 NaN / Inf" % field)
    return result


def _source_suffix(source):
    return u"（来源：%s）" % (source,) if source else u""


def build_scenario(body, source=""):
    """把解析出来的映射整理成 Scenario，并做完整性检查。"""
    name = _as_text(body.get("name") or u"unnamed")
    duration = _as_float(body.get("duration"), DEFAULT_DURATION, "duration")
    if duration <= 0:
        raise ScenarioError(u"duration 必须大于 0" + _source_suffix(source))

    initial = body.get("initial") or {}
    if not isinstance(initial, dict):
        raise ScenarioError(u"initial 必须是映射" + _source_suffix(source))
    pose = Pose(
        _as_float(initial.get("x"), 0.0, "initial.x"),
        _as_float(initial.get("y"), 0.0, "initial.y"),
        _as_float(initial.get("yaw"), 0.0, "initial.yaw"),
    )
    initial_speed = _as_float(initial.get("speed"), 0.0, "initial.speed")

    raw_steps = body.get("steps") or []
    if not isinstance(raw_steps, list):
        raise ScenarioError(u"steps 必须是列表" + _source_suffix(source))
    steps = []
    for index, item in enumerate(raw_steps):
        if not isinstance(item, dict):
            raise ScenarioError(u"steps 的第 %d 项必须是映射" % (index + 1))
        at = _as_float(item.get("at"), None, "steps.at")
        if at is None:
            raise ScenarioError(u"steps 的第 %d 项缺 at" % (index + 1))
        if at < 0:
            raise ScenarioError(u"steps 的 at 不能为负（第 %d 项）" % (index + 1))
        command = item.get("command") or {}
        if not isinstance(command, dict):
            raise ScenarioError(u"steps 的第 %d 项 command 必须是映射" % (index + 1))
        steps.append(
            Step(
                at,
                Command(
                    _as_float(command.get("linear"), 0.0, "command.linear"),
                    _as_float(command.get("angular"), 0.0, "command.angular"),
                ),
                _as_text(item.get("name") or u""),
            )
        )
    steps.sort(key=lambda step: step.at)
    for index in range(1, len(steps)):
        if steps[index].at <= steps[index - 1].at:
            raise ScenarioError(
                u"steps 的 at 必须严格递增（第 %d 项与第 %d 项都是 %.2f s）"
                % (index + 1, index, steps[index].at)
            )
    if steps and steps[-1].at > duration + 1e-9:
        raise ScenarioError(
            u"最后一条 steps 的 at=%.2f 超过了 duration=%.2f" % (steps[-1].at, duration)
        )

    raw_expects = body.get("expects") or []
    if not isinstance(raw_expects, list) or not raw_expects:
        raise ScenarioError(u"缺至少一条 expects 断言" + _source_suffix(source))
    expects = [Expect(item) for item in raw_expects]

    scenario = Scenario(
        name, duration, pose, steps, expects, source, initial_speed=initial_speed
    )
    return scenario


def parse_scenario_text(text, source=""):
    """解析场景文件文本。"""
    lines = _tokenize(text or u"")
    if not lines:
        raise ScenarioError(u"场景文件是空的" + _source_suffix(source))
    if lines[0][1].startswith("- ") or lines[0][1] == "-":
        raise ScenarioError(u"顶层必须是 `scenario:` 映射" + _source_suffix(source))
    data, _ = _parse_block(lines, 0, lines[0][0])
    if not isinstance(data, dict) or "scenario" not in data:
        raise ScenarioError(u"顶层缺少 `scenario:` 段" + _source_suffix(source))
    body = data["scenario"]
    if not isinstance(body, dict):
        raise ScenarioError(u"`scenario:` 段必须是映射" + _source_suffix(source))
    return build_scenario(body, source)


def load_scenario(path):
    """从磁盘读一份场景文件（UTF-8）。"""
    if not path:
        raise ScenarioError(u"没有指定场景文件路径")
    try:
        handle = io.open(path, "r", encoding="utf-8")
    except IOError as error:
        raise ScenarioError(u"打不开场景文件 %s：%s" % (path, error))
    try:
        text = handle.read()
    finally:
        handle.close()
    return parse_scenario_text(text, source=path)


# ---------------------------------------------------------------- 内置车辆


def advance(pose, speed, command, dt, profile):
    """按内置二维车辆模型推进一步：就地更新 pose，返回新的速度。

    节点在 mock 模式下和离线回归走的是同一个函数，所以两边结果一致。
    """
    target = max(-profile.max_speed, min(profile.max_speed, command.linear))
    if target > speed:
        speed = min(target, speed + profile.max_accel * dt)
    else:
        speed = max(target, speed - profile.max_decel * dt)
    yaw_rate = max(-profile.max_yaw_rate, min(profile.max_yaw_rate, command.angular))
    pose.x += speed * math.cos(pose.yaw) * dt
    pose.y += speed * math.sin(pose.yaw) * dt
    pose.yaw += yaw_rate * dt
    return speed


def simulate(scenario, profile=None, dt=DEFAULT_SIM_DT):
    """用内置的二维车辆模型把场景跑一遍，返回轨迹。"""
    profile = profile or VehicleProfile()
    if dt <= 0:
        raise ScenarioError(u"仿真步长 dt 必须大于 0")
    steps = int(round(scenario.duration / dt))
    if steps <= 0:
        raise ScenarioError(u"duration 太短，按 dt=%.3f 一步都跑不了" % dt)

    pose = Pose(scenario.initial.x, scenario.initial.y, scenario.initial.yaw)
    speed = scenario.initial_speed
    samples = [Sample(0.0, pose.x, pose.y, pose.yaw, speed)]
    elapsed = 0.0
    for _ in range(steps):
        speed = advance(pose, speed, scenario.command_at(elapsed), dt, profile)
        elapsed += dt
        samples.append(Sample(elapsed, pose.x, pose.y, pose.yaw, speed))
    return Trajectory(samples, scenario.name)


# ---------------------------------------------------------------- 断言判定


def normalize_angle(angle):
    """把角度折算到 (-pi, pi]。"""
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped < 0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def _check_max_speed(expect, trajectory):
    peak = trajectory.peak_speed()
    limit = expect.value + expect.tolerance
    return peak <= limit, u"峰值 %.2f m/s，上限 %.2f m/s" % (peak, limit)


def _check_reach_speed(expect, trajectory):
    peak = trajectory.peak_speed()
    target = expect.value - expect.tolerance
    return peak >= target, u"峰值 %.2f m/s，目标 %.2f m/s（容差 %.2f）" % (
        peak,
        expect.value,
        expect.tolerance,
    )


def _check_final_speed(expect, trajectory):
    final = trajectory.final().speed
    return (
        abs(final - expect.value) <= expect.tolerance,
        u"结束速度 %.2f m/s，期望 %.2f ± %.2f" % (
            final,
            expect.value,
            expect.tolerance,
        ),
    )


def _check_distance(expect, trajectory):
    travelled = trajectory.travelled()
    lower = -float("inf") if expect.min_value is None else expect.min_value
    upper = float("inf") if expect.max_value is None else expect.max_value
    text_lower = u"-inf" if expect.min_value is None else u"%.2f" % lower
    text_upper = u"+inf" if expect.max_value is None else u"%.2f" % upper
    return (
        lower <= travelled <= upper,
        u"行驶 %.2f m，期望区间 [%s, %s] m" % (travelled, text_lower, text_upper),
    )


def _check_position(expect, trajectory):
    final = trajectory.final()
    offset = math.hypot(final.x - expect.x, final.y - expect.y)
    return (
        offset <= expect.tolerance,
        u"结束于 (%.2f, %.2f)，目标 (%.2f, %.2f)，偏差 %.2f m（容差 %.2f）"
        % (final.x, final.y, expect.x, expect.y, offset, expect.tolerance),
    )


def _check_heading(expect, trajectory):
    final = trajectory.final()
    offset = normalize_angle(final.yaw - expect.value)
    return (
        abs(offset) <= expect.tolerance,
        u"结束航向 %.3f rad，期望 %.3f ± %.3f（偏差 %.3f）"
        % (final.yaw, expect.value, expect.tolerance, offset),
    )


_CHECKERS = {
    "max_speed": _check_max_speed,
    "reach_speed": _check_reach_speed,
    "final_speed": _check_final_speed,
    "distance": _check_distance,
    "position": _check_position,
    "heading": _check_heading,
}


def evaluate(trajectory, expects):
    """逐条判定断言，返回 Result 列表。"""
    results = []
    for index, expect in enumerate(expects):
        checker = _CHECKERS.get(expect.type)
        if checker is None:
            raise ScenarioError(u"不支持的断言类型 %s" % expect.type)
        passed, detail = checker(expect, trajectory)
        results.append(Result(index + 1, expect, bool(passed), detail))
    return results


def all_passed(results):
    return all(result.passed for result in results)


def format_report(scenario, results, header=u""):
    """把判定结果排成一段可以直接贴进 PR 的报告。"""
    passed = sum(1 for result in results if result.passed)
    lines = []
    if header:
        lines.append(header)
    lines.append(u"场景: %s" % scenario.name)
    if scenario.source:
        lines.append(u"来源: %s" % scenario.source)
    lines.append(
        u"断言: %d 通过 / %d 失败（共 %d 条）"
        % (passed, len(results) - passed, len(results))
    )
    for result in results:
        lines.append(
            u"  [%s] %-12s %s"
            % ("PASS" if result.passed else "FAIL", result.expect.type, result.detail)
        )
    lines.append(u"结论: %s" % (u"PASSED" if all_passed(results) else u"FAILED"))
    return u"\n".join(lines)


# ---------------------------------------------------------------- 参数


def as_bool(value, fallback=False):
    """把 rosparam 里的真值兼容成布尔（py2 下可能是 str / unicode）。"""
    if value is None:
        return fallback
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = _as_text(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off", u""):
        return False
    return fallback


class RunnerParams(object):
    """运行器参数，全部来自 rosparam。"""

    __slots__ = (
        "scenario_file",
        "publish_hz",
        "use_mock_odom",
        "sim_dt",
        "max_speed",
        "max_accel",
        "max_decel",
        "max_yaw_rate",
        "cmd_vel_topic",
        "odom_topic",
        "report_topic",
    )

    def __init__(
        self,
        scenario_file=u"",
        publish_hz=DEFAULT_PUBLISH_HZ,
        use_mock_odom=DEFAULT_USE_MOCK_ODOM,
        sim_dt=DEFAULT_SIM_DT,
        max_speed=DEFAULT_MAX_SPEED,
        max_accel=DEFAULT_MAX_ACCEL,
        max_decel=DEFAULT_MAX_DECEL,
        max_yaw_rate=DEFAULT_MAX_YAW_RATE,
        cmd_vel_topic=u"/cmd_vel",
        odom_topic=u"/odom",
        report_topic=u"/scenario/report",
    ):
        self.scenario_file = scenario_file
        self.publish_hz = float(publish_hz)
        self.use_mock_odom = bool(use_mock_odom)
        self.sim_dt = float(sim_dt)
        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)
        self.max_yaw_rate = float(max_yaw_rate)
        self.cmd_vel_topic = cmd_vel_topic
        self.odom_topic = odom_topic
        self.report_topic = report_topic
        if self.publish_hz <= 0:
            raise ScenarioError("rate/publish_hz 必须大于 0")
        if self.sim_dt <= 0:
            raise ScenarioError("control/sim_dt 必须大于 0")

    @classmethod
    def from_params(cls, get_param):
        def read(key, fallback):
            value = get_param(key, fallback)
            return fallback if value is None else value

        return cls(
            scenario_file=_as_text(read("scenario/file", u"")),
            publish_hz=_as_float(read("rate/publish_hz", DEFAULT_PUBLISH_HZ), DEFAULT_PUBLISH_HZ, "rate/publish_hz"),
            use_mock_odom=as_bool(read("control/use_mock_odom", DEFAULT_USE_MOCK_ODOM), DEFAULT_USE_MOCK_ODOM),
            sim_dt=_as_float(read("control/sim_dt", DEFAULT_SIM_DT), DEFAULT_SIM_DT, "control/sim_dt"),
            max_speed=_as_float(read("vehicle/max_speed", DEFAULT_MAX_SPEED), DEFAULT_MAX_SPEED, "vehicle/max_speed"),
            max_accel=_as_float(read("vehicle/max_accel", DEFAULT_MAX_ACCEL), DEFAULT_MAX_ACCEL, "vehicle/max_accel"),
            max_decel=_as_float(read("vehicle/max_decel", DEFAULT_MAX_DECEL), DEFAULT_MAX_DECEL, "vehicle/max_decel"),
            max_yaw_rate=_as_float(read("vehicle/max_yaw_rate", DEFAULT_MAX_YAW_RATE), DEFAULT_MAX_YAW_RATE, "vehicle/max_yaw_rate"),
            cmd_vel_topic=_as_text(read("topic/cmd_vel", u"/cmd_vel")),
            odom_topic=_as_text(read("topic/odom", u"/odom")),
            report_topic=_as_text(read("topic/report", u"/scenario/report")),
        )

    def profile(self):
        return VehicleProfile(
            self.max_speed, self.max_accel, self.max_decel, self.max_yaw_rate
        )

    def __repr__(self):
        return (
            "RunnerParams(file=%r, hz=%.1f, mock=%r, dt=%.3f)"
            % (self.scenario_file, self.publish_hz, self.use_mock_odom, self.sim_dt)
        )


def yaw_to_quaternion(yaw):
    """绕 z 轴旋转的四元数 (x, y, z, w)。"""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_to_yaw(x, y, z, w):
    """从四元数里取回航向角（只关心绕 z 的分量）。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def to_message_text(text):
    """把报告文本转成 std_msgs/String 能装的类型（py2 要 bytes，py3 要 str）。"""
    if isinstance(text, bytes):
        return text if str is bytes else text.decode("utf-8")
    if str is bytes:  # Python 2
        try:
            return text.encode("utf-8")
        except (UnicodeDecodeError, UnicodeEncodeError):
            return text
    return text
