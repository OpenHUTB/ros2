#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""倒车雷达分级告警的核心逻辑（纯标准库，不 import rospy、不 import yaml）。

设计要点：
* 四个后向超声通道，取最近的那个作为整车判定；
* 五级告警（无 / 提示 / 警告 / 危险 / 停止），进入与退出用**不同的**距离阈值（迟滞）；
* 升级必须连续 confirm_frames 帧都到线，降级必须连续 release_frames 帧都退出去，
  这样一次性的毛刺读数不会让提示音乱叫。
"""

from __future__ import division, print_function, unicode_literals

import io
import math

DEFAULT_ENTER_M = (2.00, 1.50, 1.00, 0.50)
DEFAULT_EXIT_MARGIN_M = 0.20
DEFAULT_CONFIRM_FRAMES = 3
DEFAULT_RELEASE_FRAMES = 3
DEFAULT_MIN_RANGE_M = 0.15
DEFAULT_MAX_RANGE_M = 3.00
DEFAULT_SENSOR_NAMES = (u"左后", u"左中", u"右中", u"右后")
DEFAULT_READ_HZ = 10.0
DEFAULT_PUBLISH_HZ = 10.0
DEFAULT_GRID_WIDTH = 60
DEFAULT_GRID_HEIGHT = 12
DEFAULT_SERVICE_SIMULATE = "/rear_alert/simulate"

LEVEL_NONE = 0
LEVEL_INFO = 1
LEVEL_WARN = 2
LEVEL_DANGER = 3
LEVEL_STOP = 4
LEVEL_NAMES = (u"无", u"提示", u"警告", u"危险", u"停止")

ASSERT_NAMES = (u"分级准确", u"无抖动", u"响应及时", u"覆盖全部通道")

PARAM_DEFAULTS = {
    "alert/enter_info_m": DEFAULT_ENTER_M[0],
    "alert/enter_warn_m": DEFAULT_ENTER_M[1],
    "alert/enter_danger_m": DEFAULT_ENTER_M[2],
    "alert/enter_stop_m": DEFAULT_ENTER_M[3],
    "alert/exit_margin_m": DEFAULT_EXIT_MARGIN_M,
    "alert/confirm_frames": DEFAULT_CONFIRM_FRAMES,
    "alert/release_frames": DEFAULT_RELEASE_FRAMES,
    "alert/min_range_m": DEFAULT_MIN_RANGE_M,
    "alert/max_range_m": DEFAULT_MAX_RANGE_M,
    "alert/max_switches": 6,
    "rate/read_hz": DEFAULT_READ_HZ,
    "rate/publish_hz": DEFAULT_PUBLISH_HZ,
    "topic/level": "/rear_alert/level",
    "topic/report": "/rear_alert/report",
    "topic/levels": "/rear_alert/levels",
    "topic/radar": "/rear_alert/radar",
    "service/simulate": DEFAULT_SERVICE_SIMULATE,
}

PARAM_KEYS = tuple(sorted(PARAM_DEFAULTS))


class AlertError(ValueError):
    pass


def _as_float(value, name, default=None):
    if value is None or (isinstance(value, str) and value.strip() == ""):
        if default is None:
            raise AlertError(u"参数 %s 不能为空" % (name,))
        return float(default)
    if isinstance(value, bool):
        return float(value)
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise AlertError(u"参数 %s 不是数字：%r" % (name, value))
    if number != number or number in (float("inf"), float("-inf")):
        raise AlertError(u"参数 %s 不是有限数：%r" % (name, value))
    return number


def _as_int(value, name, default=None):
    if value is None or (isinstance(value, str) and value.strip() == ""):
        if default is None:
            raise AlertError(u"参数 %s 不能为空" % (name,))
        return int(default)
    if isinstance(value, bool):
        raise AlertError(u"参数 %s 不能是布尔值" % (name,))
    if isinstance(value, float):
        if abs(value - round(value)) > 1e-9:
            raise AlertError(u"参数 %s 必须是整数：%r" % (name, value))
        return int(round(value))
    try:
        return int(value)
    except (TypeError, ValueError):
        raise AlertError(u"参数 %s 不是整数：%r" % (name, value))


def _positive(value, name):
    number = _as_float(value, name)
    if number <= 0.0:
        raise AlertError(u"参数 %s 必须大于 0：%r" % (name, value))
    return number


def _non_negative_int(value, name):
    number = _as_int(value, name)
    if number < 0:
        raise AlertError(u"参数 %s 不能为负：%r" % (name, value))
    return number


def distance_of(reading, index):
    return reading[1][index]


class Reading(object):
    """一帧读数：时刻 + 四个通道的距离（None 表示该通道没有回波）。"""

    __slots__ = ("t", "distances")

    def __init__(self, t, distances):
        self.t = _as_float(t, "t")
        self.distances = tuple(distances)
        if len(self.distances) != len(DEFAULT_SENSOR_NAMES):
            raise AlertError(u"每帧必须有 %d 个通道，实得 %d"
                             % (len(DEFAULT_SENSOR_NAMES), len(self.distances)))

    def valid(self):
        return [item for item in self.distances if item is not None]

    def nearest(self):
        values = self.valid()
        if not values:
            return None
        return min(values)

    def nearest_index(self):
        best = None
        best_index = -1
        for index, value in enumerate(self.distances):
            if value is None:
                continue
            if best is None or value < best:
                best = value
                best_index = index
        return best_index

    def __repr__(self):
        return "Reading(t=%.3f, %r)" % (self.t, self.distances)


class Grade(object):
    """一帧判定结果。"""

    __slots__ = ("t", "level", "sensor", "distance", "changed", "raw_level")

    def __init__(self, t, level, sensor, distance, changed, raw_level):
        self.t = t
        self.level = level
        self.sensor = sensor
        self.distance = distance
        self.changed = changed
        self.raw_level = raw_level

    def name(self):
        return LEVEL_NAMES[self.level]

    def as_row(self):
        return (self.t, self.level, self.sensor, self.distance)

    def __repr__(self):
        return "Grade(t=%.3f, level=%d, sensor=%d)" % (self.t, self.level,
                                                       self.sensor)


class Detector(object):
    """带迟滞与帧数的分级判定器。"""

    def __init__(self, enter=(DEFAULT_ENTER_M[0], DEFAULT_ENTER_M[1],
                             DEFAULT_ENTER_M[2], DEFAULT_ENTER_M[3]),
                 exit_margin=DEFAULT_EXIT_MARGIN_M,
                 confirm_frames=DEFAULT_CONFIRM_FRAMES,
                 release_frames=DEFAULT_RELEASE_FRAMES,
                 min_range=DEFAULT_MIN_RANGE_M,
                 max_range=DEFAULT_MAX_RANGE_M):
        self.enter = tuple(_positive(value, "enter") for value in enter)
        if len(self.enter) != 4:
            raise AlertError(u"必须有 4 个进入阈值，实得 %d" % (len(self.enter),))
        ordered = list(self.enter)
        ordered.sort(reverse=True)
        if list(self.enter) != ordered:
            raise AlertError(u"进入阈值必须从大到小：%r" % (self.enter,))
        self.exit_margin = _as_float(exit_margin, "exit_margin")
        if self.exit_margin < 0.0:
            raise AlertError(u"退出余量不能为负：%r" % (exit_margin,))
        self.confirm_frames = _as_int(confirm_frames, "confirm_frames")
        if self.confirm_frames < 1:
            raise AlertError(u"确认帧数至少 1：%r" % (confirm_frames,))
        self.release_frames = _as_int(release_frames, "release_frames")
        if self.release_frames < 1:
            raise AlertError(u"释放帧数至少 1：%r" % (release_frames,))
        self.min_range = _as_float(min_range, "min_range")
        self.max_range = _as_float(max_range, "max_range")
        if self.max_range <= self.min_range:
            raise AlertError(u"量程上限必须大于下限：%r / %r"
                             % (self.min_range, self.max_range))
        self.level = LEVEL_NONE
        self.sensor = -1
        self.distance = None
        self._pending_level = LEVEL_NONE
        self._pending_frames = 0
        self._release_count = 0

    # ---------------------------------------------------------------- 阈值
    def exit_threshold(self, level):
        """级别 level 的退出阈值（进入阈值 + 余量）。"""
        if level < 1 or level > 4:
            raise AlertError(u"级别超出范围：%r" % (level,))
        return self.enter[level - 1] + self.exit_margin

    def raw_level(self, distance):
        """不考虑迟滞时的即时级别。"""
        if distance is None:
            return LEVEL_NONE
        value = _as_float(distance, "distance")
        if value < self.min_range or value > self.max_range:
            return LEVEL_NONE
        level = LEVEL_NONE
        for index, threshold in enumerate(self.enter):
            if value <= threshold:
                level = index + 1
        return level

    # ---------------------------------------------------------------- 逐帧
    def reset(self):
        self.level = LEVEL_NONE
        self.sensor = -1
        self.distance = None
        self._pending_level = LEVEL_NONE
        self._pending_frames = 0
        self._release_count = 0

    def feed(self, t, distances):
        reading = Reading(t, distances)
        nearest = reading.nearest()
        index = reading.nearest_index()
        raw = self.raw_level(nearest)
        before = self.level

        if raw > self.level:
            if raw == self._pending_level:
                self._pending_frames += 1
            else:
                self._pending_level = raw
                self._pending_frames = 1
            if self._pending_frames >= self.confirm_frames:
                self.level = raw
                self._pending_level = LEVEL_NONE
                self._pending_frames = 0
                self._release_count = 0
        elif raw < self.level:
            # 只有真的退出了当前级别的退出阈值，才开始数释放帧
            beyond = nearest is None or nearest > self.exit_threshold(self.level)
            if beyond:
                self._release_count += 1
                if self._release_count >= self.release_frames:
                    self.level = raw
                    self._release_count = 0
                    self._pending_level = LEVEL_NONE
                    self._pending_frames = 0
            else:
                self._release_count = 0
        else:
            self._pending_level = LEVEL_NONE
            self._pending_frames = 0
            self._release_count = 0

        if self.level > LEVEL_NONE:
            self.sensor = index
            self.distance = nearest
        elif self.level == LEVEL_NONE:
            self.sensor = index if nearest is not None else -1
            self.distance = nearest

        return Grade(reading.t, self.level, self.sensor, nearest,
                     self.level != before, raw)


class Trace(object):
    """一串判定结果。"""

    __slots__ = ("grades", "name")

    def __init__(self, grades, name=""):
        self.grades = list(grades)
        self.name = name

    def __len__(self):
        return len(self.grades)

    def count(self):
        return len(self.grades)

    def is_empty(self):
        return not self.grades

    def duration(self):
        if not self.grades:
            return 0.0
        return self.grades[-1].t - self.grades[0].t

    def levels(self):
        return [item.level for item in self.grades]

    def final(self):
        if not self.grades:
            raise AlertError(u"轨迹为空，没有末帧")
        return self.grades[-1]

    def max_level(self):
        return max(self.levels()) if self.grades else LEVEL_NONE

    def switches(self):
        return sum(1 for item in self.grades if item.changed)

    def first_at(self, level):
        for index, item in enumerate(self.grades):
            if item.level >= level:
                return index
        return -1

    def kept(self, level):
        count = 0
        for item in self.grades:
            if item.level == level:
                count += 1
        return count

    def sensors_used(self):
        return sorted(set(item.sensor for item in self.grades
                          if item.level > LEVEL_NONE and item.sensor >= 0))

    def chatter(self, window=6):
        """来回振荡计数：方向相反且间隔小于 window 帧的切换对。"""
        marks = [index for index, item in enumerate(self.grades) if item.changed]
        if not marks:
            return 0
        last = self.grades[0].level
        steps = []
        for index in marks:
            level = self.grades[index].level
            steps.append((index, 1 if level > last else -1))
            last = level
        count = 0
        for first, second in zip(steps, steps[1:]):
            if first[1] * second[1] < 0 and second[0] - first[0] < window:
                count += 1
        return count

    def timeline(self):
        return [(item.t, item.level) for item in self.grades]

    def as_rows(self):
        return [item.as_row() for item in self.grades]

    def __repr__(self):
        return "Trace(count=%d, name=%r)" % (len(self.grades), self.name)


def parse_readings_text(text, source=""):
    """解析脚本化读数：每行 `t d1 d2 d3 d4`，`-` 表示该通道没有回波。"""
    readings = []
    lines = text.splitlines()
    for number, line in enumerate(lines, start=1):
        raw = line.split("#", 1)[0].strip()
        if not raw:
            continue
        parts = raw.replace(",", " ").replace("\t", " ").split()
        if len(parts) != 5:
            raise AlertError(u"%s 第 %d 行需要 5 个字段（t + 4 个通道），实得 %d"
                             % (source or u"读数", number, len(parts)))
        t = _as_float(parts[0], u"%s 第 %d 行的 t" % (source or u"读数", number))
        values = []
        for offset, token in enumerate(parts[1:], start=1):
            if token in ("-", "None", "null", "nan"):
                values.append(None)
            else:
                values.append(_as_float(
                    token, u"%s 第 %d 行第 %d 个通道"
                    % (source or u"读数", number, offset)))
        readings.append(Reading(t, values))
    if not readings:
        raise AlertError(u"%s 里没有任何读数" % (source or u"读数",))
    return readings


DEMO_READINGS_TEXT = u"""\
# 倒车接近 → 贴住 → 缓慢退出（中途左后通道丢一帧回波）
0.0  2.90 3.00 2.95 3.00
0.1  2.60 2.70 2.65 2.75
0.2  2.20 2.30 2.25 2.35
0.3  1.90 2.00 1.95 2.05
0.4  1.60 1.70 1.65 1.75
0.5  1.30 1.40 1.35 1.45
0.6  1.05 1.15 1.10 1.20
0.7  0.85 0.95 0.90 1.00
0.8  0.62 0.72 0.67 0.77
0.9  0.45 0.55 0.50 0.60
1.0  0.38 0.48 0.43 0.53
1.1  0.36 0.46 0.41 0.51
1.2  0.35 0.45 0.40 0.50
1.3  2.90 0.47 2.95 3.00
1.4  0.36 0.44 0.39 0.49
1.5  0.55 0.65 0.60 0.70
1.6  0.95 1.05 1.00 1.10
1.7  1.45 1.55 1.50 1.60
1.8  1.90 2.00 1.95 2.05
1.9  2.40 2.50 2.45 2.55
2.0  2.80 2.90 2.85 2.95
2.1  3.00 3.00 3.00 3.00
2.2  3.00 3.00 3.00 3.00
"""


def demo_readings():
    return parse_readings_text(DEMO_READINGS_TEXT, u"内置演示读数")


def run_detector(readings, detector=None, name=""):
    if detector is None:
        detector = Detector()
    else:
        detector.reset()
    grades = [detector.feed(item.t, item.distances) for item in readings]
    return Trace(grades, name)


def check_alert(trace, readings, detector=None, max_switches=6):
    """四条断言：分级准确 / 无抖动 / 响应及时 / 覆盖全部通道。"""
    if detector is None:
        detector = Detector()
    results = []

    raw_levels = [detector.raw_level(item.nearest()) for item in readings]
    expected_max = max(raw_levels) if raw_levels else LEVEL_NONE
    expected_final = raw_levels[-1] if raw_levels else LEVEL_NONE
    peak_ok = trace.max_level() == expected_max
    final_ok = trace.final().level == expected_final
    results.append({
        "type": u"分级准确",
        "passed": bool(peak_ok and final_ok),
        "detail": u"最高级别 %d（期望 %d），末帧级别 %d（期望 %d）"
                  % (trace.max_level(), expected_max,
                     trace.final().level, expected_final),
    })

    switches = trace.switches()
    chatter = trace.chatter(window=max(2, detector.confirm_frames))
    results.append({
        "type": u"无抖动",
        "passed": bool(switches <= max_switches and chatter == 0),
        "detail": u"切换 %d 次（上限 %d），%d 帧内反复切换 %d 次"
                  % (switches, max_switches, detector.confirm_frames, chatter),
    })

    top = trace.max_level()
    first_raw = -1
    for index, value in enumerate(raw_levels):
        if value >= top:
            first_raw = index
            break
    first_grade = trace.first_at(top) if top > LEVEL_NONE else -1
    if top == LEVEL_NONE:
        delay = 0
        on_time = True
    else:
        delay = first_grade - first_raw if first_grade >= 0 else -1
        on_time = 0 <= delay <= detector.confirm_frames
    results.append({
        "type": u"响应及时",
        "passed": bool(on_time),
        "detail": u"最高级别 %d 在第 %d 帧判定（原始第 %d 帧），延迟 %d 帧（上限 %d）"
                  % (top, first_grade, first_raw, delay,
                     detector.confirm_frames),
    })

    coverage = []
    for index in range(len(DEFAULT_SENSOR_NAMES)):
        hit = False
        for item in readings:
            if detector.raw_level(item.distances[index]) > LEVEL_NONE:
                hit = True
                break
        coverage.append(hit)
    covered = sum(1 for flag in coverage if flag)
    results.append({
        "type": u"覆盖全部通道",
        "passed": covered == len(DEFAULT_SENSOR_NAMES),
        "detail": u"四个通道里进入过告警区间的有 %d 个" % (covered,),
    })
    return results


def all_passed(results):
    return all(item["passed"] for item in results)


def render_levels(trace, height=DEFAULT_GRID_HEIGHT,
                  width=DEFAULT_GRID_WIDTH):
    """按时间画一条级别条带：行 = 级别（4 在上），列 = 帧。"""
    levels = trace.levels()
    if not levels:
        return [u" " * width for _ in range(height)]
    span = max(1, len(levels))
    rows = []
    for row_index in range(height):
        level = height - 1 - row_index
        if level > 4:
            rows.append(u" " * width)
            continue
        cells = []
        for column in range(width):
            start = int(column * span / width)
            end = max(start + 1, int((column + 1) * span / width))
            chunk = levels[start:end]
            if not chunk:
                cells.append(u" ")
            elif level == 0:
                cells.append(u"-" if max(chunk) == 0 else u" ")
            elif max(chunk) >= level:
                cells.append(u"#")
            else:
                cells.append(u" ")
        rows.append(u"".join(cells))
    return rows


def render_radar(distances, width=DEFAULT_GRID_WIDTH):
    """把四个通道画成四条距离条：越近条越短，末列标注级别。"""
    rows = []
    for index, name in enumerate(DEFAULT_SENSOR_NAMES):
        value = distances[index]
        if value is None:
            bar = u"-" * 0
            label = u"无回波"
            cells = u" " * (width - 12)
        else:
            usable = max(0.0, min(1.0, value / 3.0))
            filled = int(round((1.0 - usable) * (width - 16)))
            filled = max(0, min(width - 16, filled))
            cells = u"#" * filled + u"." * (width - 16 - filled)
            label = u"%4.2f m" % (value,)
        rows.append(u"%-4s [%s] %s" % (name, cells, label))
    return rows


def format_report(trace, readings, detector, results, header=""):
    lines = []
    if header:
        lines.append(header)
    lines.append(u"倒车雷达分级告警（四个后向通道，带迟滞）")
    lines.append(u"进入阈值：%s m，退出余量 %.2f m，确认 %d 帧 / 释放 %d 帧"
                 % (u" / ".join(u"%.2f" % (value,) for value in detector.enter),
                    detector.exit_margin, detector.confirm_frames,
                    detector.release_frames))
    lines.append(u"-" * 60)
    lines.append(u"帧数：%d，时长：%.2f s" % (len(trace), trace.duration()))
    lines.append(u"最高级别：%d（%s），末帧级别：%d（%s）"
                 % (trace.max_level(), LEVEL_NAMES[trace.max_level()],
                    trace.final().level, LEVEL_NAMES[trace.final().level]))
    lines.append(u"级别切换：%d 次" % (trace.switches(),))
    marks = [item for item in trace.grades if item.changed]
    for item in marks:
        lines.append(u"  t=%.2f s -> %s（通道 %s，%.2f m）"
                     % (item.t, item.name(),
                        DEFAULT_SENSOR_NAMES[item.sensor]
                        if item.sensor >= 0 else u"-",
                        item.distance if item.distance is not None else -1.0))
    passed = sum(1 for item in results if item["passed"])
    lines.append(u"断言：%d 通过 / %d 失败（共 %d 条）"
                 % (passed, len(results) - passed, len(results)))
    for item in results:
        lines.append(u"  [%s] %s %s" % (u"PASS" if item["passed"] else u"FAIL",
                                        item["type"], item["detail"]))
    lines.append(u"结论：%s" % (u"PASSED" if all_passed(results) else u"FAILED",))
    return u"\n".join(lines)


def to_message_text(text):
    if isinstance(text, bytes):
        return text
    return text.encode("utf-8")


class AlertParams(object):
    """把 ROS 参数读成一个对象；键名与 PARAM_KEYS 一一对应。"""

    def __init__(self, **kwargs):
        for key in PARAM_KEYS:
            value = kwargs.get(key, PARAM_DEFAULTS[key])
            setattr(self, self._attr(key), value)
        self.alert_confirm_frames = _as_int(self.alert_confirm_frames,
                                            "alert_confirm_frames")
        self.alert_release_frames = _as_int(self.alert_release_frames,
                                            "alert_release_frames")
        self.alert_max_switches = _as_int(self.alert_max_switches,
                                          "alert_max_switches")
        self.max_switches = self.alert_max_switches
        self.rate_read_hz = _positive(self.rate_read_hz, "rate_read_hz")
        self.rate_publish_hz = _positive(self.rate_publish_hz, "rate_publish_hz")
        for name in ("alert_enter_info_m", "alert_enter_warn_m",
                     "alert_enter_danger_m", "alert_enter_stop_m"):
            setattr(self, name, _positive(getattr(self, name), name))
        self.detector = Detector(
            enter=(self.alert_enter_info_m, self.alert_enter_warn_m,
                   self.alert_enter_danger_m, self.alert_enter_stop_m),
            exit_margin=self.alert_exit_margin_m,
            confirm_frames=self.alert_confirm_frames,
            release_frames=self.alert_release_frames,
            min_range=self.alert_min_range_m,
            max_range=self.alert_max_range_m,
        )

    @staticmethod
    def _attr(key):
        return key.replace("/", "_")

    @classmethod
    def from_params(cls, get_param):
        values = {}
        for key in PARAM_KEYS:
            values[key] = get_param(key, PARAM_DEFAULTS[key])
        return cls(**values)

    def as_dict(self):
        return dict((key, getattr(self, self._attr(key))) for key in PARAM_KEYS)

    def __repr__(self):
        return "AlertParams(%r)" % (self.as_dict(),)

def summary_text(trace, results):
    """一行 ASCII 摘要，专门给 `std_srvs/Trigger` 的 message 用。

    `rosservice call` 会把 non-ASCII 的字符串转义成 `\\u5012\\u8F66` 这种形式，
    截图里根本没法看，所以服务只回这一行纯 ASCII；完整报告仍然走日志与
    `/rear_alert/report` 话题。
    """
    passed = sum(1 for item in results if item.get("passed"))
    last_level = trace.final()
    if hasattr(last_level, "level"):
        last_level = last_level.level
    return (u"max_level=%s final_level=%s switches=%s asserts=%d/%d passed=%s"
            % (trace.max_level(), last_level, trace.switches(),
               passed, len(results), all_passed(results)))
