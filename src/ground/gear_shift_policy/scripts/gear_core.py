#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""换挡策略与扭矩需求的核心逻辑（纯标准库，不 import rospy、不 import yaml）。

三件事：
* **换挡判据**用发动机转速而不是车速：升挡线 2600 rpm、降挡线 1500 rpm，
  两个线之间的差值就是迟滞，避免在临界车速上来回换挡；
* **挡位保持时间**：每次换挡后至少待满 `min_dwell` 秒，防止两挡之间抖动；
* **扭矩需求**按纵向力平衡反推：`F = m·a + ½ρCdA·v² + m·g·Crr`，
  再乘车轮半径、除以总传动比与效率，最后与发动机外特性比对。
"""

from __future__ import division, print_function, unicode_literals

import io
import math

DEFAULT_MASS = 1500.0
DEFAULT_WHEEL_RADIUS = 0.31
DEFAULT_FINAL_DRIVE = 4.10
DEFAULT_GEAR_RATIOS = (3.55, 2.10, 1.40, 1.00)
DEFAULT_EFFICIENCY = 0.92
DEFAULT_IDLE_RPM = 800.0
DEFAULT_REDLINE_RPM = 6500.0
DEFAULT_UPSHIFT_RPM = 2600.0
DEFAULT_DOWNSHIFT_RPM = 1500.0
DEFAULT_MIN_DWELL = 0.50
DEFAULT_DRAG_COEFF = 0.32
DEFAULT_FRONTAL_AREA = 2.20
DEFAULT_AIR_DENSITY = 1.20
DEFAULT_ROLLING = 0.015
DEFAULT_GRAVITY = 9.81
DEFAULT_PEAK_TORQUE = 220.0
DEFAULT_PEAK_RPM = 4200.0
DEFAULT_ACCEL = 1.00
DEFAULT_MAX_SPEED = 30.00
DEFAULT_HOLD_SECONDS = 2.00
DEFAULT_DECEL = 2.00
DEFAULT_SIM_STEP = 0.05
DEFAULT_PUBLISH_HZ = 10.0
DEFAULT_GRID_WIDTH = 60
DEFAULT_GRID_HEIGHT = 14
GEAR_COUNT = 4

ASSERT_NAMES = (u"挡位覆盖", u"换挡平顺", u"转速区间", u"扭矩上限")

PARAM_DEFAULTS = {
    "vehicle/mass": DEFAULT_MASS,
    "vehicle/wheel_radius": DEFAULT_WHEEL_RADIUS,
    "vehicle/frontal_area": DEFAULT_FRONTAL_AREA,
    "vehicle/drag_coeff": DEFAULT_DRAG_COEFF,
    "vehicle/rolling": DEFAULT_ROLLING,
    "gearbox/final_drive": DEFAULT_FINAL_DRIVE,
    "gearbox/efficiency": DEFAULT_EFFICIENCY,
    "gearbox/ratio_1": DEFAULT_GEAR_RATIOS[0],
    "gearbox/ratio_2": DEFAULT_GEAR_RATIOS[1],
    "gearbox/ratio_3": DEFAULT_GEAR_RATIOS[2],
    "gearbox/ratio_4": DEFAULT_GEAR_RATIOS[3],
    "policy/upshift_rpm": DEFAULT_UPSHIFT_RPM,
    "policy/downshift_rpm": DEFAULT_DOWNSHIFT_RPM,
    "policy/min_dwell": DEFAULT_MIN_DWELL,
    "policy/idle_rpm": DEFAULT_IDLE_RPM,
    "policy/redline_rpm": DEFAULT_REDLINE_RPM,
    "engine/peak_torque": DEFAULT_PEAK_TORQUE,
    "engine/peak_rpm": DEFAULT_PEAK_RPM,
    "profile/accel": DEFAULT_ACCEL,
    "profile/max_speed": DEFAULT_MAX_SPEED,
    "profile/hold_seconds": DEFAULT_HOLD_SECONDS,
    "profile/decel": DEFAULT_DECEL,
    "sim/step": DEFAULT_SIM_STEP,
    "rate/publish_hz": DEFAULT_PUBLISH_HZ,
    "topic/gear": u"/gear/current",
    "topic/report": u"/gear/report",
    "topic/curve": u"/gear/curve",
    "topic/shift": u"/gear/shift",
}

PARAM_KEYS = tuple(sorted(PARAM_DEFAULTS))


class GearError(ValueError):
    pass


def _as_float(value, name, default=None):
    if value is None or (isinstance(value, str) and value.strip() == ""):
        if default is None:
            raise GearError(u"参数 %s 不能为空" % (name,))
        return float(default)
    if isinstance(value, bool):
        return float(value)
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise GearError(u"参数 %s 不是数字：%r" % (name, value))
    if number != number or number in (float("inf"), float("-inf")):
        raise GearError(u"参数 %s 不是有限数：%r" % (name, value))
    return number


def _positive(value, name):
    number = _as_float(value, name)
    if number <= 0.0:
        raise GearError(u"参数 %s 必须大于 0：%r" % (name, value))
    return number


def _non_negative(value, name):
    number = _as_float(value, name)
    if number < 0.0:
        raise GearError(u"参数 %s 不能为负：%r" % (name, value))
    return number


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


class Gearbox(object):
    """变速箱与发动机外特性。"""

    __slots__ = ("ratios", "final_drive", "efficiency", "wheel_radius",
                 "idle_rpm", "redline_rpm", "upshift_rpm", "downshift_rpm",
                 "peak_torque", "peak_rpm")

    def __init__(self, ratios=DEFAULT_GEAR_RATIOS,
                 final_drive=DEFAULT_FINAL_DRIVE,
                 efficiency=DEFAULT_EFFICIENCY,
                 wheel_radius=DEFAULT_WHEEL_RADIUS,
                 idle_rpm=DEFAULT_IDLE_RPM,
                 redline_rpm=DEFAULT_REDLINE_RPM,
                 upshift_rpm=DEFAULT_UPSHIFT_RPM,
                 downshift_rpm=DEFAULT_DOWNSHIFT_RPM,
                 peak_torque=DEFAULT_PEAK_TORQUE,
                 peak_rpm=DEFAULT_PEAK_RPM):
        self.ratios = tuple(_positive(v, "ratio") for v in ratios)
        if len(self.ratios) != GEAR_COUNT:
            raise GearError(u"必须有 %d 个挡位，实得 %d"
                            % (GEAR_COUNT, len(self.ratios)))
        ordered = list(self.ratios)
        if ordered != sorted(ordered, reverse=True):
            raise GearError(u"挡位传动比必须从大到小：%r" % (self.ratios,))
        self.final_drive = _positive(final_drive, "final_drive")
        self.efficiency = _positive(efficiency, "efficiency")
        if self.efficiency > 1.0:
            raise GearError(u"效率不能大于 1：%r" % (efficiency,))
        self.wheel_radius = _positive(wheel_radius, "wheel_radius")
        self.idle_rpm = _positive(idle_rpm, "idle_rpm")
        self.redline_rpm = _positive(redline_rpm, "redline_rpm")
        if self.redline_rpm <= self.idle_rpm:
            raise GearError(u"红线转速必须高于怠速：%r / %r"
                            % (self.redline_rpm, self.idle_rpm))
        self.upshift_rpm = _positive(upshift_rpm, "upshift_rpm")
        self.downshift_rpm = _positive(downshift_rpm, "downshift_rpm")
        if self.upshift_rpm <= self.downshift_rpm:
            raise GearError(u"升挡转速必须高于降挡转速：%r / %r"
                            % (self.upshift_rpm, self.downshift_rpm))
        self.peak_torque = _positive(peak_torque, "peak_torque")
        self.peak_rpm = _positive(peak_rpm, "peak_rpm")

    # ------------------------------------------------------------- 转速换算
    def total_ratio(self, gear):
        if gear < 1 or gear > GEAR_COUNT:
            raise GearError(u"挡位超出范围：%r" % (gear,))
        return self.ratios[gear - 1] * self.final_drive

    def rpm_for(self, speed, gear):
        ratio = self.total_ratio(gear)
        return speed * ratio * 60.0 / (2.0 * math.pi * self.wheel_radius)

    def speed_for(self, rpm, gear):
        ratio = self.total_ratio(gear)
        return rpm * 2.0 * math.pi * self.wheel_radius / (60.0 * ratio)

    def max_torque(self, rpm):
        """简化的发动机外特性：峰值以下接近平台，峰值以上抛物线衰减。"""
        if rpm <= 0.0:
            return 0.0
        if rpm <= self.peak_rpm:
            ratio = min(1.0, rpm / self.peak_rpm)
            shape = 0.60 + 0.40 * ratio
        else:
            span = max(self.redline_rpm - self.peak_rpm, 1.0)
            offset = (rpm - self.peak_rpm) / span
            shape = max(0.35, 1.0 - 0.55 * offset * offset)
        return max(self.peak_torque * 0.35, self.peak_torque * shape)

    def as_tuple(self):
        return (self.ratios, self.final_drive, self.efficiency,
                self.wheel_radius, self.upshift_rpm, self.downshift_rpm)

    def __repr__(self):
        return "Gearbox(ratios=%r, final=%.2f)" % (self.ratios,
                                                   self.final_drive)


class Vehicle(object):
    """纵向阻力模型。"""

    __slots__ = ("mass", "frontal_area", "drag_coeff", "rolling")

    def __init__(self, mass=DEFAULT_MASS, frontal_area=DEFAULT_FRONTAL_AREA,
                 drag_coeff=DEFAULT_DRAG_COEFF, rolling=DEFAULT_ROLLING):
        self.mass = _positive(mass, "mass")
        self.frontal_area = _positive(frontal_area, "frontal_area")
        self.drag_coeff = _positive(drag_coeff, "drag_coeff")
        self.rolling = _non_negative(rolling, "rolling")

    def resistance(self, speed):
        air = 0.5 * DEFAULT_AIR_DENSITY * self.drag_coeff * self.frontal_area \
            * speed * speed
        roll = self.mass * DEFAULT_GRAVITY * self.rolling
        return air + roll

    def force(self, speed, accel):
        return self.mass * accel + self.resistance(speed)

    def as_tuple(self):
        return (self.mass, self.frontal_area, self.drag_coeff, self.rolling)


class Sample(object):
    """一帧：时刻、车速、纵向加速度、挡位、转速、发动机扭矩。"""

    __slots__ = ("t", "speed", "accel", "gear", "rpm", "torque")

    def __init__(self, t, speed, accel, gear, rpm, torque):
        self.t = t
        self.speed = speed
        self.accel = accel
        self.gear = gear
        self.rpm = rpm
        self.torque = torque

    def as_row(self):
        return (self.t, self.speed, self.gear, self.rpm, self.torque)

    def __repr__(self):
        return "Sample(t=%.2f, v=%.2f, gear=%d, rpm=%.0f, T=%.1f)" % (
            self.t, self.speed, self.gear, self.rpm, self.torque)


class Shift(object):
    """一次换挡事件。"""

    __slots__ = ("t", "from_gear", "to_gear", "speed", "rpm_before", "rpm_after")

    def __init__(self, t, from_gear, to_gear, speed, rpm_before, rpm_after):
        self.t = t
        self.from_gear = from_gear
        self.to_gear = to_gear
        self.speed = speed
        self.rpm_before = rpm_before
        self.rpm_after = rpm_after

    def upward(self):
        return self.to_gear > self.from_gear

    def as_row(self):
        return (self.t, self.from_gear, self.to_gear, self.speed,
                self.rpm_before, self.rpm_after)

    def __repr__(self):
        return "Shift(t=%.2f, %d->%d)" % (self.t, self.from_gear, self.to_gear)


class Trace(object):
    """一整段驾驶过程。"""

    __slots__ = ("samples", "shifts", "gearbox", "vehicle", "name")

    def __init__(self, samples, shifts, gearbox, vehicle, name=""):
        self.samples = list(samples)
        self.shifts = list(shifts)
        self.gearbox = gearbox
        self.vehicle = vehicle
        self.name = name

    def __len__(self):
        return len(self.samples)

    def count(self):
        return len(self.samples)

    def is_empty(self):
        return not self.samples

    def duration(self):
        if not self.samples:
            return 0.0
        return self.samples[-1].t - self.samples[0].t

    def gears(self):
        return [item.gear for item in self.samples]

    def gears_used(self):
        return sorted(set(self.gears()))

    def top_speed(self):
        return max(item.speed for item in self.samples) if self.samples else 0.0

    def peak_rpm(self):
        return max(item.rpm for item in self.samples) if self.samples else 0.0

    def peak_torque(self):
        return max(item.torque for item in self.samples) if self.samples else 0.0

    def min_dwell(self):
        if len(self.shifts) < 2:
            return float("inf")
        gaps = [second.t - first.t
                for first, second in zip(self.shifts, self.shifts[1:])]
        return min(gaps)

    def hunting(self, window=1.0):
        """A→B→A 的来回换挡次数（间隔小于 window 秒）。"""
        count = 0
        for first, second in zip(self.shifts, self.shifts[1:]):
            if second.t - first.t >= window:
                continue
            if second.to_gear == first.from_gear:
                count += 1
        return count

    def rpm_out_of_range(self):
        low = self.gearbox.idle_rpm
        high = self.gearbox.redline_rpm
        return [item for item in self.samples
                if item.rpm < low * 0.5 or item.rpm > high]

    def torque_over_limit(self, margin=1e-6):
        bad = []
        for item in self.samples:
            limit = self.gearbox.max_torque(item.rpm)
            if item.torque > limit + margin:
                bad.append(item)
        return bad

    def shift_table(self):
        return [(item.t, item.from_gear, item.to_gear, item.speed,
                 item.rpm_before, item.rpm_after) for item in self.shifts]

    def as_rows(self):
        return [item.as_row() for item in self.samples]

    def __repr__(self):
        return "Trace(count=%d, shifts=%d)" % (len(self.samples),
                                               len(self.shifts))


def build_profile(accel=DEFAULT_ACCEL, max_speed=DEFAULT_MAX_SPEED,
                  hold=DEFAULT_HOLD_SECONDS, decel=DEFAULT_DECEL):
    """加速 → 定速保持 → 减速的三段速度剖面，返回 (segments, duration)。"""
    accel = _positive(accel, "accel")
    max_speed = _positive(max_speed, "max_speed")
    hold = _non_negative(hold, "hold")
    decel = _positive(decel, "decel")
    t1 = max_speed / accel
    t2 = t1 + hold
    t3 = t2 + max_speed / decel
    segments = (
        (0.0, t1, 0.0, max_speed, accel),
        (t1, t2, max_speed, max_speed, 0.0),
        (t2, t3, max_speed, 0.0, -decel),
    )
    return segments, t3


def speed_at(segments, t):
    if t <= segments[0][0]:
        return segments[0][2]
    for start, end, v0, v1, rate in segments:
        if start <= t <= end:
            return v0 + rate * (t - start)
    return segments[-1][3]


def accel_at(segments, t):
    if t <= segments[0][0]:
        return segments[0][4]
    for start, end, _v0, _v1, rate in segments:
        if start <= t <= end:
            return rate
    return 0.0


def shift_decision(gearbox, speed, gear, dwell_elapsed):
    """返回目标挡位：转速过升挡线就升，过低降挡线就降，但必须先待够 dwell。"""
    if dwell_elapsed < 0.0:
        return gear
    rpm = gearbox.rpm_for(speed, gear)
    if rpm > gearbox.upshift_rpm and gear < GEAR_COUNT:
        return gear + 1
    if rpm < gearbox.downshift_rpm and gear > 1:
        return gear - 1
    return gear


def simulate(gearbox=None, vehicle=None, segments=None, step=DEFAULT_SIM_STEP,
             min_dwell=DEFAULT_MIN_DWELL, name=""):
    if gearbox is None:
        gearbox = Gearbox()
    if vehicle is None:
        vehicle = Vehicle()
    if segments is None:
        segments, _duration = build_profile()
    step = _positive(step, "step")
    min_dwell = _non_negative(min_dwell, "min_dwell")
    duration = segments[-1][1]

    samples = []
    shifts = []
    gear = 1
    last_shift_t = -min_dwell
    index = 0
    while True:
        t = index * step
        if t > duration + 1e-9:
            break
        speed = max(0.0, speed_at(segments, t))
        accel = accel_at(segments, t)
        dwell_elapsed = t - last_shift_t - min_dwell
        target = shift_decision(gearbox, speed, gear, dwell_elapsed)
        if target != gear:
            rpm_before = gearbox.rpm_for(speed, gear)
            rpm_after = gearbox.rpm_for(speed, target)
            shifts.append(Shift(t, gear, target, speed, rpm_before, rpm_after))
            gear = target
            last_shift_t = t
        # 起步与极低速时靠离合器打滑，发动机不会低于怠速
        rpm = max(gearbox.idle_rpm, gearbox.rpm_for(speed, gear))
        force = vehicle.force(speed, accel)
        wheel_torque = force * gearbox.wheel_radius
        engine_torque = wheel_torque / (gearbox.total_ratio(gear)
                                        * gearbox.efficiency)
        engine_torque = max(0.0, engine_torque)
        samples.append(Sample(t, speed, accel, gear, rpm, engine_torque))
        index += 1
    return Trace(samples, shifts, gearbox, vehicle, name)


def check_shift(trace, min_dwell=DEFAULT_MIN_DWELL, expected_gears=None):
    """四条断言：挡位覆盖 / 换挡平顺 / 转速区间 / 扭矩上限。"""
    results = []

    used = trace.gears_used()
    if expected_gears is None:
        expected_gears = GEAR_COUNT
    results.append({
        "type": u"挡位覆盖",
        "passed": len(used) >= expected_gears,
        "detail": u"用到的挡位 %r（期望至少 %d 个）" % (used, expected_gears),
    })

    dwell = trace.min_dwell()
    window = max(2.0 * min_dwell, 1.0)
    hunting = trace.hunting(window)
    ok = (dwell >= min_dwell - 1e-9) and hunting == 0
    results.append({
        "type": u"换挡平顺",
        "passed": bool(ok),
        "detail": u"最短保持 %.2f s（要求 >= %.2f s），%.1f s 内来回换挡 %d 次"
                  % (dwell if dwell != float("inf") else 0.0, min_dwell,
                     window, hunting),
    })

    bad = trace.rpm_out_of_range()
    results.append({
        "type": u"转速区间",
        "passed": not bad,
        "detail": u"越界 %d 帧（怠速 %.0f，红线 %.0f，实测最高 %.0f）"
                  % (len(bad), trace.gearbox.idle_rpm,
                     trace.gearbox.redline_rpm, trace.peak_rpm()),
    })

    over = trace.torque_over_limit()
    results.append({
        "type": u"扭矩上限",
        "passed": not over,
        "detail": u"超限 %d 帧（峰值需求 %.1f N·m）"
                  % (len(over), trace.peak_torque()),
    })
    return results


def all_passed(results):
    return all(item["passed"] for item in results)


def render_gears(trace, width=DEFAULT_GRID_WIDTH, height=DEFAULT_GRID_HEIGHT):
    gears = trace.gears()
    if not gears:
        return [u" " * width for _ in range(height)]
    span = max(1, len(gears))
    rows = []
    for row_index in range(height):
        gear = height - 1 - row_index
        cells = []
        for column in range(width):
            start = int(column * span / width)
            end = max(start + 1, int((column + 1) * span / width))
            chunk = gears[start:end]
            if gear >= 1 and chunk and max(chunk) == gear:
                cells.append(u"#")
            elif gear >= 1 and chunk and max(chunk) > gear:
                cells.append(u"|")
            elif gear == 0 and chunk and max(chunk) == 1:
                cells.append(u"-")
            else:
                cells.append(u" ")
        rows.append(u"".join(cells))
    return rows


def render_torque(trace, width=DEFAULT_GRID_WIDTH, height=6):
    peaks = [item.torque for item in trace.samples]
    if not peaks:
        return [u" " * width for _ in range(height)]
    top = max(peaks) or 1.0
    span = max(1, len(peaks))
    rows = []
    for row_index in range(height):
        threshold = top * (height - row_index) / height
        cells = []
        for column in range(width):
            start = int(column * span / width)
            end = max(start + 1, int((column + 1) * span / width))
            chunk = peaks[start:end]
            cells.append(u"#" if chunk and max(chunk) >= threshold else u" ")
        rows.append(u"".join(cells))
    return rows


def format_report(trace, results, header=""):
    lines = []
    if header:
        lines.append(header)
    lines.append(u"换挡策略与扭矩需求（按转速判据 + 保持时间）")
    box = trace.gearbox
    lines.append(u"挡位传动比：%s，主减速比 %.2f，效率 %.2f，车轮半径 %.2f m"
                 % (u" / ".join(u"%.2f" % (value,) for value in box.ratios),
                    box.final_drive, box.efficiency, box.wheel_radius))
    lines.append(u"升挡线 %.0f rpm，降挡线 %.0f rpm，最短保持 %.2f s"
                 % (box.upshift_rpm, box.downshift_rpm, trace.min_dwell()
                    if trace.min_dwell() != float("inf")
                    else DEFAULT_MIN_DWELL))
    lines.append(u"-" * 60)
    lines.append(u"帧数：%d，时长：%.2f s，最高车速 %.2f m/s，最高转速 %.0f rpm"
                 % (len(trace), trace.duration(), trace.top_speed(),
                    trace.peak_rpm()))
    lines.append(u"换挡 %d 次：" % (len(trace.shifts),))
    for item in trace.shifts:
        lines.append(u"  t=%.2f s  %d -> %d  （%.2f m/s，%.0f -> %.0f rpm）"
                     % (item.t, item.from_gear, item.to_gear, item.speed,
                        item.rpm_before, item.rpm_after))
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


class ShiftParams(object):
    """把 ROS 参数读成一个对象。"""

    def __init__(self, **kwargs):
        for key in PARAM_KEYS:
            value = kwargs.get(key, PARAM_DEFAULTS[key])
            if key.startswith("topic/"):
                setattr(self, self._attr(key), value)
            else:
                setattr(self, self._attr(key), _as_float(value, key))
        ratios = (self.gearbox_ratio_1, self.gearbox_ratio_2,
                  self.gearbox_ratio_3, self.gearbox_ratio_4)
        self.gearbox = Gearbox(
            ratios=ratios,
            final_drive=self.gearbox_final_drive,
            efficiency=self.gearbox_efficiency,
            wheel_radius=self.vehicle_wheel_radius,
            idle_rpm=self.policy_idle_rpm,
            redline_rpm=self.policy_redline_rpm,
            upshift_rpm=self.policy_upshift_rpm,
            downshift_rpm=self.policy_downshift_rpm,
            peak_torque=self.engine_peak_torque,
            peak_rpm=self.engine_peak_rpm,
        )
        self.vehicle = Vehicle(
            mass=self.vehicle_mass,
            frontal_area=self.vehicle_frontal_area,
            drag_coeff=self.vehicle_drag_coeff,
            rolling=self.vehicle_rolling,
        )
        self.min_dwell = _non_negative(self.policy_min_dwell, "min_dwell")
        self.sim_step = _positive(self.sim_step, "sim_step")
        self.rate_publish_hz = _positive(self.rate_publish_hz, "publish_hz")
        self.topic_gear = getattr(self, "topic_gear", u"/gear/current")
        self.topic_report = getattr(self, "topic_report", u"/gear/report")
        self.topic_curve = getattr(self, "topic_curve", u"/gear/curve")
        self.topic_shift = getattr(self, "topic_shift", u"/gear/shift")

    @staticmethod
    def _attr(key):
        return key.replace("/", "_")

    @classmethod
    def from_params(cls, get_param):
        values = {}
        for key in PARAM_KEYS:
            values[key] = get_param(key, PARAM_DEFAULTS[key])
        return cls(**values)

    def segments(self):
        return build_profile(accel=self.profile_accel,
                             max_speed=self.profile_max_speed,
                             hold=self.profile_hold_seconds,
                             decel=self.profile_decel)

    def as_dict(self):
        return dict((key, getattr(self, self._attr(key))) for key in PARAM_KEYS)

    def __repr__(self):
        return "ShiftParams(%d keys)" % (len(PARAM_KEYS),)


# =================================================================== 闭环行驶
# 上面 simulate() 是「车速已知、反推扭矩」的开环算法；下面 drive() 是闭环：
# 只给目标车速，油门与刹车由比例控制器给，扭矩受发动机外特性限制，
# 车速由「牵引力 − 阻力 − 制动力」积分出来。于是换挡会反过来影响加速度，
# 挡位挂错、扭矩不够，车速就跟不上目标。

DEFAULT_THROTTLE_GAIN = 2.00
DEFAULT_BRAKE_GAIN = 2.00
DEFAULT_MAX_CMD_ACCEL = 2.00
DEFAULT_TRACK_TOLERANCE = 2.50

DRIVE_ASSERT_NAMES = (u"挡位覆盖", u"换挡平顺", u"跟踪误差", u"转速区间")


class DriveSample(object):
    """闭环的一帧：目标车速、实际车速、挡位、转速、扭矩、累计里程。"""

    __slots__ = ("t", "target", "speed", "gear", "rpm", "torque", "distance")

    def __init__(self, t, target, speed, gear, rpm, torque, distance):
        self.t = t
        self.target = target
        self.speed = speed
        self.gear = gear
        self.rpm = rpm
        self.torque = torque
        self.distance = distance

    def error(self):
        return abs(self.target - self.speed)

    def as_row(self):
        return (self.t, self.target, self.speed, self.gear, self.rpm,
                self.torque, self.distance)

    def __repr__(self):
        return ("DriveSample(t=%.2f, target=%.2f, v=%.2f, gear=%d, rpm=%.0f)"
                % (self.t, self.target, self.speed, self.gear, self.rpm))


class DriveTrace(object):
    """闭环行驶的完整记录。"""

    __slots__ = ("samples", "shifts", "gearbox", "vehicle", "name")

    def __init__(self, samples, shifts, gearbox, vehicle, name=""):
        self.samples = list(samples)
        self.shifts = list(shifts)
        self.gearbox = gearbox
        self.vehicle = vehicle
        self.name = name

    def __len__(self):
        return len(self.samples)

    def count(self):
        return len(self.samples)

    def is_empty(self):
        return not self.samples

    def duration(self):
        if not self.samples:
            return 0.0
        return self.samples[-1].t

    def gears(self):
        return [item.gear for item in self.samples]

    def gears_used(self):
        return sorted(set(self.gears()))

    def top_speed(self):
        return max(item.speed for item in self.samples) if self.samples else 0.0

    def distance(self):
        return self.samples[-1].distance if self.samples else 0.0

    def peak_rpm(self):
        return max(item.rpm for item in self.samples) if self.samples else 0.0

    def peak_torque(self):
        return max(item.torque for item in self.samples) if self.samples else 0.0

    def errors(self):
        return [item.error() for item in self.samples]

    def tracking_error(self):
        values = self.errors()
        return max(values) if values else 0.0

    def mean_error(self):
        values = self.errors()
        return sum(values) / len(values) if values else 0.0

    def worst_index(self):
        values = self.errors()
        if not values:
            return 0
        best = 0
        for index, value in enumerate(values):
            if value > values[best]:
                best = index
        return best

    def min_dwell(self):
        if len(self.shifts) < 2:
            return float("inf")
        gaps = [second.t - first.t
                for first, second in zip(self.shifts, self.shifts[1:])]
        return min(gaps)

    def hunting(self, window=1.0):
        count = 0
        for first, second in zip(self.shifts, self.shifts[1:]):
            if second.t - first.t >= window:
                continue
            if second.to_gear == first.from_gear:
                count += 1
        return count

    def rpm_out_of_range(self):
        low = self.gearbox.idle_rpm
        high = self.gearbox.redline_rpm
        return [item for item in self.samples
                if item.rpm < low * 0.5 or item.rpm > high]

    def shift_table(self):
        return [(item.t, item.from_gear, item.to_gear, item.rpm_before,
                 item.rpm_after) for item in self.shifts]

    def as_rows(self):
        return [item.as_row() for item in self.samples]

    def __repr__(self):
        return "DriveTrace(count=%d, shifts=%d, dist=%.1f)" % (
            len(self.samples), len(self.shifts), self.distance())


def drive(gearbox=None, vehicle=None, segments=None, step=DEFAULT_SIM_STEP,
          min_dwell=DEFAULT_MIN_DWELL, throttle_gain=DEFAULT_THROTTLE_GAIN,
          brake_gain=DEFAULT_BRAKE_GAIN,
          max_cmd_accel=DEFAULT_MAX_CMD_ACCEL, name=""):
    """闭环行驶：目标车速 → 踏板 → 受限扭矩 → 车速积分。"""
    if gearbox is None:
        gearbox = Gearbox()
    if vehicle is None:
        vehicle = Vehicle()
    if segments is None:
        segments, _duration = build_profile()
    step = _positive(step, "step")
    min_dwell = _non_negative(min_dwell, "min_dwell")
    throttle_gain = _positive(throttle_gain, "throttle_gain")
    brake_gain = _positive(brake_gain, "brake_gain")
    max_cmd_accel = _positive(max_cmd_accel, "max_cmd_accel")
    duration = segments[-1][1]

    samples = []
    shifts = []
    gear = 1
    speed = 0.0
    distance = 0.0
    last_shift_t = -min_dwell
    index = 0
    while True:
        t = index * step
        if t > duration + 1e-9:
            break
        target = max(0.0, speed_at(segments, t))
        error = target - speed
        if error > 0.0:
            cmd_accel = min(throttle_gain * error, max_cmd_accel)
            brake = 0.0
        else:
            cmd_accel = 0.0
            brake = min(brake_gain * (-error), max_cmd_accel)

        ratio = gearbox.total_ratio(gear)
        rpm = max(gearbox.idle_rpm, gearbox.rpm_for(speed, gear))
        need_force = vehicle.force(speed, cmd_accel)
        need_torque = need_force * gearbox.wheel_radius / (
            ratio * gearbox.efficiency)
        torque = clamp(need_torque, 0.0, gearbox.max_torque(rpm))
        traction = torque * ratio * gearbox.efficiency / gearbox.wheel_radius
        net = traction - vehicle.resistance(speed) - vehicle.mass * brake
        speed = max(0.0, speed + net / vehicle.mass * step)
        distance += speed * step

        dwell_elapsed = t - last_shift_t - min_dwell
        next_gear = shift_decision(gearbox, speed, gear, dwell_elapsed)
        if next_gear != gear:
            rpm_before = gearbox.rpm_for(speed, gear)
            rpm_after = gearbox.rpm_for(speed, next_gear)
            shifts.append(Shift(t, gear, next_gear, speed, rpm_before,
                                rpm_after))
            gear = next_gear
            last_shift_t = t
        samples.append(DriveSample(t, target, speed, gear, rpm, torque,
                                   distance))
        index += 1
    return DriveTrace(samples, shifts, gearbox, vehicle, name)


def check_drive(trace, min_dwell=DEFAULT_MIN_DWELL,
                track_tolerance=DEFAULT_TRACK_TOLERANCE,
                expected_gears=None):
    """闭环的四条断言：挡位覆盖 / 换挡平顺 / 跟踪误差 / 转速区间。"""
    results = []

    used = trace.gears_used()
    if expected_gears is None:
        expected_gears = GEAR_COUNT
    results.append({
        "type": u"挡位覆盖",
        "passed": len(used) >= expected_gears,
        "detail": u"用到的挡位 %r（期望至少 %d 个）" % (used, expected_gears),
    })

    dwell = trace.min_dwell()
    window = max(2.0 * min_dwell, 1.0)
    hunting = trace.hunting(window)
    results.append({
        "type": u"换挡平顺",
        "passed": bool((dwell >= min_dwell - 1e-9) and hunting == 0),
        "detail": u"最短保持 %.2f s（要求 >= %.2f s），%.1f s 内来回换挡 %d 次"
                  % (dwell if dwell != float("inf") else 0.0, min_dwell,
                     window, hunting),
    })

    worst = trace.tracking_error()
    index = trace.worst_index()
    point = trace.samples[index] if trace.samples else None
    detail = u"最大偏差 %.2f m/s（容差 %.2f）" % (worst, track_tolerance)
    if point is not None:
        detail += u"，最差在 t=%.2f s（目标 %.2f，实到 %.2f）" % (
            point.t, point.target, point.speed)
    results.append({
        "type": u"跟踪误差",
        "passed": worst <= track_tolerance + 1e-9,
        "detail": detail,
    })

    bad = trace.rpm_out_of_range()
    results.append({
        "type": u"转速区间",
        "passed": not bad,
        "detail": u"越界 %d 帧（怠速 %.0f，红线 %.0f，实测最高 %.0f）"
                  % (len(bad), trace.gearbox.idle_rpm,
                     trace.gearbox.redline_rpm, trace.peak_rpm()),
    })
    return results


def render_tracking(trace, width=DEFAULT_GRID_WIDTH, height=8):
    """目标车速（`.`）与实际车速（`#`）的叠合字符图。"""
    if not trace.samples:
        return [u" " * width for _ in range(height)]
    top = max([max(item.target, item.speed) for item in trace.samples] + [1.0])
    span = max(1, len(trace.samples))
    rows = []
    for row_index in range(height):
        threshold = top * (height - row_index) / height
        cells = []
        for column in range(width):
            start = int(column * span / width)
            end = max(start + 1, int((column + 1) * span / width))
            chunk = trace.samples[start:end]
            actual = max(item.speed for item in chunk)
            wanted = max(item.target for item in chunk)
            if actual >= threshold:
                cells.append(u"#")
            elif wanted >= threshold:
                cells.append(u".")
            else:
                cells.append(u" ")
        rows.append(u"".join(cells))
    return rows


def format_drive_report(trace, results, header=""):
    lines = []
    if header:
        lines.append(header)
    lines.append(u"换挡策略与扭矩需求（闭环：踏板 → 受限扭矩 → 车速积分）")
    box = trace.gearbox
    lines.append(u"挡位传动比：%s，主减速比 %.2f，效率 %.2f，车轮半径 %.2f m"
                 % (u" / ".join(u"%.2f" % (value,) for value in box.ratios),
                    box.final_drive, box.efficiency, box.wheel_radius))
    lines.append(u"升挡线 %.0f rpm，降挡线 %.0f rpm，最短保持 %.2f s"
                 % (box.upshift_rpm, box.downshift_rpm,
                    trace.min_dwell() if trace.min_dwell() != float("inf")
                    else DEFAULT_MIN_DWELL))
    lines.append(u"-" * 60)
    lines.append(u"帧数：%d，时长：%.2f s，行驶 %.2f m，最高车速 %.2f m/s，"
                 u"最高转速 %.0f rpm"
                 % (len(trace), trace.duration(), trace.distance(),
                    trace.top_speed(), trace.peak_rpm()))
    lines.append(u"跟踪：最大偏差 %.2f m/s，平均偏差 %.2f m/s（最差第 %d 帧）"
                 % (trace.tracking_error(), trace.mean_error(),
                    trace.worst_index() + 1))
    lines.append(u"换挡 %d 次：" % (len(trace.shifts),))
    for item in trace.shifts:
        lines.append(u"  t=%.2f s  %d -> %d  （%.2f m/s，%.0f -> %.0f rpm）"
                     % (item.t, item.from_gear, item.to_gear, item.speed,
                        item.rpm_before, item.rpm_after))
    passed = sum(1 for item in results if item["passed"])
    lines.append(u"断言：%d 通过 / %d 失败（共 %d 条）"
                 % (passed, len(results) - passed, len(results)))
    for item in results:
        lines.append(u"  [%s] %s %s" % (u"PASS" if item["passed"] else u"FAIL",
                                        item["type"], item["detail"]))
    lines.append(u"结论：%s" % (u"PASSED" if all_passed(results) else u"FAILED",))
    return u"\n".join(lines)
