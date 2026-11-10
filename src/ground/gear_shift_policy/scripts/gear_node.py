#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""换挡策略与扭矩需求的 ROS 节点（纯计算，不依赖 CARLA / 可视化）。"""

from __future__ import division, print_function, unicode_literals

import sys
import traceback

import rospy
from std_msgs.msg import Int32, String

sys.path.insert(0, __file__.rsplit("/", 1)[0] if "/" in __file__
                else __file__.rsplit("\\", 1)[0])

import gear_core as core


def _log(level, message):
    """Melodic（Python 2.7）上日志消息必须是 byte str，否则 rosgraph 会报错。"""
    text = message
    if not isinstance(text, bytes):
        try:
            text = text.encode("utf-8")
        except (AttributeError, UnicodeDecodeError):
            text = str(text)
    getattr(rospy, "log" + level)(text)


def _as_bool(value, default=False):
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    text = u"%s" % (value,)
    return text.strip().lower() in ("1", "true", "yes", "on")


def _as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


class GearRunner(object):

    def __init__(self):
        self.params = core.ShiftParams.from_params(rospy.get_param)
        self.closed_loop = _as_bool(rospy.get_param("sim/closed_loop", True))
        self.throttle_gain = _as_float(
            rospy.get_param("driver/throttle_gain", core.DEFAULT_THROTTLE_GAIN),
            core.DEFAULT_THROTTLE_GAIN)
        self.brake_gain = _as_float(
            rospy.get_param("driver/brake_gain", core.DEFAULT_BRAKE_GAIN),
            core.DEFAULT_BRAKE_GAIN)
        self.max_cmd_accel = _as_float(
            rospy.get_param("driver/max_cmd_accel", core.DEFAULT_MAX_CMD_ACCEL),
            core.DEFAULT_MAX_CMD_ACCEL)
        self.track_tolerance = _as_float(
            rospy.get_param("check/track_tolerance",
                            core.DEFAULT_TRACK_TOLERANCE),
            core.DEFAULT_TRACK_TOLERANCE)
        self.trace = None
        self.drive_trace = None
        self.results = []
        self.report = u""
        self.curve = []
        self.pub_gear = rospy.Publisher(self.params.topic_gear, Int32,
                                        queue_size=1, latch=True)
        self.pub_report = rospy.Publisher(self.params.topic_report, String,
                                          queue_size=1, latch=True)
        self.pub_curve = rospy.Publisher(self.params.topic_curve, String,
                                         queue_size=1, latch=True)
        self.pub_shift = rospy.Publisher(self.params.topic_shift, String,
                                        queue_size=1, latch=True)
        self.loop_hz = max(1.0, self.params.rate_publish_hz)

    # ------------------------------------------------------------- 计算
    def compute(self):
        segments, duration = self.params.segments()
        _log("info", u"换挡策略节点已启动：升挡 %.0f rpm，降挡 %.0f rpm，"
                     u"最短保持 %.2f s，剖面 %.1f s，%s"
              % (self.params.gearbox.upshift_rpm,
                 self.params.gearbox.downshift_rpm,
                 self.params.min_dwell, duration,
                 u"闭环（踏板 → 受限扭矩 → 车速积分）"
                 if self.closed_loop else u"开环（车速已知反推扭矩）"))
        if self.closed_loop:
            self.drive_trace = core.drive(
                gearbox=self.params.gearbox, vehicle=self.params.vehicle,
                segments=segments, step=self.params.sim_step,
                min_dwell=self.params.min_dwell,
                throttle_gain=self.throttle_gain,
                brake_gain=self.brake_gain,
                max_cmd_accel=self.max_cmd_accel,
                name=u"drive")
            self.results = core.check_drive(
                self.drive_trace, min_dwell=self.params.min_dwell,
                track_tolerance=self.track_tolerance)
            self.report = core.format_drive_report(
                self.drive_trace, self.results,
                header=u"换挡策略与扭矩需求（闭环执行：踏板 → 受限扭矩 → "
                       u"车速积分，不依赖 CARLA）")
            self.curve = (core.render_gears(self.drive_trace)
                          + core.render_tracking(self.drive_trace))
            return
        self.trace = core.simulate(self.params.gearbox, self.params.vehicle,
                                   segments, step=self.params.sim_step,
                                   min_dwell=self.params.min_dwell,
                                   name=u"run")
        self.results = core.check_shift(self.trace,
                                       min_dwell=self.params.min_dwell)
        self.report = core.format_report(
            self.trace, self.results,
            header=u"换挡策略与扭矩需求（按转速判据 + 保持时间，不依赖 CARLA）")
        self.curve = (core.render_gears(self.trace)
                      + core.render_torque(self.trace))

    # ------------------------------------------------------------- 发布
    def publish(self):
        if self.closed_loop:
            trace = self.drive_trace
        else:
            trace = self.trace
        if trace is None:
            return
        self.pub_gear.publish(Int32(data=int(trace.samples[-1].gear)))
        self.pub_report.publish(String(data=core.to_message_text(self.report)))
        self.pub_curve.publish(String(data=core.to_message_text(
            u"\n".join(self.curve))))
        rows = [u"t(s)\tfrom\tto\tv(m/s)\trpm_before\trpm_after"]
        for item in trace.shifts:
            rows.append(u"%.2f\t%d\t%d\t%.2f\t%.0f\t%.0f" % item.as_row())
        if len(rows) == 1:
            rows.append(u"（本次剖面没有换挡）")
        self.pub_shift.publish(String(data=core.to_message_text(
            u"\n".join(rows))))

    def run(self):
        try:
            self.compute()
        except Exception:
            _log("err", u"场景执行异常：\n%s" % (traceback.format_exc(),))
            return
        print(self.report)
        sys.stdout.flush()
        self.publish()
        period = 1.0 / self.loop_hz
        while not rospy.is_shutdown():
            rospy.sleep(period)

    def shutdown(self):
        try:
            if self.trace is not None:
                self.publish()
        except Exception:
            pass


def main():
    rospy.init_node("gear_shift_policy", anonymous=False)
    runner = GearRunner()
    rospy.on_shutdown(runner.shutdown)
    runner.run()
    if not rospy.is_shutdown():
        runner.shutdown()


if __name__ == "__main__":
    main()
