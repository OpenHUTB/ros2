#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""倒车雷达分级告警的 ROS 节点。

内置一段脚本化读数（模拟倒车接近再退出），按 read_hz 逐帧喂给判定器，
把当前级别、级别条带、雷达条、报告分别发到四个 latched 话题上。
"""

from __future__ import division, print_function, unicode_literals

import traceback

import rospy
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger, TriggerResponse

import alert_core as core


def _log(level, message):
    """Melodic（Python 2.7）下 rospy 的日志必须拿 byte str。"""
    text = message
    if not isinstance(text, bytes):
        text = text.encode("utf-8")
    getattr(rospy, "log" + level)(text)


class RearAlert(object):
    def __init__(self):
        self.params = core.AlertParams.from_params(rospy.get_param)
        self.detector = self.params.detector
        self.readings = core.demo_readings()
        self.index = 0
        self.trace = None
        self.results = None
        self.report = ""

        self.pub_level = rospy.Publisher(self.params.topic_level, Int32,
                                         queue_size=1, latch=True)
        self.pub_report = rospy.Publisher(self.params.topic_report, String,
                                          queue_size=1, latch=True)
        self.pub_levels = rospy.Publisher(self.params.topic_levels, String,
                                          queue_size=1, latch=True)
        self.pub_radar = rospy.Publisher(self.params.topic_radar, String,
                                         queue_size=1, latch=True)
        self.rate = rospy.Rate(self.params.rate_publish_hz)
        self.srv_simulate = rospy.Service(self.params.service_simulate,
                                         Trigger, self.handle_simulate)
        rospy.on_shutdown(self.on_shutdown)

    # ------------------------------------------------------------------ 服务
    def handle_simulate(self, request):
        """被 rosservice 调用时重算一遍，返回一行 ASCII 摘要。

        `std_srvs/Trigger` 只有 success 与 message 两个字段。message 若放全中文
        报告，`rosservice call` 会把它转义成 `\\u5012\\u8F66\\u2026`，截图里
        没法读；所以这里只回一行 ASCII，完整报告仍在日志与话题里。
        """
        try:
            self.finish()
        except Exception:
            detail = traceback.format_exc()
            try:
                detail = detail.encode("ascii", "replace").decode("ascii")
            except AttributeError:
                pass
            return TriggerResponse(
                success=False,
                message=u"recompute failed: %s"
                        % (u" ".join(detail.split())[:200],))
        return TriggerResponse(
            success=core.all_passed(self.results),
            message=core.summary_text(self.trace, self.results))

    def publish_level(self, grade):
        self.pub_level.publish(Int32(data=int(grade.level)))

    def publish_views(self, grade, distances, trace=None):
        if trace is None:
            trace = core.Trace([grade])
        self.pub_levels.publish(String(data=core.to_message_text(
            u"\n".join(core.render_levels(trace)))))
        self.pub_radar.publish(String(data=core.to_message_text(
            u"\n".join(core.render_radar(distances)))))

    # ------------------------------------------------------------------ 主循环
    def run(self):
        _log("info", u"倒车雷达分级告警节点已启动：%d 帧读数，%.2f Hz，"
                     u"进入阈值 %s m，确认 %d 帧 / 释放 %d 帧"
             % (len(self.readings), self.params.rate_read_hz,
                u" / ".join(u"%.2f" % (value,)
                            for value in self.detector.enter),
                self.detector.confirm_frames, self.detector.release_frames))
        start = rospy.get_time()
        grades = []
        while not rospy.is_shutdown():
            if self.index >= len(self.readings):
                break
            reading = self.readings[self.index]
            wait = start + reading.t - rospy.get_time()
            if wait > 0.0:
                rospy.sleep(wait)
            grade = self.detector.feed(reading.t, reading.distances)
            grades.append(grade)
            if grade.level > core.LEVEL_NONE and grade.sensor >= 0:
                _log("info", u"t=%.2f s 级别 %d（%s），最近通道 %s，%.2f m"
                     % (grade.t, grade.level, grade.name(),
                        core.DEFAULT_SENSOR_NAMES[grade.sensor],
                        grade.distance if grade.distance is not None else -1.0))
            self.publish_level(grade)
            self.publish_views(grade, reading.distances,
                               core.Trace(grades[:]))
            self.index += 1
            self.rate.sleep()
        self.finish()
        # 报告发完后保持挂起，方便在另一个终端用 rosparam get 核对参数；Ctrl+C 退出
        while not rospy.is_shutdown():
            self.rate.sleep()

    def finish(self):
        detector = core.Detector(
            enter=self.detector.enter,
            exit_margin=self.detector.exit_margin,
            confirm_frames=self.detector.confirm_frames,
            release_frames=self.detector.release_frames,
            min_range=self.detector.min_range,
            max_range=self.detector.max_range)
        self.trace = core.run_detector(self.readings, detector,
                                       u"内置演示读数")
        self.results = core.check_alert(self.trace, self.readings,
                                        self.detector,
                                        self.params.max_switches)
        self.report = core.format_report(self.trace, self.readings,
                                         self.detector, self.results)
        print(self.report)
        self.pub_report.publish(String(data=core.to_message_text(self.report)))
        self.publish_views(self.trace.final(), self.readings[-1].distances,
                           self.trace)
        self.publish_level(self.trace.final())
        passed = sum(1 for item in self.results if item["passed"])
        _log("info", u"结论：%s（%d 条断言通过）"
             % (u"PASSED" if core.all_passed(self.results) else u"FAILED",
                passed))

    def on_shutdown(self):
        try:
            self.pub_level.publish(Int32(data=int(core.LEVEL_NONE)))
            if self.report:
                self.pub_report.publish(String(
                    data=core.to_message_text(self.report)))
        except Exception:
            pass


def main():
    rospy.init_node("ultrasonic_rear_alert", anonymous=False)
    node = None
    try:
        node = RearAlert()
        node.run()
    except Exception:
        _log("error", u"倒车雷达节点异常：%s"
             % (traceback.format_exc().decode("utf-8", "replace"),))
        return 1
    finally:
        if node is not None:
            node.on_shutdown()
    if not rospy.is_shutdown():
        rospy.signal_shutdown("scenario finished")
    return 0
