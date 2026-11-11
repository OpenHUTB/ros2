#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""轮胎滑移率-附着曲线节点的 ROS 封装（ROS 1 / Melodic 可跑，Python 2.7 & 3.x）。

节点只做三件事：
  1. 从参数服务器读配置，算出一条附着曲线并做四条断言；
  2. 把报告、曲线表、峰值滑移率分别发布到三个 latched 话题；
  3. 打印完保持挂起，方便另一个终端用 rosparam get 核对参数。
"""
from __future__ import division, print_function, unicode_literals

import os
import sys
import traceback

import rospy
from std_msgs.msg import Float64, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tire_core as core  # noqa: E402


def _log(level, message):
    """Melodic 的 rosgraph 对非 ASCII unicode 调 str() 会抛 UnicodeEncodeError，
    所以日志消息必须是 byte str。"""
    func = getattr(rospy, "log" + level)
    try:
        unicode  # noqa: F821
    except NameError:
        func(message)
        return
    if isinstance(message, unicode):  # noqa: F821
        func(message.encode("utf-8"))
    else:
        func(message)


class TireNode(object):
    def __init__(self):
        self.params = core.TireParams.from_params(rospy.get_param)
        spec = self.params.spec()
        self.curve = core.build_curve(spec, self.params.samples)
        self.results = core.check_curve(
            self.curve, spec,
            lock_friction_ratio=self.params.lock_friction_ratio,
            linear_tolerance=self.params.linear_tolerance,
            peak_tolerance=self.params.peak_tolerance,
        )
        self.report = core.format_report(
            self.curve, self.results,
            header=u"轮胎滑移率-附着曲线（纯解析，不依赖 CARLA）")

        self.pub_report = rospy.Publisher(
            self.params.topic_report, String, queue_size=1, latch=True)
        self.pub_curve = rospy.Publisher(
            self.params.topic_curve, String, queue_size=1, latch=True)
        self.pub_peak = rospy.Publisher(
            self.params.topic_peak_slip, Float64, queue_size=1, latch=True)
        rospy.on_shutdown(self.on_shutdown)

    # ── 发布 ──────────────────────────────────────────────────────────

    def curve_text(self):
        rows = [u"slip,friction,ratio"]
        for slip, mu, ratio in self.curve.as_rows():
            rows.append(u"%.4f,%.4f,%.4f" % (slip, mu, ratio))
        return u"\n".join(rows)

    def publish_all(self):
        self.pub_report.publish(String(data=core.to_message_text(self.report)))
        self.pub_curve.publish(String(data=core.to_message_text(self.curve_text())))
        self.pub_peak.publish(Float64(data=float(self.params.peak_slip)))

    def on_shutdown(self):
        try:
            _log("info", u"已发布零控制量")
        except Exception:  # noqa: BLE001
            pass

    # ── 主循环 ────────────────────────────────────────────────────────

    def run(self):
        try:
            keys = core.key_points(self.params.spec())
            _log("info", u"轮胎附着曲线节点已启动：路面=%s，峰值滑移率=%.3f，峰值附着=%.3f，"
                         u"采样=%d 点，ABS 目标带=[%.4f, %.4f]" % (
                             keys["surface"], keys["peak_slip"], keys["peak_friction"],
                             self.curve.count, keys["abs_low"], keys["abs_high"]))
            _log("info", u"发布 %s、%s、%s" % (
                self.params.topic_report, self.params.topic_curve,
                self.params.topic_peak_slip))
            self.publish_all()
            print(self.report)
            passed = sum(1 for item in self.results if item["passed"])
            _log("info", u"结论：%s（%d/%d 条断言通过）" % (
                u"PASSED" if passed == len(self.results) else u"FAILED",
                passed, len(self.results)))

            # 保持挂起，方便另一个终端 rosparam get
            rate = rospy.Rate(max(1.0, self.params.publish_hz))
            while not rospy.is_shutdown():
                rate.sleep()
            return 0
        except Exception:  # noqa: BLE001
            _log("err", u"曲线核算异常：%s" % traceback.format_exc())
            return 1


def main():
    rospy.init_node("tire_slip_curve", anonymous=False)
    node = TireNode()
    code = node.run()
    try:
        node.publish_all()
    except Exception:  # noqa: BLE001
        pass
    return code


if __name__ == "__main__":
    sys.exit(main())
