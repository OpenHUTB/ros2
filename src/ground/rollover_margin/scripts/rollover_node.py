#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""侧翻裕度节点的 ROS 封装（ROS 1 / Melodic，Python 2.7 & 3.x）。"""
from __future__ import division, print_function, unicode_literals

import os
import sys
import traceback

import rospy
from std_msgs.msg import Float64, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rollover_core as core  # noqa: E402


def _log(level, message):
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


class RolloverNode(object):
    def __init__(self):
        self.params = core.RolloverParams.from_params(rospy.get_param)
        self.cornering = self.params.cornering()
        self.results = core.check_cornering(
            self.cornering, self.params.rollover_margin,
            self.params.transfer_tolerance)
        self.report = core.format_report(
            self.cornering, self.results,
            header=u"横向载荷转移与侧翻裕度（准静态，不依赖 CARLA）")

        self.pub_report = rospy.Publisher(
            self.params.topic_report, String, queue_size=1, latch=True)
        self.pub_loads = rospy.Publisher(
            self.params.topic_loads, String, queue_size=1, latch=True)
        self.pub_speed = rospy.Publisher(
            self.params.topic_critical_speed, Float64, queue_size=1, latch=True)
        rospy.on_shutdown(self.on_shutdown)

    def loads_text(self):
        rows = [u"speed_kmh,lateral_accel,inner_load,outer_load,lift_off"]
        for item in core.speed_table(self.cornering.spec, self.cornering.radius):
            rows.append(u"%.1f,%.4f,%.1f,%.1f,%d" % (
                item.speed_kmh, item.lateral_accel(),
                item.inner_load(), item.outer_load(), 1 if item.lifts_off() else 0))
        return u"\n".join(rows)

    def publish_all(self):
        self.pub_report.publish(String(data=core.to_message_text(self.report)))
        self.pub_loads.publish(String(data=core.to_message_text(self.loads_text())))
        self.pub_speed.publish(Float64(data=float(self.cornering.speed_limit_kmh())))

    def on_shutdown(self):
        try:
            _log("info", u"已发布零控制量")
        except Exception:  # noqa: BLE001
            pass

    def run(self):
        try:
            spec = self.cornering.spec
            _log("info", u"侧翻裕度节点已启动：轮距 %.2f m，质心高 %.2f m，质量 %.0f kg，"
                         u"半径 %.1f m，车速 %.1f km/h" % (
                             spec.track, spec.cg_height, spec.mass,
                             self.cornering.radius, self.cornering.speed_kmh))
            _log("info", u"SSF=%.4f，临界加速度 %.3f m/s²，极限车速 %.1f km/h" % (
                spec.ssf(), spec.critical_accel(), self.cornering.speed_limit_kmh()))
            _log("info", u"发布 %s、%s、%s" % (
                self.params.topic_report, self.params.topic_loads,
                self.params.topic_critical_speed))
            self.publish_all()
            print(self.report)
            passed = sum(1 for item in self.results if item["passed"])
            _log("info", u"结论：%s（%d/%d 条断言通过）" % (
                u"PASSED" if passed == len(self.results) else u"FAILED",
                passed, len(self.results)))
            rate = rospy.Rate(max(1.0, self.params.publish_hz))
            while not rospy.is_shutdown():
                rate.sleep()
            return 0
        except Exception:  # noqa: BLE001
            _log("err", u"侧翻裕度核算异常：%s" % traceback.format_exc())
            return 1


def main():
    rospy.init_node("rollover_margin", anonymous=False)
    node = RolloverNode()
    code = node.run()
    try:
        node.publish_all()
    except Exception:  # noqa: BLE001
        pass
    return code


if __name__ == "__main__":
    sys.exit(main())
