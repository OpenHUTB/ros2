#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""速度曲线规划节点：算完打到终端，并把报告与曲线挂到 latched 话题上。"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import os
import sys
import traceback

import rospy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import speed_core as core


def _log(level, message):
    """Melodic（Py2.7）上 rospy 的日志消息必须是 byte str。"""
    text = message
    if not isinstance(text, bytes):
        text = text.encode("utf-8")
    getattr(rospy, level)(text)


class SpeedProfileNode(object):
    def __init__(self):
        self.params = core.SpeedParams.from_params(rospy.get_param)
        self.profile = self.params.profile()
        self.results = core.check_profile(
            self.profile, self.params.speed_limit_kmh,
            self.params.time_budget, self.params.tolerance)
        self.curve = u"\n".join(core.render_profile(self.profile))
        self.plans = core.compare_plans(
            self.params.distance, self.params.cruise_kmh,
            self.params.accel, self.params.decel,
            self.params.speed_limit_kmh, self.params.start_kmh)
        self.report = core.format_report(
            self.profile, self.results,
            header=u"速度曲线规划（内置曲线计算，不依赖 CARLA）",
            plans=self.plans,
            speed_limit_kmh=self.params.speed_limit_kmh)
        self.pub_profile = rospy.Publisher(
            self.params.topic_profile, String, queue_size=10, latch=True)
        self.pub_report = rospy.Publisher(
            self.params.topic_report, String, queue_size=10, latch=True)

    def publish(self):
        self.pub_profile.publish(String(data=core.to_message_text(self.curve)))
        self.pub_report.publish(String(data=core.to_message_text(self.report)))

    def run(self):
        try:
            _log("loginfo", u"速度曲线规划节点已启动：峰值 %.2f km/h，距离 %.2f m，"
                             u"加速 %.2f m/s^2，减速 %.2f m/s^2" % (
                                 self.profile.peak_speed_kmh, self.profile.distance,
                                 self.profile.accel, self.profile.decel))
            _log("loginfo", u"发布 %s 与 %s" % (
                self.params.topic_profile, self.params.topic_report))
            self.publish()
            print(self.report)
            print(self.curve)

            rate = rospy.Rate(self.params.publish_hz)
            self.pub_profile.publish(String(data=core.to_message_text(self.curve)))
            self.pub_report.publish(String(data=core.to_message_text(self.report)))
            while not rospy.is_shutdown():
                rate.sleep()
        except Exception:
            _log("logerr", u"曲线规划异常：%s" % traceback.format_exc())
            raise

    def publish_zero(self):
        try:
            self.pub_report.publish(String(data=core.to_message_text(self.report)))
        except Exception:
            pass

    def shutdown(self):
        _log("loginfo", u"节点退出")


def main():
    rospy.init_node("speed_profile_check", anonymous=False)
    node = None
    try:
        node = SpeedProfileNode()
        rospy.on_shutdown(node.publish_zero)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except core.ProfileError as error:
        _log("logerr", u"参数错误：%s" % error)
        return 1
    finally:
        if node is not None and rospy.is_shutdown():
            node.publish_zero()
    return 0


if __name__ == "__main__":
    sys.exit(main())
