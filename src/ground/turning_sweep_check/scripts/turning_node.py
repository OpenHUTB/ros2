#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""转弯扫掠校验节点：算一次、打印报告、把报告和字符图以话题形式发出来。"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import sys
import traceback

import rospy
from std_msgs.msg import String

import turning_core as core


def _log(level, message):
    """Melodic（Python 2.7）上给 rospy 的日志消息必须是 byte str。"""
    text = message
    if not isinstance(text, bytes):
        text = text.encode("utf-8")
    getattr(rospy, level)(text)


class TurningSweepNode(object):
    """一次性算出扫掠包络，然后按 publish_hz 重发。"""

    def __init__(self):
        self.params = core.TurningParams.from_params(rospy.get_param)
        self.vehicle = self.params.vehicle()
        self.result = core.sweep(self.vehicle, self.params.turn_radius)
        self.results = core.check_sweep(
            self.vehicle,
            self.result,
            channel_width=self.params.channel_width,
            inner_clearance=self.params.inner_clearance,
            max_inner_diff=self.params.max_inner_diff,
        )
        header = "转弯扫掠校验（内置几何计算，不依赖 CARLA）"
        self.report = core.format_report(self.vehicle, self.result, self.results, header)
        self.ring = "\n".join(core.render_ring(self.vehicle, self.result))
        self.report_publisher = rospy.Publisher(self.params.topic_report, String,
                                                queue_size=1, latch=True)
        self.ring_publisher = rospy.Publisher(self.params.topic_ring, String,
                                              queue_size=1, latch=True)

    def publish(self):
        self.report_publisher.publish(String(data=core.to_message_text(self.report)))
        self.ring_publisher.publish(String(data=core.to_message_text(self.ring)))

    def run(self):
        rate = rospy.Rate(self.params.publish_hz)
        try:
            _log("loginfo", self.report)
            _log("loginfo", self.ring)
            while not rospy.is_shutdown():
                self.publish()
                rate.sleep()
        except Exception:
            _log("logerr", "转弯扫掠校验异常：\n" + traceback.format_exc())
            raise


def main():
    rospy.init_node("turning_sweep_check")
    try:
        node = TurningSweepNode()
    except core.TurningError as error:
        _log("logerr", "参数错误：%s" % error)
        return 1
    rospy.on_shutdown(node.publish)
    node.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
