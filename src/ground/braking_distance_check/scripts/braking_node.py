#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""制动距离核算节点：算一次、打印报告、把报告与字符条以话题形式发出来。"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import sys
import traceback

import rospy
from std_msgs.msg import String

import braking_core as core


def _log(level, message):
    """Melodic（Python 2.7）上给 rospy 的日志消息必须是 byte str。"""
    text = message
    if not isinstance(text, bytes):
        text = text.encode("utf-8")
    getattr(rospy, level)(text)


class BrakingNode(object):
    """一次性算出停车距离分解，然后按 publish_hz 重发。"""

    def __init__(self):
        self.params = core.BrakingParams.from_params(rospy.get_param)
        road = self.params.road()
        self.result = core.measure(self.params.speed_kmh, self.params.reaction_time, road)
        self.results = core.check_braking(
            self.result,
            available_distance=self.params.available_distance,
            gap=self.params.gap,
            speed_limit_kmh=self.params.speed_limit_kmh,
        )
        self.rows = core.speed_table(road, self.params.reaction_time)
        header = "刹车距离核算（内置物理模型，不依赖 CARLA）"
        self.report = core.format_report(self.result, self.results, self.rows, header)
        self.bars = "\n".join(core.render_bars(self.result, self.params.available_distance))
        self.report_publisher = rospy.Publisher(self.params.topic_report, String,
                                                queue_size=1, latch=True)
        self.bars_publisher = rospy.Publisher(self.params.topic_bars, String,
                                              queue_size=1, latch=True)

    def publish(self):
        self.report_publisher.publish(String(data=core.to_message_text(self.report)))
        self.bars_publisher.publish(String(data=core.to_message_text(self.bars)))

    def run(self):
        rate = rospy.Rate(self.params.publish_hz)
        try:
            _log("loginfo", self.report)
            _log("loginfo", self.bars)
            while not rospy.is_shutdown():
                self.publish()
                rate.sleep()
        except Exception:
            _log("logerr", "制动距离核算异常：\n" + traceback.format_exc())
            raise


def main():
    rospy.init_node("braking_distance_check")
    try:
        node = BrakingNode()
    except core.BrakingError as error:
        _log("logerr", "参数错误：%s" % error)
        return 1
    rospy.on_shutdown(node.publish)
    node.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
