#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""lane_change_feasibility 的 ROS 节点：算一次换道可行性并把报告发出去。

Melodic（Python 2.7）上发给 rospy 的日志消息必须是 byte str，所有日志都过 _log()。
"""

from __future__ import division, print_function, unicode_literals

import os
import sys
import traceback

import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool, Float64, String

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import lane_change_core as core


def _log(level, message):
    text = message
    if not isinstance(text, str):
        text = u"%s" % (text,)
    try:
        text = text.encode("utf-8")
    except AttributeError:
        pass
    getattr(rospy, "log" + level)(text)


def _as_bool(value, default=False):
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    return u"%s" % (value,).strip().lower() in (u"1", u"true", u"yes", u"on")


def _as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


class LaneChangeNode(object):

    def __init__(self):
        self.params = core.LaneChangeParams.from_params(rospy.get_param)
        self.report_topic = rospy.get_param("topic/report",
                                           "/lane_change/report")
        self.lane_topic = rospy.get_param("topic/lane",
                                          "/lane_change/lane")
        self.decision_topic = rospy.get_param("topic/decision",
                                              "/lane_change/decision")
        self.duration_topic = rospy.get_param("topic/duration",
                                             "/lane_change/duration")
        self.report_pub = rospy.Publisher(self.report_topic, String,
                                          queue_size=1, latch=True)
        self.lane_pub = rospy.Publisher(self.lane_topic, String,
                                        queue_size=1, latch=True)
        self.decision_pub = rospy.Publisher(self.decision_topic, Bool,
                                            queue_size=1, latch=True)
        self.duration_pub = rospy.Publisher(self.duration_topic, Float64,
                                            queue_size=1, latch=True)
        self.tf_enabled = _as_bool(rospy.get_param("tf/enabled", True))
        self.tf_rate_hz = _as_float(rospy.get_param("tf/rate_hz", 10.0),
                                     10.0)
        self.tf_frames = []
        self.tf_index = 0
        self.tf_broadcaster = None
        if self.tf_enabled:
            try:
                self.tf_broadcaster = tf2_ros.TransformBroadcaster()
                _log("info", u"TF 广播已启用（%.1f Hz）" % (self.tf_rate_hz,))
            except Exception:
                _log("warn", u"TF 广播初始化失败，已跳过")
        rospy.on_shutdown(self.on_shutdown)

    def publish_frames(self):
        """按 tf/rate_hz 把预先算好的关键帧循环广播出去。"""
        if self.tf_broadcaster is None or not self.tf_frames:
            return
        frame = self.tf_frames[self.tf_index % len(self.tf_frames)]
        self.tf_index += 1
        parent, child, x, y, yaw = frame
        message = TransformStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x = x
        message.transform.translation.y = y
        message.transform.translation.z = 0.0
        message.transform.rotation.x = 0.0
        message.transform.rotation.y = 0.0
        message.transform.rotation.z = math.sin(yaw / 2.0)
        message.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(message)

    def on_shutdown(self):
        try:
            _log("info", u"节点退出")
        except Exception:
            pass

    def run(self):
        params = self.params
        spec = params.spec

        _log("info", u"换道可行性节点已启动：车道 %.2f m，横向加速度上限 "
                     u"%.2f m/s^2，TTC 下限 %.2f s"
             % (spec.lane_width, spec.max_lateral_accel, spec.ttc_min))
        _log("info", u"自车 %.1f km/h，目标车道前车 %.1f km/h 距 %.1f m，"
                     u"后车 %.1f km/h 距 %.1f m"
             % (params.traffic.ego_kmh, params.traffic.front_kmh,
                params.traffic.front_gap, params.traffic.rear_kmh,
                params.traffic.rear_gap))

        try:
            plan = core.make_plan(spec, params.traffic, params.direction)
            results = core.check_plan(plan)
            report = core.format_report(spec, params.traffic, plan, results)
            lane = u"\n".join(core.render_lane(plan))
        except core.LaneChangeError as error:
            _log("err", u"参数不合法：%s" % (error,))
            return 1
        except Exception:
            _log("err", u"换道核算异常：%s" % (traceback.format_exc(),))
            return 1

        self.report_pub.publish(String(data=core.to_message_text(report)))
        self.lane_pub.publish(String(data=core.to_message_text(lane)))
        self.decision_pub.publish(Bool(data=plan.feasible))
        self.duration_pub.publish(Float64(data=plan.duration))

        _log("info", report)
        _log("info", lane)
        _log("info", u"已发布：%s / %s / %s / %s"
             % (self.report_topic, self.lane_topic, self.decision_topic,
                self.duration_topic))

        if self.tf_broadcaster is not None:
            self.tf_frames = core.ego_frames(plan)
            _log("info", u"已开始广播 TF，%d 个关键帧"
                 % (len(self.tf_frames),))

        rate = rospy.Rate(max(1.0, params.rate_publish_hz))
        throttle = rospy.Rate(max(1.0, self.tf_rate_hz))
        while not rospy.is_shutdown():
            self.publish_frames()
            throttle.sleep()
            rate.sleep()
        return 0


def main():
    rospy.init_node("lane_change_feasibility")
    node = LaneChangeNode()
    return node.run()


if __name__ == "__main__":
    sys.exit(main())
