#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""trailer_kinematics 的 ROS 节点：算一次，把报告与相图发出去。

注意：Melodic（Python 2.7）上发给 rospy 的日志消息**必须是 byte str**，
否则 rosgraph 的格式化会在 str(unicode) 上抛 UnicodeEncodeError。所以所有
日志都经过 _log()，先在 py2 下把 unicode 编码成 utf-8 再交给 rospy。
"""

from __future__ import division, print_function, unicode_literals

import io
import os
import sys
import traceback

import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64, String

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import trailer_core as core


def _log(level, message):
    """把消息转成字节串再交给 rospy，兼容 Python 2.7 与 3.x。"""
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


class TrailerNode(object):

    def __init__(self):
        self.params = core.TrailerParams.from_params(rospy.get_param)
        self.report_topic = rospy.get_param("topic/report", "/trailer/report")
        self.phase_topic = rospy.get_param("topic/phase", "/trailer/phase")
        self.articulation_topic = rospy.get_param("topic/articulation",
                                                 "/trailer/articulation")
        self.report_pub = rospy.Publisher(self.report_topic, String,
                                          queue_size=1, latch=True)
        self.phase_pub = rospy.Publisher(self.phase_topic, String,
                                         queue_size=1, latch=True)
        self.articulation_pub = rospy.Publisher(self.articulation_topic,
                                                Float64, queue_size=1,
                                                latch=True)
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
            _log("info", u"节点退出，最后一次铰接角已发布")
        except Exception:
            pass

    def run(self):
        params = self.params
        spec = params.spec

        _log("info", u"牵引车-挂车运动学节点已启动：轴距 %.2f m，挂车 %.2f m，"
                     u"铰接偏移 %.2f m"
             % (spec.wheelbase, spec.trailer_length, spec.hitch_offset))

        try:
            trajectory = core.simulate(
                spec=spec,
                speed=params.sim_speed,
                steer_deg=params.sim_steer_deg,
                seconds=params.sim_seconds,
                step=params.sim_step,
                gear=params.sim_gear,
                name="sim",
            )
            results = core.check_trailer(
                trajectory,
                jackknife_deg=params.check_jackknife_deg,
                channel_width=params.check_channel_width,
                step_tolerance=params.check_step_tolerance,
                gear=params.sim_gear,
                speed=params.sim_speed,
                steer_deg=params.sim_steer_deg,
                seconds=params.sim_seconds,
                step=params.sim_step,
            )
            report = core.format_report(spec, trajectory, results)
        except core.TrailerError as error:
            _log("err", u"参数不合法：%s" % (error,))
            return 1
        except Exception:
            _log("err", u"仿真异常：%s"
                 % (traceback.format_exc().decode("utf-8", "replace")
                    if not isinstance(traceback.format_exc(), str)
                    else traceback.format_exc(),))
            return 1

        phase = u"\n".join(core.render_phase(trajectory,
                                             jackknife_deg=params.check_jackknife_deg))
        body = u"\n".join(core.render_body(trajectory))

        self.report_pub.publish(String(data=core.to_message_text(report)))
        self.phase_pub.publish(String(data=core.to_message_text(phase)))
        self.articulation_pub.publish(
            Float64(data=trajectory.articulation_max_deg()))

        _log("info", report)
        _log("info", u"铰接角相图（纵轴 ±折刀角，横轴时间）：")
        _log("info", phase)
        _log("info", u"车身俯视（# 车身角点，o 起点）：")
        _log("info", body)
        _log("info", u"已发布：%s / %s / %s"
             % (self.report_topic, self.phase_topic, self.articulation_topic))

        if self.tf_broadcaster is not None:
            self.tf_frames = core.frames_for(trajectory)
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
    rospy.init_node("trailer_kinematics")
    node = TrailerNode()
    return node.run()


if __name__ == "__main__":
    sys.exit(main())
