#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ride_comfort_transmissibility 的 ROS 节点。

发给 rospy 的日志必须是 byte str，统一走 _log()。
"""

from __future__ import division, print_function, unicode_literals

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import ride_core as core  # noqa: E402


def _log(level, msg):
    """把 unicode 消息转成 byte str 再交给 rospy。"""
    import rospy
    text = msg.encode("utf-8") if isinstance(msg, type(u"")) else msg
    if level == "warn":
        rospy.logwarn(text)
    elif level == "err":
        rospy.logerr(text)
    else:
        rospy.loginfo(text)


def main(argv=None):
    import rospy
    import std_msgs.msg as std_msg

    rospy.init_node("ride_comfort_transmissibility", anonymous=False)

    def par(name, default):
        return rospy.get_param(name, default)

    params = core.RideParams(
        **{
            "suspension/fn_hz": par("/suspension/fn_hz", 1.2),
            "suspension/damping": par("/suspension/damping", 0.25),
            "road/input_accel": par("/road/input_accel", 0.5),
            "road/freq_hz": par("/road/freq_hz", 6.0),
            "check/pos_tolerance": par("/check/pos_tolerance", 0.02),
            "check/peak_tolerance": par("/check/peak_tolerance", 0.05),
            "check/high_freq_limit": par("/check/high_freq_limit", 0.1),
            "check/linear_tolerance": par("/check/linear_tolerance", 0.02),
            "scan/low": par("/scan/low", 0.05),
            "scan/high": par("/scan/high", 4.0),
            "scan/steps": par("/scan/steps", 400),
        }
    )

    spec = params.spec
    peak_r, peak_t = core.find_peak(spec, params.scan_low, params.scan_high,
                                    params.scan_steps)
    asserts = core.check_ride(
        spec,
        pos_tolerance=params.pos_tolerance,
        peak_tolerance=params.peak_tolerance,
        high_freq_limit=params.high_freq_limit,
        linear_tolerance=params.linear_tolerance,
    )

    report = core.format_report(spec, asserts)
    _log("info", report)
    _log("info", core.render_transmissibility(spec))

    pub_report = rospy.Publisher("/ride/report", std_msg.String,
                                 queue_size=1, latch=True)
    pub_curve = rospy.Publisher("/ride/curve", std_msg.Float64MultiArray,
                                queue_size=1, latch=True)
    pub_peak = rospy.Publisher("/ride/peak", std_msg.Float64MultiArray,
                               queue_size=1, latch=True)

    report_msg = std_msg.String()
    report_msg.data = report
    pub_report.publish(report_msg)

    curve_msg = std_msg.Float64MultiArray()
    curve = []
    for r, freq, t, aw in core.scan(spec, core.DEFAULT_RATIOS):
        curve.extend([r, t, aw])
    curve_msg.data = curve
    pub_curve.publish(curve_msg)

    peak_msg = std_msg.Float64MultiArray()
    peak_msg.data = [spec.peak_ratio(), spec.peak_value(), peak_r, peak_t,
                     spec.weighted_accel(spec.road_freq_hz)]
    pub_peak.publish(peak_msg)

    passed = len([a for a in asserts if a["passed"]])
    _log("info", u"已发布 /ride/report、/ride/curve、/ride/peak"
                 u"（断言 %d/%d，激励处等级「%s」）"
                 % (passed, len(asserts),
                    core.comfort_grade(spec.weighted_accel(spec.road_freq_hz))))

    rate = rospy.Rate(par("/rate/publish_hz", 1.0))
    while not rospy.is_shutdown():
        pub_report.publish(report_msg)
        pub_curve.publish(curve_msg)
        pub_peak.publish(peak_msg)
        rate.sleep()
    return 0


if __name__ == "__main__":
    sys.exit(main())
