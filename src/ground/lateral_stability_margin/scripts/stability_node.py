#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""lateral_stability_margin 的 ROS 节点：算完就发三条 latched 话题，然后保活。

注意：发给 rospy 的日志必须是 **byte str**（Python 2.7 下 str），
否则 rosgraph/roslogging.py 里的 str(record_message) 会抛
UnicodeEncodeError。统一走 _log()。
"""

from __future__ import division, print_function, unicode_literals

import os
import sys

try:
    _HERE = os.path.dirname(os.path.abspath(__file__))
    if _HERE not in sys.path:
        sys.path.insert(0, _HERE)
    import stability_core as core
except ImportError:  # pragma: no cover
    from . import stability_core as core


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

    rospy.init_node("lateral_stability_margin", anonymous=False)

    def par(name, default):
        return rospy.get_param(name, default)

    params = core.StabilityParams(
        **{
            "vehicle/mass": par("/vehicle/mass", 1500.0),
            "vehicle/wheelbase": par("/vehicle/wheelbase", 2.7),
            "vehicle/cg_to_front": par("/vehicle/cg_to_front", 1.2),
            "vehicle/gravity": par("/vehicle/gravity", 9.81),
            "tire/front_cornering": par("/tire/front_cornering", 80000.0),
            "tire/rear_cornering": par("/tire/rear_cornering", 95000.0),
            "scene/radius": par("/scene/radius", 50.0),
            "scene/speed_kmh": par("/scene/speed_kmh", 40.0),
            "check/understeer_min": par("/check/understeer_min", 0.0),
            "check/ackermann_speed_kmh": par("/check/ackermann_speed_kmh", 2.0),
            "check/ackermann_tolerance": par("/check/ackermann_tolerance", 0.02),
        }
    )

    spec = params.spec
    cornering = core.Cornering(spec, radius=params.radius,
                               speed_kmh=params.speed_kmh)
    asserts = core.check_stability(
        cornering,
        understeer_min=params.understeer_min,
        ackermann_speed_kmh=params.ackermann_speed_kmh,
        ackermann_tolerance=params.ackermann_tolerance,
    )

    report = core.format_report(spec, cornering, asserts, params.speeds)
    _log("info", report)
    _log("info", core.render_speed_table(spec, params.radius, params.speeds))

    pub_report = rospy.Publisher("/stability/report", std_msg.String,
                                 queue_size=1, latch=True)
    pub_corner = rospy.Publisher("/stability/cornering",
                                 std_msg.Float64MultiArray,
                                 queue_size=1, latch=True)
    pub_limit = rospy.Publisher("/stability/limit",
                                std_msg.Float64MultiArray,
                                queue_size=1, latch=True)

    report_msg = std_msg.String()
    report_msg.data = report
    pub_report.publish(report_msg)

    corner_msg = std_msg.Float64MultiArray()
    corner_msg.data = [
        spec.gradient(),
        cornering.lateral_accel(),
        cornering.steer_angle(),
        cornering.ackermann_angle(),
        cornering.deflection(),
    ]
    pub_corner.publish(corner_msg)

    limit_msg = std_msg.Float64MultiArray()
    limit_msg.data = [
        float(spec.characteristic_speed() or -1.0),
        float(spec.critical_speed() or -1.0),
    ]
    pub_limit.publish(limit_msg)

    passed = len([a for a in asserts if a["passed"]])
    _log("info", u"已发布 /stability/report、/stability/cornering、"
                 u"/stability/limit（断言 %d/%d）" % (passed, len(asserts)))

    rate = rospy.Rate(par("/rate/publish_hz", 1.0))
    while not rospy.is_shutdown():
        pub_report.publish(report_msg)
        pub_corner.publish(corner_msg)
        pub_limit.publish(limit_msg)
        rate.sleep()
    return 0


if __name__ == "__main__":
    sys.exit(main())
