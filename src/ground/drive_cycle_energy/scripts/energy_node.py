#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""工况能耗节点的 ROS 封装（ROS 1 / Melodic；Python 2.7 & 3.x）。

节点只做三件事：
  1. 从参数服务器读整车参数与工况参数；
  2. 调 energy_core 积分出一段工况的能量与续航，跑四条断言；
  3. 把报告、逐项能量（CSV）、续航分别发布到三个 latched 话题，然后打印一遍并保活。

注意：roslaunch 打出的日志由 rosgraph 转成 Python 2 的 str，如果消息里有中文，
必须先在 _log() 里 encode 成 byte str，否则 Melodic 的 roslogging 会抛
UnicodeEncodeError（rosgraph/roslogging.py:249 的 str(record_message)）。
"""

from __future__ import division, print_function, unicode_literals

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import rospy  # noqa: E402
from std_msgs.msg import Float64, String  # noqa: E402

import energy_core as core  # noqa: E402

PARAM_KEYS = (
    "vehicle/mass", "vehicle/c_rr", "vehicle/cd", "vehicle/area",
    "vehicle/battery_kwh", "vehicle/regen_eff",
    "check/breakdown_tolerance", "check/regen_limit",
    "rate/publish_hz", "topic/report", "topic/breakdown", "topic/range",
)

DEFAULT_CYCLE = core.DEFAULT_CYCLE


def _log(level, msg):
    """把 unicode 消息压成 byte str 再交给 rospy，规避 Melodic 的编码问题。"""
    text = msg
    if isinstance(text, type(u"")):
        text = text.encode("utf-8", "replace")
    getattr(rospy, level)(text)


def read_params():
    values = {}
    for key in PARAM_KEYS:
        values[key] = rospy.get_param("/" + key)
    return values


def main():
    rospy.init_node("drive_cycle_energy", anonymous=False)
    params = read_params()

    spec = core.VehicleSpec(
        mass=params["vehicle/mass"],
        c_rr=params["vehicle/c_rr"],
        cd=params["vehicle/cd"],
        area=params["vehicle/area"],
        battery_kwh=params["vehicle/battery_kwh"],
        regen_eff=params["vehicle/regen_eff"],
    )

    report_pub = rospy.Publisher(params["topic/report"], String, queue_size=1,
                                 latch=True)
    breakdown_pub = rospy.Publisher(params["topic/breakdown"], String,
                                    queue_size=1, latch=True)
    range_pub = rospy.Publisher(params["topic/range"], Float64, queue_size=1,
                                latch=True)

    _log("loginfo", u"工况能耗节点已启动：电池 %.1f kWh，回收 η %.2f，工况 %d 段"
         % (spec.battery_kwh, spec.regen_eff, len(DEFAULT_CYCLE)))

    result = core.simulate(spec, DEFAULT_CYCLE)
    asserts = core.check_energy(
        spec, result,
        breakdown_tolerance=params["check/breakdown_tolerance"],
        regen_limit=params["check/regen_limit"],
        range_tolerance=0.02,
    )
    report = core.format_report(spec, result, asserts)
    for line in report.splitlines():
        _log("loginfo", line)

    keys = ("roll_j", "aero_j", "brake_j", "regen_j", "ke_j", "drive_j", "net_j")
    csv_text = "item,joules\n" + "\n".join(
        "%s,%.3f" % (k, result[k]) for k in keys)
    _log("loginfo", core.render_energy(result))

    report_pub.publish(report)
    breakdown_pub.publish(csv_text)
    range_pub.publish(core.range_km(spec, result))
    _log("loginfo", u"已发布 %s / %s / %s"
         % (params["topic/report"], params["topic/breakdown"],
            params["topic/range"]))

    rate = rospy.Rate(params["rate/publish_hz"])
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
    _log("loginfo", u"已退出")


if __name__ == "__main__":
    main()
