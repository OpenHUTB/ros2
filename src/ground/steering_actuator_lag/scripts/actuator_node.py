#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""steering_actuator_lag 的 ROS 节点：算一次阶跃响应，把报告与曲线发出去。

注意：Melodic（Python 2.7）上发给 rospy 的日志消息**必须是 byte str**，
否则 rosgraph 的格式化会在 str(unicode) 上抛 UnicodeEncodeError。所有日志
都经过 _log()，先在 py2 下把 unicode 编码成 utf-8 再交给 rospy。
"""

from __future__ import division, print_function, unicode_literals

import os
import sys
import traceback

import rospy
from std_msgs.msg import Float64, String

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import actuator_core as core


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


class ActuatorNode(object):

    def __init__(self):
        self.params = core.ActuatorParams.from_params(rospy.get_param)
        self.report_topic = rospy.get_param("topic/report",
                                            "/steering/report")
        self.curve_topic = rospy.get_param("topic/curve", "/steering/curve")
        self.rise_topic = rospy.get_param("topic/rise_time",
                                          "/steering/rise_time")
        self.report_pub = rospy.Publisher(self.report_topic, String,
                                          queue_size=1, latch=True)
        self.curve_pub = rospy.Publisher(self.curve_topic, String,
                                         queue_size=1, latch=True)
        self.rise_pub = rospy.Publisher(self.rise_topic, Float64,
                                        queue_size=1, latch=True)
        rospy.on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        try:
            _log("info", u"节点退出")
        except Exception:
            pass

    def run(self):
        params = self.params
        spec = params.spec

        _log("info", u"转向执行器节点已启动：时间常数 %.2f s，死区 ±%.2f 度，"
                     u"打角上限 %.1f 度/s，回正上限 %.1f 度/s"
             % (spec.time_constant, spec.dead_band_deg, spec.max_rate_deg,
                spec.center_rate_deg))

        try:
            response = core.simulate(
                spec=spec,
                step_deg=params.test_step_deg,
                dwell=params.test_dwell_seconds,
                dt=params.sim_step,
            )
            results = core.check_response(
                response,
                rise_limit=params.test_rise_limit_seconds,
                steady_tolerance=params.test_steady_tolerance_deg,
                center_limit=params.test_center_limit_seconds,
                center_tolerance=params.test_center_tolerance_deg,
            )
            report = core.format_report(spec, response, results)
        except core.ActuatorError as error:
            _log("err", u"参数不合法：%s" % (error,))
            return 1
        except Exception:
            _log("err", u"仿真异常：%s" % (traceback.format_exc(),))
            return 1

        curve = u"\n".join(core.render_curve(response))
        rate_curve = u"\n".join(core.render_rate(response))

        self.report_pub.publish(String(data=core.to_message_text(report)))
        self.curve_pub.publish(String(data=core.to_message_text(curve)))
        rise = response.rise_time()
        self.rise_pub.publish(Float64(data=rise if rise is not None else -1.0))

        _log("info", report)
        _log("info", u"指令与实际角度（o 指令、* 实际、- 零线）：")
        _log("info", curve)
        _log("info", u"角速度（# 打角、= 回正）：")
        _log("info", rate_curve)
        _log("info", u"已发布：%s / %s / %s"
             % (self.report_topic, self.curve_topic, self.rise_topic))

        rate = rospy.Rate(max(1.0, params.rate_publish_hz))
        while not rospy.is_shutdown():
            rate.sleep()
        return 0


def main():
    rospy.init_node("steering_actuator_lag")
    node = ActuatorNode()
    return node.run()


if __name__ == "__main__":
    sys.exit(main())
