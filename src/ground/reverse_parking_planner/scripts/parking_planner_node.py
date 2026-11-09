#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""倒车入库规划节点。

把 parking_core 算出来的路径发布成 ROS 消息：

* ``<topic/plan>``   ``nav_msgs/Path``，一次性 latched 发布，可在 RViz 里直接看；
* ``<topic/report>`` ``std_msgs/String``，报告全文（含断言与俯视图），latched；
* ``<topic/point>``  ``geometry_msgs/PoseStamped``，按 ``rate/publish_hz`` 逐点回放。

节点本身不控制车辆，也不订阅任何话题，所以有没有 CARLA、有没有车都能跑。
"""

from __future__ import division, print_function, unicode_literals

import sys
import traceback

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

import parking_core as core

try:                       # Python 2
    text_type = unicode    # noqa: F821
except NameError:          # Python 3
    text_type = str


def _log(level, message):
    """Melodic 上给 rospy 打日志必须传 byte str，否则 rosgraph 会炸。"""
    if isinstance(message, bytes):
        text = message
    else:
        text = text_type(message)
        if sys.version_info[0] < 3:
            text = text.encode("utf-8")
    getattr(rospy, level)(text)


class ParkingPlanner(object):
    """读参数、算路径、发消息。"""

    def __init__(self):
        self.params = core.PlannerParams.from_params(rospy.get_param)
        _log("loginfo",
             "参数：车位 %.2f x %.2f m，转弯半径 %.2f m，直线段 %.2f m，"
             "频率 %.1f Hz，输出 %s"
             % (self.params.slot_width, self.params.slot_length,
                self.params.turn_radius, self.params.entry_length,
                self.params.publish_hz, self.params.plan_topic))

        self.vehicle = self.params.make_vehicle()
        self.slot = self.params.make_slot()
        self.plan = core.build_plan(self.vehicle, self.slot,
                                    self.params.turn_radius,
                                    self.params.entry_length,
                                    self.params.step)
        self.results = core.evaluate_plan(self.plan,
                                          self.params.tolerance,
                                          self.params.clearance,
                                          self.params.max_path)
        self.report = core.format_report(self.plan, self.results,
                                         header="倒车入库规划报告")

        self.plan_pub = rospy.Publisher(self.params.plan_topic, Path,
                                        queue_size=1, latch=True)
        self.report_pub = rospy.Publisher(self.params.report_topic, String,
                                          queue_size=1, latch=True)
        self.point_pub = rospy.Publisher(self.params.point_topic, PoseStamped,
                                         queue_size=10)
        self.index = 0

    def to_pose_stamped(self, pose):
        message = PoseStamped()
        message.header.frame_id = self.params.frame_id
        message.header.stamp = rospy.Time.now()
        message.pose.position.x = pose.x
        message.pose.position.y = pose.y
        message.pose.position.z = 0.0
        qx, qy, qz, qw = core.yaw_to_quaternion(pose.yaw)
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        return message

    def publish_plan(self):
        path = Path()
        path.header.frame_id = self.params.frame_id
        path.header.stamp = rospy.Time.now()
        for pose in self.plan["poses"]:
            path.poses.append(self.to_pose_stamped(pose))
        self.plan_pub.publish(path)
        _log("loginfo", "已发布路径 %s（%d 个点，latched）"
             % (self.params.plan_topic, len(path.poses)))

    def publish_report(self):
        self.report_pub.publish(String(data=core.to_message_text(self.report)))
        _log("loginfo", "已发布报告 %s（latched）" % self.params.report_topic)

    def run(self):
        self.publish_plan()
        self.publish_report()
        print(core.to_message_text(self.report))

        total = len(self.plan["poses"])
        rate = rospy.Rate(max(1.0, self.params.publish_hz))
        _log("loginfo", "开始按 %.1f Hz 回放 %d 个采样点"
             % (max(1.0, self.params.publish_hz), total))
        while not rospy.is_shutdown() and self.index < total:
            self.point_pub.publish(self.to_pose_stamped(self.plan["poses"][self.index]))
            self.index += 1
            rate.sleep()

        if not rospy.is_shutdown():
            _log("loginfo", "路径回放完成，节点保持运行以便查看 latched 话题")
        return 0


def main():
    rospy.init_node("reverse_parking_planner")
    try:
        planner = ParkingPlanner()
    except core.ParkingError as exc:
        _log("logerr", "规划参数不合法：%s" % (exc,))
        return 1
    except Exception:
        _log("logerr", "节点初始化异常：%s" % (traceback.format_exc(),))
        return 1

    try:
        return planner.run()
    except Exception:
        _log("logerr", "节点运行异常：%s" % (traceback.format_exc(),))
        return 1


if __name__ == "__main__":
    sys.exit(main())
