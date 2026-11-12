#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""里程计轨迹记录/回放/比对的 ROS 节点。

区别于「算一次就打印」的工具：
1. 订阅 `/odom`，边收边把采样点累加成一条轨迹（有状态）；
2. 到 `record/seconds` 秒自动结算，把轨迹写成 CSV（有磁盘）；
3. 结算后把轨迹回放成 `nav_msgs/Path` 发到 `/trace/replay`，并与内置参考
   圆弧逐点比对，给出误差直方图与四条断言（有反馈）。
"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import io
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import String

import trace_core as core


def _log(level, message):
    """Melodic 的 rosgraph 会对非 ASCII unicode 调 str()，这里统一转成 byte str。"""
    text = message
    try:
        if isinstance(text, unicode):  # noqa: F821  (Python 2)
            text = text.encode("utf-8")
    except NameError:
        pass  # Python 3：str 直接可用
    getattr(rospy, "log" + level)(text)


def _yaw_to_quaternion(yaw):
    half = 0.5 * core.normalize_angle(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


class TraceRecorder(object):
    """记录 -> 落盘 -> 回放 -> 比对。"""

    def __init__(self):
        self.params = core.TraceParams.from_params(rospy.get_param)
        self.points = []
        self.mock_index = 0
        self.started_at = rospy.get_time()
        self.finished = False

        self.replay_pub = rospy.Publisher(self.params.topic_replay, Path,
                                          queue_size=1, latch=True)
        self.report_pub = rospy.Publisher(self.params.topic_report, String,
                                         queue_size=1, latch=True)

        self.mock_state = [0.0, 0.0, 0.0]  # x, y, yaw
        self.mock_pub = None
        if self.params.use_mock:
            self.mock_pub = rospy.Publisher(self.params.topic_odom, Odometry,
                                            queue_size=10)
            period = rospy.Duration(1.0 / self.params.rate)
            self.timer = rospy.Timer(period, self._mock_tick)
            rospy.Subscriber(self.params.topic_odom, Odometry, self._on_odom)

        self.finish_timer = rospy.Timer(
            rospy.Duration(self.params.seconds), self._on_finish, oneshot=True)

        _log("info", "轨迹记录节点已启动：记录 %.1f s，采样率 %.1f Hz，"
                     "形态=%s，参考半径 %.2f m，CSV=%s" % (
                         self.params.seconds, self.params.rate,
                         _mode_text(self.params.use_mock),
                         self.params.reference_radius, self.params.csv_path))

    # ------------------------------------------------------------ 订阅/模拟

    def _elapsed(self):
        return rospy.get_time() - self.started_at

    def _append(self, x, y, yaw, stamp):
        if self.finished:
            return
        if stamp < -1e-9 or stamp > self.params.seconds + 1e-6:
            return
        self.points.append(core.TracePoint(stamp, x, y, yaw))

    def _on_odom(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = core.normalize_angle(2.0 * math.atan2(orientation.z, orientation.w))
        if self.params.use_mock:
            return
        stamp = message.header.stamp.to_sec() - self.started_at
        self._append(position.x, position.y, yaw, stamp)

    def _mock_tick(self, event):
        """内置车辆：定步长积分一条圆弧，并把当前的位姿发给 /odom。"""
        dt = 1.0 / self.params.rate
        speed = self.params.mock_speed
        yaw_rate = speed / self.params.mock_radius
        self.mock_state[2] = core.normalize_angle(self.mock_state[2] + yaw_rate * dt)
        self.mock_state[0] += speed * math.cos(self.mock_state[2]) * dt
        self.mock_state[1] += speed * math.sin(self.mock_state[2]) * dt

        self.mock_index += 1
        simulated = self.mock_index * dt
        message = Odometry()
        message.header.stamp = rospy.Time.from_sec(self.started_at + simulated)
        message.header.frame_id = "map"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = self.mock_state[0]
        message.pose.pose.position.y = self.mock_state[1]
        quaternion = _yaw_to_quaternion(self.mock_state[2])
        message.pose.pose.orientation.x = quaternion[0]
        message.pose.pose.orientation.y = quaternion[1]
        message.pose.pose.orientation.z = quaternion[2]
        message.pose.pose.orientation.w = quaternion[3]
        message.twist.twist.linear.x = speed
        message.twist.twist.angular.z = yaw_rate
        self.mock_pub.publish(message)

        if self.mock_index == 1 and not self.points:
            self._append(0.0, 0.0, 0.0, 0.0)
        self._append(self.mock_state[0], self.mock_state[1],
                     self.mock_state[2], simulated)

    # ------------------------------------------------------------ 结算

    def _on_finish(self, event):
        if self.finished:
            return
        self.finished = True
        if self.points and self.points[-1].t < self.params.seconds - 1e-6:
            stamp = max(self.params.seconds, self.points[-1].t)
            self.points.append(core.TracePoint(stamp,
                                               self.points[-1].x,
                                               self.points[-1].y,
                                               self.points[-1].yaw))
        try:
            self.report = self._build_report()
        except core.TraceError as error:
            _log("err", "轨迹结算失败：%s" % error)
            return
        _log("info", self.report)
        self.report_pub.publish(core.to_message_text(self.report))

    def _build_report(self):
        trace = core.Trace(self.points, "recorded")
        reference = self.params.reference()
        results = core.check_trace(trace, reference, self.params.min_samples,
                                   self.params.max_error,
                                   self.params.length_tolerance)
        stats = core.compare(reference, trace)
        csv_note = self._write_csv(trace)
        self._publish_replay(trace)
        lines = [
            core.format_report(trace, reference, results, stats,
                               csv_path=csv_note,
                               header="里程计轨迹记录与回放（内置参考圆弧，不依赖 CARLA）"),
            "",
            "误差直方图（单位 mm）：",
        ]
        lines.extend(core.histogram([value * 1000.0 for value in
                                     trace.errors_against(reference)]))
        return "\n".join(lines)

    def _write_csv(self, trace):
        path = self.params.csv_path
        try:
            folder = os.path.dirname(os.path.abspath(path))
            if folder and not os.path.isdir(folder):
                os.makedirs(folder)
            handle = io.open(path, "w", encoding="utf-8")
            try:
                handle.write(trace.csv_text())
            finally:
                handle.close()
            return path
        except (IOError, OSError) as error:
            _log("warn", "CSV 写入失败：%s" % error)
            return "%s（写入失败）" % path

    def _publish_replay(self, trace):
        message = Path()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "map"
        for point in trace.points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            quaternion = _yaw_to_quaternion(point.yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            message.poses.append(pose)
        self.replay_pub.publish(message)
        _log("info", "已回放 %d 个位姿到 %s" % (
            len(message.poses), self.params.topic_replay))


def _mode_text(use_mock):
    return "内置模拟" if use_mock else "外部 /odom"


def main():
    rospy.init_node("odom_trace_recorder")
    recorder = TraceRecorder()

    def _on_shutdown():
        if not recorder.finished:
            _log("info", "节点在结算前被关闭，未生成报告")

    rospy.on_shutdown(_on_shutdown)
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
