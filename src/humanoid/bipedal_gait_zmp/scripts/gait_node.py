#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把双足步态仿真结果发布成 ROS 话题，供 rqt_plot / RViz 查看。

离线分析（``main.py``）只算一遍、打印指标、出 PNG。本节点做的是同一件事，
但结果走 ROS 话题发布出来，可以被其他节点订阅、录制、画曲线：

* 启动时算一遍闭环步态（DCM 反馈 + ZMP 限幅），把整条质心与 ZMP 轨迹
  分别发布成 ``nav_msgs/Path``（latched，订阅方后起也能拿到）
* 按 ``~playback_rate`` 倍速回放，用定时器发布当前时刻的
  ``geometry_msgs/PoseStamped`` 与稳定裕度 ``std_msgs/Float64``
* 每一步的落脚点发布成 ``visualization_msgs/MarkerArray``，
  当前支撑区间发布成 ``visualization_msgs/Marker``（地面上的线段）

话题：
    /bipedal_gait/com_path     nav_msgs/Path                 整条质心轨迹
    /bipedal_gait/zmp_path     nav_msgs/Path                 整条 ZMP 轨迹
    /bipedal_gait/com_pose     geometry_msgs/PoseStamped     当前质心
    /bipedal_gait/zmp_pose     geometry_msgs/PoseStamped     当前 ZMP
    /bipedal_gait/margin       std_msgs/Float64              当前稳定裕度
    /bipedal_gait/footsteps    visualization_msgs/MarkerArray 落脚点
    /bipedal_gait/support      visualization_msgs/Marker      当前支撑区间

参数（私有参数，可在 launch 里覆盖）：
    ~step_length    步长 (m)，默认 0.35
    ~step_time      步时 (s)，默认 0.5
    ~duty           支撑相占比，默认 0.6
    ~n_steps        落脚点数，默认 12
    ~com_height     质心高度 (m)，默认 0.8
    ~gain           DCM 反馈增益，默认 1.0
    ~playback_rate  回放倍速，默认 1.0
    ~publish_rate   定时器频率 (Hz)，默认 50
    ~loop           走完后是否重头再来，默认 True
    ~frame_id       坐标系，默认 "map"
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import rospy  # noqa: E402
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

from dcm import DEFAULT_GAIN, track  # noqa: E402
from gait import build_symmetric_plan  # noqa: E402
from lipm import LIPMParams  # noqa: E402

DT = 0.001          # 仿真步长，与离线演示保持一致
FOOT_HALF = 0.12    # 脚掌半长，仅用于画落脚点方块


def _pose(x, frame_id):
    """x 轴上的一维轨迹点 -> PoseStamped。"""
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = frame_id
    ps.pose = Pose(Point(x, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
    return ps


class GaitNode(object):
    """步态结果的 ROS 发布端。"""

    def __init__(self):
        rospy.init_node("bipedal_gait_zmp")

        gp = rospy.get_param
        self.frame_id = gp("~frame_id", "map")
        self.step_length = float(gp("~step_length", 0.35))
        self.step_time = float(gp("~step_time", 0.5))
        self.duty = float(gp("~duty", 0.6))
        self.n_steps = int(gp("~n_steps", 12))
        self.com_height = float(gp("~com_height", 0.8))
        self.gain = float(gp("~gain", DEFAULT_GAIN))
        self.playback_rate = float(gp("~playback_rate", 1.0))
        self.publish_rate = float(gp("~publish_rate", 50.0))
        self.loop = bool(gp("~loop", True))

        if self.step_time <= 0 or self.playback_rate <= 0 or self.publish_rate <= 0:
            raise ValueError("step_time / playback_rate / publish_rate 必须为正")

        self.params = LIPMParams(z_c=self.com_height)
        self.plan = build_symmetric_plan(
            n_steps=self.n_steps, step_length=self.step_length,
            step_time=self.step_time, duty=self.duty)
        self.v_avg = self.step_length / self.step_time

        rospy.loginfo("计算闭环步态：%d 步、步长 %.2f m、步时 %.2f s、增益 %.2f",
                      self.n_steps, self.step_length, self.step_time, self.gain)
        self.traj = track(self.plan, self.params, 0.0, 0.0, self.v_avg,
                          dt=DT, gain=self.gain)
        self.n = len(self.traj["time"])
        self.duration = self.traj["time"][-1] - self.traj["time"][0]
        rospy.loginfo("轨迹 %d 点，时长 %.2f s，平均速度 %.3f m/s",
                      self.n, self.duration, self.v_avg)

        self.pub_com_path = rospy.Publisher(
            "/bipedal_gait/com_path", Path, queue_size=1, latch=True)
        self.pub_zmp_path = rospy.Publisher(
            "/bipedal_gait/zmp_path", Path, queue_size=1, latch=True)
        self.pub_com_pose = rospy.Publisher(
            "/bipedal_gait/com_pose", PoseStamped, queue_size=1)
        self.pub_zmp_pose = rospy.Publisher(
            "/bipedal_gait/zmp_pose", PoseStamped, queue_size=1)
        self.pub_margin = rospy.Publisher(
            "/bipedal_gait/margin", Float64, queue_size=1)
        self.pub_footsteps = rospy.Publisher(
            "/bipedal_gait/footsteps", MarkerArray, queue_size=1, latch=True)
        self.pub_support = rospy.Publisher(
            "/bipedal_gait/support", Marker, queue_size=1)

        self._publish_paths()
        self._publish_footsteps()

        self.t0 = rospy.get_time()
        self.finished = False
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._tick)
        rospy.on_shutdown(self._on_shutdown)

    # ------------------------------------------------------------ 一次发布
    def _publish_paths(self):
        now = rospy.Time.now()
        for pub, key in ((self.pub_com_path, "com_x"),
                         (self.pub_zmp_path, "zmp_actual")):
            path = Path()
            path.header.stamp = now
            path.header.frame_id = self.frame_id
            path.poses = [_pose(x, self.frame_id) for x in self.traj[key]]
            pub.publish(path)

    def _publish_footsteps(self):
        arr = MarkerArray()
        for i, step in enumerate(self.plan.steps):
            m = Marker()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = self.frame_id
            m.ns = "footsteps"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position = Point(step.center, 0.0, 0.0)
            m.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
            m.scale = Point(2 * step.half_length, 0.10, 0.02)
            if step.side == "L":
                m.color.r, m.color.g, m.color.b = 0.16, 0.45, 0.86
            else:
                m.color.r, m.color.g, m.color.b = 0.85, 0.35, 0.12
            m.color.a = 0.85
            arr.markers.append(m)
        self.pub_footsteps.publish(arr)

    def _publish_support(self, i):
        lo = self.traj["poly_lo"][i]
        hi = self.traj["poly_hi"][i]
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self.frame_id
        m.ns = "support"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        m.scale.x = 0.03
        m.color.r, m.color.g, m.color.b, m.color.a = 0.90, 0.25, 0.20, 1.0
        m.points = [Point(lo, 0.0, 0.0), Point(hi, 0.0, 0.0)]
        self.pub_support.publish(m)

    # ------------------------------------------------------------ 定时器
    def _tick(self, _event):
        elapsed = (rospy.get_time() - self.t0) * self.playback_rate
        if elapsed > self.duration:
            if not self.loop:
                if not self.finished:
                    self.finished = True
                    rospy.loginfo("回放结束（%.2f s），节点保持运行", self.duration)
                return
            self.t0 = rospy.get_time()
            elapsed = 0.0

        i = min(int(round(elapsed / DT)), self.n - 1)
        self.pub_com_pose.publish(_pose(self.traj["com_x"][i], self.frame_id))
        self.pub_zmp_pose.publish(_pose(self.traj["zmp_actual"][i], self.frame_id))
        self.pub_margin.publish(Float64(self.traj["margin"][i]))
        self._publish_support(i)

    def _on_shutdown(self):
        self.timer.shutdown()
        rospy.loginfo("bipedal_gait_zmp 退出")


def main():
    try:
        GaitNode()
    except rospy.ROSInterruptException:
        return 0
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
