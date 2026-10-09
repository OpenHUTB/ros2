#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_trajectory.py -- AllocNet 仿真数据记录

抓取键盘遥操作与航点规划过程中的坐标日志 (time, x, y, z, v, source)。

订阅
----
    /visualizer/spheres     ns="spheres"    无人机实时位置（蓝色小球，飞行中每 tick 一发）
    /visualizer/spheres     ns="StartGoal"  起点 / 终点航点球
    /visualizer/speed       std_msgs/Float64 实时速度
    /visualizer/trajectory  ns="trajectory"  规划出的轨迹（LINE_LIST，可用于回放）
    /teleop/cursor          ns="teleop_cursor" 键盘游标位置

输出
----
    CSV 列: time, x, y, z, v, source
      time   相对本节点启动的秒数
      source 来源标记: teleop | flight | start | goal | planned

用法
----
    rosrun planner record_trajectory.py _output:=/path/to/log.csv
"""

import csv
import os

import rospy
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray

FIELDS = ["time", "x", "y", "z", "v", "source"]


class TrajectoryRecorder(object):
    def __init__(self):
        self.output = rospy.get_param(
            "~output",
            os.path.expanduser("~/allocnet_trajectory.csv"))
        # 低于该时间间隔的重复点会被丢弃，避免 CSV 被 100Hz 的球体刷屏
        self.min_dt = rospy.get_param("~min_dt", 0.02)
        # 是否记录规划轨迹的采样点
        self.record_planned = rospy.get_param("~record_planned", True)

        self.t0 = rospy.Time.now().to_sec()
        self.last_t = {}
        self.rows = []
        self.last_speed = 0.0

        self._ensure_dir()
        self._write_header()

        rospy.Subscriber("/visualizer/spheres", Marker,
                         self.on_marker, queue_size=50)
        rospy.Subscriber("/visualizer/speed", Float64,
                         self.on_speed, queue_size=50)
        rospy.Subscriber("/teleop/cursor", Marker,
                         self.on_marker, queue_size=50)
        if self.record_planned:
            rospy.Subscriber("/visualizer/trajectory", Marker,
                             self.on_marker, queue_size=10)
        rospy.Subscriber("/teleop/waypoints", MarkerArray,
                         self.on_waypoint_array, queue_size=10)

        rospy.on_shutdown(self.flush)
        rospy.loginfo("[recorder] 记录到 %s", self.output)

    # ------------------------------------------------------------------
    def _ensure_dir(self):
        d = os.path.dirname(os.path.abspath(self.output))
        if d and not os.path.isdir(d):
            os.makedirs(d)

    def _write_header(self):
        with open(self.output, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=FIELDS).writeheader()

    def _add(self, x, y, z, source, v=None, throttle=True):
        """追加一个采样点。

        throttle=False 用于**批量**数据（如 /visualizer/trajectory 一次性
        发布的整条轨迹，1700+ 个点）。这类数据时间戳相同，若照常按
        min_dt 限流，只会写下第一个点，整条轨迹就丢了。
        """
        now = rospy.Time.now().to_sec()
        t = now - self.t0
        if throttle and t - self.last_t.get(source, -1e9) < self.min_dt:
            return
        self.last_t[source] = t
        self.rows.append({
            "time": round(t, 4),
            "x": round(x, 4),
            "y": round(y, 4),
            "z": round(z, 4),
            "v": round(v if v is not None else self.last_speed, 4),
            "source": source,
        })
        # 增量落盘，避免仿真中途崩溃丢数据
        if len(self.rows) >= 200:
            self.flush()

    # ------------------------------------------------------------------
    def on_marker(self, msg):
        if msg.action == Marker.DELETE or msg.action == Marker.DELETEALL:
            return
        ns = msg.ns
        if ns == "spheres":
            for p in msg.points:
                self._add(p.x, p.y, p.z, "flight")
        elif ns == "StartGoal":
            src = "start" if msg.id == 0 else "goal"
            for p in msg.points:
                self._add(p.x, p.y, p.z, src)
        elif ns == "trajectory":
            # LINE_LIST：点成对出现（段起点/段终点）。
            # 整条轨迹是一次性发布的，必须**关掉限流**逐点记录，
            # 否则只会记下第一个点。
            pts = [(p.x, p.y, p.z) for p in msg.points]
            seq = [pts[i] for i in range(0, len(pts) - 1, 2)]
            if pts:
                seq.append(pts[-1])
            for (x, y, z) in seq:
                self._add(x, y, z, "planned", throttle=False)
        elif ns == "teleop_cursor":
            # 只记球体（id=0），跳过朝向箭头
            if msg.id == 0:
                for p in msg.points:
                    self._add(p.x, p.y, p.z, "teleop")
            elif msg.type == Marker.SPHERE:
                self._add(msg.pose.position.x, msg.pose.position.y,
                          msg.pose.position.z, "teleop")

    def on_waypoint_array(self, msg):
        for m in msg.markers:
            if m.type == Marker.SPHERE:
                self._add(m.pose.position.x, m.pose.position.y,
                          m.pose.position.z, "goal")

    def on_speed(self, msg):
        self.last_speed = msg.data

    # ------------------------------------------------------------------
    def flush(self):
        if not self.rows:
            return
        try:
            with open(self.output, "a", newline="") as f:
                w = csv.DictWriter(f, fieldnames=FIELDS)
                w.writerows(self.rows)
            rospy.loginfo("[recorder] 已写入 %d 行 -> %s",
                          len(self.rows), self.output)
            self.rows = []
        except Exception as e:                       # noqa: BLE001
            rospy.logerr("[recorder] 写盘失败: %s", e)


def main():
    rospy.init_node("record_trajectory", anonymous=False)
    TrajectoryRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
