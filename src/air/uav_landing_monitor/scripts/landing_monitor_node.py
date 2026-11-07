#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""降落过程监控节点：订阅里程计 / 速度指令 / 桥接层状态，实时判"指令-运动"一致性。

话题
====
订阅：
    /uav/odom                      nav_msgs/Odometry   机载里程计（用 position.z）
    /uav/cmd_vel                   geometry_msgs/Twist 上游下发的速度指令（用 linear.z）
    /uav/status                    std_msgs/String     桥接层状态（READY / GOAL / GOAL_DONE / VELOCITY / HOVER …）
    /uav/goal                      geometry_msgs/Point 目标点（可选，用来推断落地目标高度）
    /rooftop_nn/landing_point      geometry_msgs/Point 落点（可选，优先级高于 goal）
发布：
    /uav/landing_monitor/alert     std_msgs/String     一条告警一行 JSON
    /uav/landing_monitor/state     std_msgs/String     周期状态 JSON

坐标：核心判据只用 ENU（上为正）。``frame/z_up:=false`` 时，本节点会把 **高度、速度指令、
目标高度三处一起** 取负（``monitor_core.to_enu``），不会出现"z 换了、vz 没换"的半实现。
"""
from __future__ import annotations

import json
import os
import sys
import time

import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_core import (  # noqa: E402
    Alert, DEFAULT_CONFIG, LEVEL_ERROR, LEVEL_INFO, LEVEL_WARN, LEVEL_OF,
    PHASE_ABORTED, PHASE_LANDED, LandingMonitor, parse_bool, to_enu,
)


class LandingMonitorNode(object):
    def __init__(self):
        self.topic_odom = str(rospy.get_param("~topic/odom", "/uav/odom"))
        self.topic_cmd = str(rospy.get_param("~topic/cmd_vel", "/uav/cmd_vel"))
        self.topic_status = str(rospy.get_param("~topic/status", "/uav/status"))
        self.topic_goal = str(rospy.get_param("~topic/goal", "/uav/goal"))
        self.topic_landing = str(rospy.get_param("~topic/landing_point", "/rooftop_nn/landing_point"))
        self.topic_alert = str(rospy.get_param("~topic/alert", "/uav/landing_monitor/alert"))
        self.topic_state = str(rospy.get_param("~topic/state", "/uav/landing_monitor/state"))

        self.z_up = parse_bool(rospy.get_param("~frame/z_up", True), True)
        self.rate_hz = float(rospy.get_param("~monitor/rate_hz", 10.0))
        self.state_rate = float(rospy.get_param("~monitor/state_hz", 2.0))
        self.csv_path = str(rospy.get_param("~monitor/csv", ""))
        self.verbose = parse_bool(rospy.get_param("~monitor/verbose", True), True)

        self.target_mode = str(rospy.get_param("~target/mode", "auto"))
        self.target_fixed = rospy.get_param("~target/z", None)
        self.target_goal_offset = float(rospy.get_param("~target/goal_offset_m", 5.0))

        cfg = {}
        for key in DEFAULT_CONFIG:
            val = rospy.get_param("~thresholds/%s" % key, None)
            if val is not None:
                cfg[key] = val if isinstance(val, str) else float(val)
        self.core = LandingMonitor(cfg)
        for key in self.core.unknown_keys:
            rospy.logwarn("[landing_monitor] 未知阈值键 thresholds/%s 已被忽略（拼写错误？）", key)
        if self.target_fixed is not None:
            self.core.set_target_z(to_enu(self.target_fixed, self.z_up))

        # 时钟：use_sim_time 但没 /clock 时 rospy.get_time() 恒为 0，所有基于时间的判据都会失效
        self._mono0 = time.monotonic()
        self._clock_mode = "ros"
        if not self._clock_alive():
            self._clock_mode = "monotonic"
            rospy.logerr("[landing_monitor] 检测到仿真时间没有推进（use_sim_time 但没有 /clock），"
                         "改用单调时钟；时间列与超时判据仍有效")

        self.pub_alert = rospy.Publisher(self.topic_alert, String, queue_size=20)
        self.pub_state = rospy.Publisher(self.topic_state, String, queue_size=2)
        rospy.Subscriber(self.topic_odom, Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(self.topic_cmd, Twist, self.on_cmd, queue_size=1)
        rospy.Subscriber(self.topic_status, String, self.on_status, queue_size=10)
        rospy.Subscriber(self.topic_goal, Point, self.on_goal, queue_size=1)
        rospy.Subscriber(self.topic_landing, Point, self.on_landing, queue_size=1)

        self.csv_file = None
        if self.csv_path:
            try:
                folder = os.path.dirname(os.path.abspath(self.csv_path))
                if folder and not os.path.isdir(folder):
                    os.makedirs(folder)
                self.csv_file = open(self.csv_path, "w", encoding="utf-8")
                self.csv_file.write("t,phase,z,target_z,vz_cmd,status,alerts\n")
            except Exception as exc:                        # pragma: no cover
                rospy.logwarn("[landing_monitor] 打不开 CSV %s: %s", self.csv_path, exc)
                self.csv_file = None

        self._last_state_t = 0.0
        self._finished_reported = False
        rospy.on_shutdown(self.on_shutdown)
        rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1e-3)), self.on_timer)

        rospy.loginfo("[landing_monitor] 已启动：odom=%s cmd=%s status=%s 告警=%s%s",
                      self.topic_odom, self.topic_cmd, self.topic_status, self.topic_alert,
                      ("，CSV=%s" % self.csv_path) if self.csv_file else "")
        rospy.loginfo("[landing_monitor] 生效阈值：%s", json.dumps(self.core.cfg, ensure_ascii=False))

    # ------------------------------------------------------------------ 时钟
    def _clock_alive(self):
        t1 = rospy.get_time()
        time.sleep(0.05)
        t2 = rospy.get_time()
        return t2 > 1e-6 and t2 > t1

    def now(self):
        if self._clock_mode == "ros":
            return rospy.get_time()
        return time.monotonic() - self._mono0

    # ------------------------------------------------------------------ 回调
    def on_odom(self, msg):
        self.core.on_odom(self.now(), to_enu(msg.pose.pose.position.z, self.z_up))

    def on_cmd(self, msg):
        self.core.on_cmd(self.now(), to_enu(msg.linear.z, self.z_up))

    def on_status(self, msg):
        self.core.on_status(self.now(), msg.data)

    def on_goal(self, msg):
        if self.target_mode == "fixed":
            return
        if self.target_mode == "goal_offset" or (
                self.target_mode == "auto" and self.core.target_z is None):
            self.core.set_target_z(to_enu(float(msg.z) - self.target_goal_offset, self.z_up))
            rospy.loginfo("[landing_monitor] 由 /uav/goal 推断落地目标高度 %.2f m"
                          "（= goal.z - %.1f m，该偏移是约定值，请确认与上游一致）",
                          self.core.target_z, self.target_goal_offset)

    def on_landing(self, msg):
        if self.target_mode in ("auto", "landing"):
            self.core.set_target_z(to_enu(float(msg.z), self.z_up))
            rospy.loginfo("[landing_monitor] 由落点话题设定落地目标高度 %.2f m", self.core.target_z)

    # ------------------------------------------------------------------ 主循环
    def on_timer(self, _event=None):
        now = self.now()
        alerts = self.core.tick(now)
        for a in alerts:
            self.pub_alert.publish(String(data=json.dumps(a.as_dict(), ensure_ascii=False)))
            if self.verbose:
                log = rospy.logerr if a.level == LEVEL_ERROR else (
                    rospy.logwarn if a.level == LEVEL_WARN else rospy.loginfo)
                log("[landing_monitor] %s %s", a.code, a.msg)

        if self.csv_file is not None:
            core = self.core
            t0 = core.t_first if core.t_first is not None else now
            self.csv_file.write("%.3f,%s,%.3f,%.3f,%.3f,%s,%s\n"
                                % (now - t0, core.phase,
                                   core.z if core.z is not None else float("nan"),
                                   core.target_z if core.target_z is not None else float("nan"),
                                   core.vz_cmd, str(core.status).replace(",", " "),
                                   "|".join(a.code for a in alerts)))
            self.csv_file.flush()

        if now - self._last_state_t >= 1.0 / max(self.state_rate, 1e-3):
            self._last_state_t = now
            rep = self.core.report()
            state = {"phase": rep["phase"],
                     "z": rep["final_z"],
                     "target_z": rep["target_z"],
                     "vz_cmd": round(self.core.vz_cmd, 3),
                     "status": self.core.status,
                     "active": rep["active"],
                     "alerts": rep["alert_counts"]}
            self.pub_state.publish(String(data=json.dumps(state, ensure_ascii=False)))

        if self.core.phase in (PHASE_LANDED, PHASE_ABORTED) and not self._finished_reported:
            self._finished_reported = True
            rospy.loginfo("[landing_monitor] %s", self.core.report_text().replace("\n", "\n  "))
        elif self.core.phase == "IDLE":
            self._finished_reported = False

    # ------------------------------------------------------------------ 收尾
    def on_shutdown(self):
        try:
            if self.csv_file is not None:
                self.csv_file.flush()
                self.csv_file.close()
        except Exception:                                   # pragma: no cover
            pass
        sys.stdout.write("\n" + self.core.report_text() + "\n")
        sys.stdout.flush()


def main():
    rospy.init_node("uav_landing_monitor", anonymous=False)
    LandingMonitorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
