#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""降落过程监控节点：订阅里程计 / 速度指令 / 桥接层状态，实时判"指令-运动"一致性。

话题
====
订阅：
    /uav/odom                      nav_msgs/Odometry   机载里程计（用 position.z）
    /uav/cmd_vel                   geometry_msgs/Twist 上游下发的速度指令（用 linear.z）
    /uav/status                    std_msgs/String     桥接层状态（HOVER / GOAL / GOAL_DONE / VELOCITY）
    /uav/goal                      geometry_msgs/Point 目标点（可选，用来推断落地目标高度）
    /rooftop_nn/landing_point      geometry_msgs/Point 落点（可选，优先级高于 goal）
发布：
    /uav/landing_monitor/alert     std_msgs/String     一条告警一行 JSON
    /uav/landing_monitor/state     std_msgs/String     周期状态 JSON

参数见 ``config/uav_landing_monitor.yaml``；命令行：
    roslaunch uav_landing_monitor main.launch
    roslaunch uav_landing_monitor main.launch csv:=/tmp/landing.csv
"""
from __future__ import annotations

import json
import os
import sys

import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_core import (  # noqa: E402
    LandingMonitor, LEVEL_ERROR, LEVEL_INFO, LEVEL_WARN, PHASE_LANDED,
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

        self.z_up = bool(rospy.get_param("~frame/z_up", True))
        self.rate_hz = float(rospy.get_param("~monitor/rate_hz", 10.0))
        self.state_rate = float(rospy.get_param("~monitor/state_hz", 2.0))
        self.csv_path = str(rospy.get_param("~monitor/csv", ""))
        self.verbose = bool(rospy.get_param("~monitor/verbose", True))

        self.target_mode = str(rospy.get_param("~target/mode", "auto"))   # auto | landing | goal_offset | fixed
        self.target_fixed = rospy.get_param("~target/z", None)
        self.target_goal_offset = float(rospy.get_param("~target/goal_offset_m", 5.0))

        cfg = {}
        for key in ("stale_input_s", "cmd_min_vz", "cmd_effect_window_s", "cmd_effect_ratio",
                    "cmd_effect_min_expect_m", "descend_grace_s",
                    "descend_stall_s", "descend_stall_eps_m",
                    "free_fall_window_s", "free_fall_rate", "free_fall_cmd_slack",
                    "landing_tol_m", "overshoot_m", "timeout_s", "alert_cooldown_s"):
            val = rospy.get_param("~thresholds/%s" % key, None)
            if val is not None:
                cfg[key] = float(val)

        self.core = LandingMonitor(cfg)
        if self.target_fixed is not None:
            self.core.set_target_z(float(self.target_fixed))

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

    # ------------------------------------------------------------------ 回调
    def on_odom(self, msg):
        p = msg.pose.pose.position
        z = float(p.z) if self.z_up else -float(p.z)
        self.core.on_odom(rospy.get_time(), z)

    def on_cmd(self, msg):
        self.core.on_cmd(rospy.get_time(), float(msg.linear.z))

    def on_status(self, msg):
        self.core.on_status(rospy.get_time(), msg.data)

    def on_goal(self, msg):
        if self.target_mode in ("auto", "goal_offset") and self.target_mode != "fixed":
            if self.target_mode == "goal_offset" or self.core.target_z is None:
                self.core.set_target_z(float(msg.z) - self.target_goal_offset)
                rospy.loginfo("[landing_monitor] 由 /uav/goal 推断落地目标高度 %.2f m", self.core.target_z)

    def on_landing(self, msg):
        if self.target_mode in ("auto", "landing") and self.target_mode != "fixed":
            if self.target_mode == "landing" or self.core.target_z is None or \
                    abs(float(msg.z) - self.core.target_z) > 1e-6:
                self.core.set_target_z(float(msg.z))
                rospy.loginfo("[landing_monitor] 由落点话题设定落地目标高度 %.2f m", self.core.target_z)

    # ------------------------------------------------------------------ 主循环
    def on_timer(self, _event=None):
        now = rospy.get_time()
        alerts = self.core.tick(now)
        for a in alerts:
            payload = json.dumps(a.as_dict(), ensure_ascii=False)
            self.pub_alert.publish(String(data=payload))
            if self.verbose:
                log = rospy.logerr if a.level == LEVEL_ERROR else (
                    rospy.logwarn if a.level == LEVEL_WARN else rospy.loginfo)
                log("[landing_monitor] %s %s", a.code, a.msg)

        if self.csv_file is not None:
            codes = "|".join(a.code for a in alerts)
            self.csv_file.write("%.3f,%s,%.3f,%.3f,%.3f,%s,%s\n"
                                % (now - (self.core.t_first or now), self.core.phase,
                                   self.core.z if self.core.z is not None else float("nan"),
                                   self.core.target_z if self.core.target_z is not None else float("nan"),
                                   self.core.vz_cmd, self.core.status.replace(",", " "), codes))
            self.csv_file.flush()

        if now - self._last_state_t >= 1.0 / max(self.state_rate, 1e-3):
            self._last_state_t = now
            state = {"phase": self.core.phase,
                     "z": None if self.core.z is None else round(self.core.z, 3),
                     "target_z": None if self.core.target_z is None else round(self.core.target_z, 3),
                     "vz_cmd": round(self.core.vz_cmd, 3),
                     "status": self.core.status,
                     "alerts": len(self.core.alerts)}
            self.pub_state.publish(String(data=json.dumps(state, ensure_ascii=False)))

        if self.core.phase in (PHASE_LANDED, "ABORTED") and not self._finished_reported:
            self._finished_reported = True
            rospy.loginfo("[landing_monitor] %s", self.core.report_text().replace("\n", "\n  "))

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
