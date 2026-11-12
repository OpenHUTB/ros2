#!/usr/bin/env python3
"""Mission supervisor: goal, enable, metrics and JSON result logging."""

from __future__ import annotations

import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from gvnn.runtime import ensure_venv

ensure_venv(__file__)

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String

from gvnn.ros_utils import pose_stamped


class MissionNode:
    def __init__(self) -> None:
        self.mode = rospy.get_param("~mode", "classical")
        self.scene = rospy.get_param("~scene", "warehouse")
        self.goal = [float(rospy.get_param("~goal_x", 22.0)),
                     float(rospy.get_param("~goal_y", 14.0))]
        self.duration = float(rospy.get_param("~duration", 90.0))
        self.success_radius = float(rospy.get_param("~success_radius", 0.85))
        self.output_dir = rospy.get_param("~output_dir", os.path.join(ROOT, "results"))
        os.makedirs(self.output_dir, exist_ok=True)

        self.truth = None
        self.odom = None
        self.slam = None
        self.state = {}
        self.coverage = 0.0
        self.collisions = 0
        self.started = None
        self.done = False
        self.truth_traj = []
        self.slam_traj = []
        self.odom_traj = []
        self.slam_errors = []
        self.path_length = 0.0
        self.last_truth = None
        self.last_metrics = rospy.Time(0)

        self.pub_goal = rospy.Publisher("/goal_pose", PoseStamped, queue_size=2, latch=True)
        self.pub_enable = rospy.Publisher("/gvnn/enable", Bool, queue_size=2)
        self.pub_metrics = rospy.Publisher("/gvnn/metrics", String, queue_size=2)
        self.pub_mode = rospy.Publisher("/gvnn/mode", String, queue_size=2, latch=True)
        rospy.Subscriber("/ground_truth/pose", PoseStamped, self.truth_cb, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber("/slam_pose", PoseStamped, self.slam_cb, queue_size=10)
        rospy.Subscriber("/gvnn/state", String, self.state_cb, queue_size=5)
        rospy.Subscriber("/gvnn/slam_status", String, self.slam_status_cb, queue_size=5)
        rospy.Subscriber("/gvnn/metrics_request", Empty, self.request_cb, queue_size=2)
        self.pub_mode.publish(String(data=self.mode))

    def truth_cb(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.truth = np.array([msg.pose.position.x, msg.pose.position.y, yaw])

    def odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])

    def slam_cb(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.slam = np.array([msg.pose.position.x, msg.pose.position.y, yaw])

    def state_cb(self, msg: String) -> None:
        try:
            self.state = json.loads(msg.data)
            self.collisions = int(self.state.get("collisions", self.collisions))
        except Exception:
            pass

    def slam_status_cb(self, msg: String) -> None:
        try:
            self.coverage = float(json.loads(msg.data).get("coverage", self.coverage))
        except Exception:
            pass

    def request_cb(self, _msg: Empty) -> None:
        self.finish("requested", success=False)

    def _start(self, now) -> None:
        if self.started is None and self.truth is not None:
            self.started = now
            self.pub_goal.publish(pose_stamped([self.goal[0], self.goal[1], 0.0], "map", now))
            rospy.Timer(rospy.Duration(1.0), self._enable_once, oneshot=True)
            rospy.loginfo("mission started: mode=%s scene=%s goal=%s", self.mode,
                          self.scene, self.goal)

    def _enable_once(self, _event) -> None:
        self.pub_enable.publish(Bool(data=True))

    def tick(self, event) -> None:
        now = rospy.Time.now()
        self._start(now)
        if self.done or self.started is None or self.truth is None:
            return
        if self.last_truth is not None:
            self.path_length += float(np.linalg.norm(self.truth[:2] - self.last_truth[:2]))
        self.last_truth = self.truth.copy()
        self.truth_traj.append(self.truth[:2].copy())
        if self.slam is not None:
            self.slam_traj.append(self.slam[:2].copy())
            self.slam_errors.append(float(np.linalg.norm(self.slam[:2] - self.truth[:2])))
        if self.odom is not None:
            self.odom_traj.append(self.odom[:2].copy())

        distance = float(np.linalg.norm(self.truth[:2] - np.asarray(self.goal)))
        elapsed = (now - self.started).to_sec()
        if distance < self.success_radius:
            self.finish("success", success=True)
        elif elapsed > self.duration:
            self.finish("timeout", success=False)
        else:
            if (now - self.last_metrics).to_sec() > 0.5:
                self.last_metrics = now
                self.pub_metrics.publish(String(data=json.dumps({
                    "mode": self.mode,
                    "time": round(elapsed, 2),
                    "distance": round(distance, 3),
                    "collisions": self.collisions,
                    "coverage": round(self.coverage, 3),
                    "ate": round(float(np.sqrt(np.mean(np.square(self.slam_errors)))), 3)
                    if self.slam_errors else None,
                    "success": False,
                })))

    def finish(self, reason: str, success: bool) -> None:
        if self.done:
            return
        self.done = True
        self.pub_enable.publish(Bool(data=False))
        elapsed = 0.0 if self.started is None else (rospy.Time.now() - self.started).to_sec()
        ate = float(np.sqrt(np.mean(np.square(self.slam_errors)))) if self.slam_errors else None
        result = {
            "mode": self.mode,
            "scene": self.scene,
            "success": bool(success),
            "reason": reason,
            "duration_s": round(elapsed, 2),
            "goal": self.goal,
            "final_distance_m": None if self.truth is None else round(
                float(np.linalg.norm(self.truth[:2] - np.asarray(self.goal))), 3),
            "collisions": int(self.collisions),
            "path_length_m": round(float(self.path_length), 3),
            "slam_ate_rmse_m": None if ate is None else round(ate, 4),
            "map_coverage": round(float(self.coverage), 4),
            "frames": len(self.truth_traj),
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        safe = f"ros_{self.mode}_{self.scene}.json"
        with open(os.path.join(self.output_dir, safe), "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        self.pub_metrics.publish(String(data=json.dumps({**result, "success": bool(success)})))
        rospy.loginfo("mission finished: %s", json.dumps(result, ensure_ascii=False))

    def spin(self) -> None:
        rospy.Timer(rospy.Duration(0.1), self.tick)
        rospy.loginfo("GVNN mission ready: %s / %s", self.mode, self.scene)
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("gvnn_mission")
    MissionNode().spin()
