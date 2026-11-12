#!/usr/bin/env python3
"""评测节点：真值 vs SLAM 位姿，输出 results/ros_<mode>_<scene>.json。"""

from __future__ import annotations

import json
import math
import os
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from rov_mujoco_slam_nn_nav.model import make_layout


class Metrics:
    def __init__(self):
        self.mode = rospy.get_param("~mode", "classical")
        self.scene = rospy.get_param("~scene", "warehouse")
        self.seed = int(rospy.get_param("~seed", 0))
        goal_x = float(rospy.get_param("~goal_x", float("nan")))
        goal_y = float(rospy.get_param("~goal_y", float("nan")))
        if math.isnan(goal_x) or math.isnan(goal_y):
            layout_goal = make_layout(self.scene, self.seed).goal
            goal_x, goal_y = float(layout_goal[0]), float(layout_goal[1])
        self.goal = (goal_x, goal_y)
        self.timeout = float(rospy.get_param("~timeout", 90.0))
        self.out = rospy.get_param("~output", "")
        self.gt = self.est = None
        self.state = {}
        self.errors = []
        self.travel = 0.0
        self.prev = None
        self.done = False
        self.t0 = rospy.Time.now()
        rospy.Subscriber("/rov/ground_truth", PoseStamped, self._on_gt, queue_size=1)
        rospy.Subscriber("/rov/slam_pose", PoseStamped, self._on_est, queue_size=1)
        rospy.Subscriber("/rov/state", String, self._on_state, queue_size=1)
        rospy.Subscriber("/nav_status", String, self._on_status, queue_size=1)
        rospy.Timer(rospy.Duration(0.5), self._tick)

    def _on_state(self, msg):
        try:
            self.state = json.loads(msg.data)
        except ValueError:
            pass

    def _on_gt(self, msg):
        p = (msg.pose.position.x, msg.pose.position.y)
        if self.prev is not None:
            self.travel += math.hypot(p[0] - self.prev[0], p[1] - self.prev[1])
        self.prev = p
        self.gt = p

    def _on_est(self, msg):
        self.est = (msg.pose.position.x, msg.pose.position.y)
        if self.gt:
            self.errors.append(math.hypot(self.est[0] - self.gt[0], self.est[1] - self.gt[1]))

    def _on_status(self, msg):
        if msg.data == "done":
            self._write()

    def _tick(self, _):
        if not self.done and (rospy.Time.now() - self.t0).to_sec() > self.timeout:
            self._write()

    def _write(self):
        if self.done:
            return
        self.done = True
        out = self.out or os.path.join("results", f"ros_{self.mode}_{self.scene}.json")
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        dist = None
        if self.gt:
            dist = math.hypot(self.goal[0] - self.gt[0], self.goal[1] - self.gt[1])
        fallen = bool(self.state.get("fallen", False))
        collisions = int(self.state.get("collisions", 0))
        data = {
            "mode": self.mode, "scene": self.scene, "seed": self.seed,
            # success 与离线口径一致：到达 0.5 m 内且未倾覆（不含碰撞约束）；
            # success_no_collision 额外要求全程 0 碰撞。
            "success": bool(dist is not None and dist < 0.5 and not fallen),
            "success_no_collision": bool(dist is not None and dist < 0.5
                                         and not fallen and collisions == 0),
            "fallen": fallen,
            "collisions": collisions,
            "time_s": round((rospy.Time.now() - self.t0).to_sec(), 2),
            "travel_m": round(self.travel, 3),
            "final_dist_m": round(dist, 3) if dist else None,
            "ate_rmse_m": round(float(np.sqrt(np.mean(np.square(self.errors)))), 4)
            if self.errors else None,
        }
        with open(out, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        rospy.loginfo("指标写入 %s: %s", out, data)


if __name__ == "__main__":
    rospy.init_node("rov_metrics")
    Metrics()
    rospy.spin()

