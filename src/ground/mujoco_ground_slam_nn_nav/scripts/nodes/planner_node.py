#!/usr/bin/env python3
"""Global planner node: A* or neural local planner on the SLAM map."""

from __future__ import annotations

import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from gvnn.runtime import ensure_venv

ensure_venv(__file__)

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String

from gvnn.config import MAP_RESOLUTION
from gvnn.geometry import line_of_sight
from gvnn.planning.astar import GridPlanner
from gvnn.planning.neural_planner import NeuralPlanner
from gvnn.ros_utils import path as path_msg
from gvnn.ros_utils import pose_stamped


class PlannerNode:
    def __init__(self) -> None:
        self.width = float(rospy.get_param("~width", 24.0))
        self.height = float(rospy.get_param("~height", 16.0))
        self.goal = np.array([float(rospy.get_param("~goal_x", 22.0)),
                              float(rospy.get_param("~goal_y", 14.0))])
        self.use_neural = bool(rospy.get_param("~use_neural", False))
        self.model_path = rospy.get_param(
            "~planner_model",
            os.path.join(ROOT, "models", "planner_mlp.npz"))
        self.neural = None
        if self.use_neural and os.path.exists(self.model_path):
            try:
                self.neural = NeuralPlanner(self.model_path)
                rospy.loginfo("neural planner loaded: %s", self.model_path)
            except Exception as exc:
                rospy.logwarn("neural planner unavailable: %s", exc)
        self.use_neural = self.neural is not None

        self.planner = GridPlanner(self.width, self.height, 0.10, 0.42, 1.5)
        self.pose = np.array([2.0, 2.0, 0.0])
        self.occupancy = None
        self.unknown = None
        self.last_plan = rospy.Time(0)
        self.current_path = []
        self.last_status = rospy.Time(0)
        self.plan_count = 0

        self.pub_path = rospy.Publisher("/plan", Path, queue_size=2, latch=True)
        self.pub_status = rospy.Publisher("/gvnn/planner_status", String, queue_size=5)
        self.pub_goal = rospy.Publisher("/goal_pose", PoseStamped, queue_size=2, latch=True)
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=2)
        rospy.Subscriber("/slam_pose", PoseStamped, self.pose_cb, queue_size=5)
        rospy.Subscriber("/goal_pose", PoseStamped, self.goal_cb, queue_size=2)
        rospy.Subscriber("/gvnn/reset", Empty, self.reset_cb, queue_size=2)
        rospy.Subscriber("/gvnn/scene", String, self.scene_cb, queue_size=2)
        self.pub_goal.publish(pose_stamped([self.goal[0], self.goal[1], 0.0], "map",
                                           rospy.Time.now()))

    def map_cb(self, msg: OccupancyGrid) -> None:
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.occupancy = data == 100
        self.unknown = data == -1

    def pose_cb(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = np.array([msg.pose.position.x, msg.pose.position.y, yaw])

    def goal_cb(self, msg: PoseStamped) -> None:
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.last_plan = rospy.Time(0)
        rospy.loginfo("new goal: %.2f %.2f", self.goal[0], self.goal[1])

    def reset_cb(self, _msg: Empty) -> None:
        self.planner = GridPlanner(self.width, self.height, 0.10, 0.42, 1.5)
        self.current_path = []
        self.last_plan = rospy.Time(0)

    def scene_cb(self, _msg: String) -> None:
        self.reset_cb(Empty())

    def plan_once(self, stamp) -> None:
        if self.occupancy is None or self.unknown is None:
            return
        if (stamp - self.last_plan).to_sec() < 0.5:
            return
        self.last_plan = stamp
        self.planner.set_map(self.occupancy, self.unknown)
        path = None
        if self.use_neural:
            wp, cls, conf = self.neural.waypoint(self.pose, self.occupancy,
                                                 self.unknown, self.goal)
            if wp is not None and conf >= 0.50 and self._waypoint_ok(wp):
                path = [(float(self.pose[0]), float(self.pose[1])), wp]
        if path is None:
            astar = self.planner.plan(self.pose, self.goal)
            if astar is not None:
                path = astar
        if path is not None:
            self.current_path = path
            self.plan_count += 1
            self.pub_path.publish(path_msg(path, stamp))
            payload = {
                "mode": "neural" if self.use_neural and len(path) == 2 else "astar",
                "waypoints": len(path),
                "length": round(self.planner.path_cost(path), 3),
                "plans": self.plan_count,
            }
            self.pub_status.publish(String(data=json.dumps(payload)))

    def _waypoint_ok(self, wp) -> bool:
        occupied = ~self.planner.traversable
        return bool(line_of_sight(occupied, self.pose[:2], wp, 0.10, 1))

    def spin(self) -> None:
        rate = rospy.Rate(4.0)
        rospy.loginfo("GVNN planner ready: goal=(%.1f, %.1f) neural=%s",
                      self.goal[0], self.goal[1], self.use_neural)
        while not rospy.is_shutdown():
            self.plan_once(rospy.Time.now())
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("gvnn_planner")
    PlannerNode().spin()
