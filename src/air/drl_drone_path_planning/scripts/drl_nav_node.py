#!/usr/bin/env python3
"""ROS 1 node: DRL-based optimal path finding for a quadrotor.

The node subscribes to odometry and a 2-D laser scan, assembles the same 28-D
observation used during training, runs either the exported NumPy DRL policy or
the geometric baseline, and publishes a body-frame ``geometry_msgs/Twist``.
It also publishes the flown path and a short status string for RViz/rostopic.

中文说明：ROS 1 控制节点。订阅里程计与激光雷达，按训练时完全一致的 28 维观测
运行 DRL 策略（纯 NumPy 推理）或几何基线，输出机体系速度指令。支持多航点、
障碍刹停、路径可视化与状态发布。

Developer: 汤湘粤 (学号: 2535030067)
"""

from __future__ import annotations

import os
import threading
from typing import Any, Dict, Optional, Sequence

import numpy as np

try:
    import rospy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
except ImportError as exc:  # pragma: no cover - ROS-only dependency
    raise ImportError(
        "drl_nav_node.py must run inside a ROS 1 environment (Noetic recommended)"
    ) from exc

from baseline import baseline_action
from common import (
    DEFAULT_CONFIG_PATH,
    action_to_velocity,
    apply_altitude_hold,
    body_to_world,
    build_observation,
    front_min_distance,
    histogram_from_ranges,
    load_config,
    observation_dim,
    quat_to_rpy,
    resolve_model_path,
    world_to_body,
)
from policy import NumpyActor


class DrlNavNode:
    """Glue between ROS messages and the shared observation/action interface."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.odom: Optional[Odometry] = None
        self.scan_histogram: Optional[np.ndarray] = None
        self.last_scan_time: Optional[rospy.Time] = None
        self._warned_scan = False

        self.config = self._load_config()
        self.mode = str(self.config["mode"]).lower()
        self.waypoints = np.asarray(
            self.config["goal"]["waypoints"], dtype=np.float64
        )
        self.tolerance = float(self.config["goal"]["tolerance"])
        self.waypoint_index = 0
        self.finished = False

        self.observation_dim = observation_dim(self.config)
        self.actor = self._load_actor()

        self.control_rate = float(self.config["control_rate"])
        self.odom_topic = str(self.config["odom_topic"])
        self.scan_topic = str(self.config["scan_topic"])
        self.cmd_topic = str(self.config["cmd_topic"])
        self.path_topic = str(self.config["path_topic"])
        self.status_topic = str(self.config["status_topic"])
        self.frame_id = str(self.config.get("frame_id", "odom"))
        self.command_frame = str(self.config.get("command_frame", "body")).lower()
        if self.command_frame not in ("body", "world"):
            raise ValueError("command_frame must be 'body' or 'world'")
        self.scan_timeout = float(
            self.config["safety"].get("scan_timeout", 1.0)
        )
        if self.scan_timeout <= 0.0:
            raise ValueError("safety.scan_timeout must be positive")

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1)
        self.waypoint_pub = rospy.Publisher(
            self.path_topic + "_waypoints", Path, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1)

        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self._last_path_publish = rospy.Time(0)
        self._last_status_publish = rospy.Time(0)

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_callback, queue_size=1)
        rospy.on_shutdown(self._stop)

        self._publish_waypoints()
        self._log_startup()
        rospy.loginfo(
            "[drl_nav] mode=%s obs_dim=%d waypoints=%d",
            self.mode,
            self.observation_dim,
            len(self.waypoints),
        )

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def _load_config(self) -> Dict[str, Any]:
        """Load YAML, then apply private ROS parameter overrides."""
        config_file = str(
            rospy.get_param("~config_file", DEFAULT_CONFIG_PATH)
        )
        config = load_config(config_file)

        for key in ("mode", "odom_topic", "scan_topic", "cmd_topic",
                    "path_topic", "status_topic", "command_frame",
                    "control_rate", "model_path", "frame_id"):
            value = rospy.get_param("~" + key, None)
            if value is not None:
                config[key] = value

        goal = rospy.get_param("~goal", None)
        if isinstance(goal, (list, tuple)) and len(goal) == 3:
            config["goal"]["waypoints"] = [[float(v) for v in goal]]
            rospy.loginfo("[drl_nav] override goal: %s", goal)

        waypoints = rospy.get_param("~waypoints", None)
        if isinstance(waypoints, (list, tuple)) and waypoints:
            parsed = []
            for point in waypoints:
                if not isinstance(point, (list, tuple)) or len(point) != 3:
                    raise ValueError("each waypoint must be [x, y, z]")
                parsed.append([float(v) for v in point])
            config["goal"]["waypoints"] = parsed
            rospy.loginfo("[drl_nav] override waypoints: %s", parsed)

        safety_enabled = rospy.get_param("~safety_enabled", None)
        if safety_enabled is not None:
            if isinstance(safety_enabled, str):
                safety_enabled = safety_enabled.strip().lower() in (
                    "1", "true", "yes", "on"
                )
            config["safety"]["enabled"] = bool(safety_enabled)

        scan_timeout = rospy.get_param("~scan_timeout", None)
        if scan_timeout is not None:
            config["safety"]["scan_timeout"] = float(scan_timeout)
        return config

    def _load_actor(self) -> Optional[NumpyActor]:
        """Load the exported policy; fall back to the baseline when allowed."""
        model_path = resolve_model_path(self.config)
        if self.mode == "baseline":
            return None
        if not os.path.isfile(model_path):
            if self.mode == "drl":
                raise FileNotFoundError(
                    "mode=drl but policy weights are missing: %s" % model_path
                )
            rospy.logwarn(
                "[drl_nav] weights not found (%s); falling back to baseline",
                model_path,
            )
            self.mode = "baseline"
            return None
        actor = NumpyActor(model_path)
        if actor.obs_dim != self.observation_dim:
            message = (
                "policy obs_dim=%d does not match config obs_dim=%d"
                % (actor.obs_dim, self.observation_dim)
            )
            if self.mode == "drl":
                raise ValueError(message)
            rospy.logwarn("[drl_nav] %s; falling back to baseline", message)
            self.mode = "baseline"
            return None
        self.mode = "drl"
        rospy.loginfo(
            "[drl_nav] loaded %s policy from %s (%s)",
            actor.algo,
            model_path,
            actor.describe(),
        )
        return actor

    def _log_startup(self) -> None:
        """Print interface information once, so `roslaunch` output is enough."""
        rospy.loginfo("[drl_nav] odom=%s scan=%s cmd=%s",
                      self.odom_topic, self.scan_topic, self.cmd_topic)
        rospy.loginfo("[drl_nav] command frame: %s", self.command_frame)
        rospy.loginfo("[drl_nav] route: %s", self.waypoints.tolist())

    def _publish_waypoints(self) -> None:
        """Publish the route once, latched, for RViz."""
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time.now()
        for waypoint in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.position.z = float(waypoint[2])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.waypoint_pub.publish(path)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_callback(self, message: Odometry) -> None:
        with self.lock:
            self.odom = message

    def _scan_callback(self, message: LaserScan) -> None:
        cfg = self.config
        try:
            histogram = histogram_from_ranges(
                message.ranges,
                message.angle_min,
                message.angle_increment,
                message.range_min,
                message.range_max,
                int(cfg["observation"]["sectors"]),
                float(cfg["observation"]["lidar_range"]),
            )
        except Exception as exc:  # keep the node alive on malformed scans
            rospy.logwarn_throttle(5.0, "[drl_nav] bad LaserScan: %s", exc)
            return
        stamp = message.header.stamp
        with self.lock:
            self.scan_histogram = histogram
            self.last_scan_time = (
                stamp if stamp is not None and stamp.to_sec() > 0.0
                else rospy.Time.now()
            )

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def spin(self) -> None:
        """Run the control loop at ``control_rate``."""
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            self._tick()
            rate.sleep()

    def _tick(self) -> None:
        with self.lock:
            odom = self.odom
            histogram = self.scan_histogram
            scan_time = self.last_scan_time
        if odom is None:
            rospy.logwarn_throttle(3.0, "[drl_nav] waiting for %s", self.odom_topic)
            self._publish_command(np.zeros(4, dtype=np.float64), 0.0)
            return

        pose = odom.pose.pose
        position = np.array([
            pose.position.x, pose.position.y, pose.position.z
        ], dtype=np.float64)
        quaternion = [
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        ]
        rpy = quat_to_rpy(quaternion)
        yaw = float(rpy[2])

        now = rospy.Time.now()
        scan_age = None
        if scan_time is not None:
            scan_age = max(0.0, (now - scan_time).to_sec())
        scan_ready = (
            histogram is not None
            and scan_age is not None
            and scan_age <= self.scan_timeout
        )
        if not scan_ready:
            if histogram is None or scan_age is None:
                rospy.logwarn_throttle(
                    5.0,
                    "[drl_nav] no LaserScan on %s; publishing zero velocity",
                    self.scan_topic,
                )
            else:
                rospy.logwarn_throttle(
                    5.0,
                    "[drl_nav] LaserScan on %s is stale (%.2f s > %.2f s); "
                    "publishing zero velocity",
                    self.scan_topic,
                    scan_age,
                    self.scan_timeout,
                )
            self._warned_scan = True
            self._publish_command(np.zeros(4, dtype=np.float64), yaw)
            return
        if self._warned_scan:
            rospy.loginfo("[drl_nav] fresh LaserScan received; resuming control")
            self._warned_scan = False

        twist = odom.twist.twist
        linear = np.array([
            twist.linear.x, twist.linear.y, twist.linear.z
        ], dtype=np.float64)
        if str(self.config.get("odom_twist_frame", "body")).lower() == "world":
            velocity_body = world_to_body(linear, yaw)
        else:
            velocity_body = linear
        angular_body = np.array([
            twist.angular.x, twist.angular.y, twist.angular.z
        ], dtype=np.float64)

        goal = self.waypoints[min(self.waypoint_index, len(self.waypoints) - 1)]
        distance = float(np.linalg.norm(goal - position))
        if distance <= self.tolerance:
            if self.waypoint_index < len(self.waypoints) - 1:
                rospy.loginfo(
                    "[drl_nav] reached waypoint %d/%d",
                    self.waypoint_index + 1,
                    len(self.waypoints),
                )
                self.waypoint_index += 1
            else:
                if not self.finished:
                    rospy.loginfo("[drl_nav] final waypoint reached; holding")
                self.finished = True

        if self.finished:
            self._publish_command(np.zeros(4, dtype=np.float64), yaw)
            self._publish_status(position, 0.0)
            return

        observation = build_observation(
            position, rpy, velocity_body, angular_body, goal, histogram, self.config
        )
        if self.actor is not None:
            action = self.actor.predict(observation, deterministic=True)
        else:
            action = baseline_action(observation, self.config)
        action = apply_altitude_hold(action, position, goal, self.config)

        safety = self.config["safety"]
        if bool(safety.get("enabled", True)):
            clearance = front_min_distance(histogram, self.config)
            if clearance < float(safety["stop_distance"]):
                action = np.asarray(action, dtype=np.float64).copy()
                action[0:3] = 0.0

        self._publish_command(action_to_velocity(action, self.config), yaw)
        self._publish_path(position, odom.header.frame_id or self.path.header.frame_id)
        self._publish_status(position, distance)

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------

    def _publish_command(self, command: Sequence[float], yaw: float) -> None:
        """Publish a Twist; convert world-frame velocity when requested."""
        linear = np.asarray(command[0:3], dtype=np.float64)
        if self.command_frame == "world":
            linear = body_to_world(linear, yaw)
        twist = Twist()
        twist.linear.x = float(linear[0])
        twist.linear.y = float(linear[1])
        twist.linear.z = float(linear[2])
        twist.angular.z = float(command[3])
        self.cmd_pub.publish(twist)

    def _publish_path(self, position: np.ndarray, frame_id: str) -> None:
        now = rospy.Time.now()
        if self.path.header.frame_id == "odom" and frame_id:
            self.path.header.frame_id = frame_id
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.w = 1.0
        self.path.poses.append(pose)
        if len(self.path.poses) > 2000:
            del self.path.poses[:500]
        if (now - self._last_path_publish).to_sec() >= 0.2:
            self.path.header.stamp = now
            self.path_pub.publish(self.path)
            self._last_path_publish = now

    def _publish_status(self, position: np.ndarray, distance: float) -> None:
        now = rospy.Time.now()
        if (now - self._last_status_publish).to_sec() < 0.5:
            return
        message = String()
        message.data = (
            "mode=%s waypoint=%d/%d distance=%.2f position=(%.2f, %.2f, %.2f)"
            % (self.mode, self.waypoint_index + 1, len(self.waypoints),
               distance, position[0], position[1], position[2])
        )
        self.status_pub.publish(message)
        self._last_status_publish = now

    def _stop(self) -> None:
        """Publish a zero command on shutdown."""
        try:
            self.cmd_pub.publish(Twist())
        except Exception:  # pragma: no cover - shutdown is best effort
            pass


def main() -> None:
    """Initialise ROS and block in the control loop."""
    rospy.init_node("drl_drone_path_planning")
    try:
        node = DrlNavNode()
    except Exception as exc:
        rospy.logfatal("[drl_nav] failed to start: %s", exc)
        raise
    rospy.loginfo("[drl_nav] started")
    node.spin()


if __name__ == "__main__":
    main()
