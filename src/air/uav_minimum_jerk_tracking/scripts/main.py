#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OpenHUTB 无人机最小加加速度轨迹生成与跟踪节点."""

from __future__ import annotations

import csv
import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from controller import (  # noqa: E402
    body_frame_is_safe,
    body_offsets_to_world,
    tracking_velocity,
    yaw_from_quaternion,
)
from trajectory import MinimumJerkTrajectory  # noqa: E402


def _pose_position(odom):
    pose = odom.pose.pose.position
    return np.array([pose.x, pose.y, pose.z], dtype=np.float64)


def _pose_velocity(odom):
    twist = odom.twist.twist.linear
    return np.array([twist.x, twist.y, twist.z], dtype=np.float64)


class MinimumJerkTrackingNode:
    def __init__(self):
        body_frame = rospy.get_param("control/body_frame", None)
        self.body_frame_safe = body_frame_is_safe(body_frame)
        if not self.body_frame_safe:
            rospy.logerr(
                "control/body_frame 必须显式为 false；节点将拒绝飞行"
            )

        self.mission = str(rospy.get_param("mission", "smooth_forward_6m"))
        self.mode = str(rospy.get_param("trajectory/mode", "minimum_jerk"))
        self.waypoint_offsets = [
            tuple(float(value) for value in point)
            for point in rospy.get_param("missions/" + self.mission)
        ]
        self.control_params = rospy.get_param("control", {})
        self.trajectory_params = rospy.get_param("trajectory", {})
        self.rate_hz = float(self.control_params.get("rate_hz", 20.0))
        self.start_delay_sec = float(
            self.control_params.get("start_delay_sec", 2.0)
        )
        self.odom_timeout_sec = float(
            self.control_params.get("odom_timeout_sec", 0.5)
        )

        self.topic_odom = rospy.get_param("topics/odom", "/uav/odom")
        self.topic_cmd = rospy.get_param("topics/cmd_vel", "/uav/cmd_vel")
        self.topic_status = rospy.get_param(
            "topics/status", "/uav_minimum_jerk_tracking/status"
        )
        self.topic_desired = rospy.get_param(
            "topics/desired_path", "/uav_minimum_jerk_tracking/desired_path"
        )
        self.topic_actual = rospy.get_param(
            "topics/actual_path", "/uav_minimum_jerk_tracking/actual_path"
        )

        self.latest_odom = None
        self.start_position = None
        self.trajectory = None
        self.first_odom_at = None
        self.last_odom_at = None
        self.trajectory_elapsed = 0.0
        self.settled_since = None
        self.finished = False
        self.last_step_at = None
        self.csv_handle = None
        self.csv_writer = None

        self.cmd_pub = rospy.Publisher(self.topic_cmd, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(self.topic_status, String, queue_size=1)
        self.desired_pub = rospy.Publisher(
            self.topic_desired, Path, queue_size=1, latch=True
        )
        self.actual_pub = rospy.Publisher(self.topic_actual, Path, queue_size=1)
        rospy.Subscriber(
            self.topic_odom, Odometry, self._odom_callback, queue_size=1
        )
        rospy.on_shutdown(self._shutdown)

        self.actual_path = Path()
        self.actual_path.header.frame_id = "world"
        rospy.loginfo(
            "minimum-jerk tracking: mission=%s mode=%s waypoints=%s",
            self.mission,
            self.mode,
            self.waypoint_offsets,
        )

    def _odom_callback(self, msg):
        self.latest_odom = msg
        self.last_odom_at = rospy.get_time()

    def _status(self, text, force=False):
        self.status_pub.publish(String(data=text))
        if force:
            rospy.loginfo("%s", text)
        else:
            rospy.loginfo_throttle(1.0, "%s", text)

    def _initialize(self):
        position = _pose_position(self.latest_odom)
        yaw = yaw_from_quaternion(self.latest_odom.pose.pose.orientation)
        waypoints = body_offsets_to_world(
            position, yaw, self.waypoint_offsets
        )
        self.start_position = position
        self.trajectory = MinimumJerkTrajectory(
            position,
            waypoints,
            self.trajectory_params,
            self.mode,
        )
        self.trajectory_elapsed = 0.0
        self.last_step_at = rospy.get_time()
        self._publish_desired_path()
        self._open_csv()
        rospy.loginfo(
            "轨迹锁定: start=%s duration=%.2f s path=%s",
            position.tolist(),
            self.trajectory.total_duration,
            [point.tolist() for point in waypoints],
        )

    def _publish_desired_path(self):
        path = Path()
        path.header.frame_id = "world"
        path.header.stamp = rospy.Time.now()
        for sample in self.trajectory.sampled_path(0.1):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(sample.position[0])
            pose.pose.position.y = float(sample.position[1])
            pose.pose.position.z = float(sample.position[2])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.desired_pub.publish(path)

    def _metrics_path(self):
        directory = os.path.expanduser(
            str(self.control_params.get(
                "metrics_directory", "~/.ros/uav_minimum_jerk_tracking"
            ))
        )
        os.makedirs(directory, exist_ok=True)
        return os.path.join(
            directory,
            "%s_%s.csv" % (self.mission, self.mode),
        )

    def _open_csv(self):
        self.csv_handle = open(self._metrics_path(), "w", newline="")
        self.csv_writer = csv.writer(self.csv_handle)
        self.csv_writer.writerow(
            [
                "t",
                "segment",
                "x_ref",
                "y_ref",
                "z_ref",
                "x",
                "y",
                "z",
                "error",
                "vx_ref",
                "vy_ref",
                "vz_ref",
                "ax_ref",
                "ay_ref",
                "az_ref",
                "jx_ref",
                "jy_ref",
                "jz_ref",
                "cmd_vx",
                "cmd_vy",
                "cmd_vz",
            ]
        )

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _step_once(self):
        if not self.body_frame_safe:
            self._publish_stop()
            self._status("REJECTED body_frame_not_false")
            return
        if self.latest_odom is None:
            self._publish_stop()
            self._status("WAIT_ODOM")
            return
        now = rospy.get_time()
        if (
            self.last_odom_at is None
            or now - self.last_odom_at > self.odom_timeout_sec
        ):
            self.last_step_at = now
            self._publish_stop()
            self._status("STALE_ODOM")
            return
        if self.first_odom_at is None:
            self.first_odom_at = now
        if (
            self.trajectory is None
            and now - self.first_odom_at < self.start_delay_sec
        ):
            self._publish_stop()
            self._status("WAIT_TAKEOFF")
            return
        if self.trajectory is None:
            self._initialize()
        if self.finished:
            self._publish_stop()
            return

        dt = max(now - float(self.last_step_at or now), 1e-3)
        self.last_step_at = now
        self.trajectory_elapsed += min(dt, 0.2)
        elapsed = self.trajectory_elapsed
        reference = self.trajectory.sample(elapsed)
        position = _pose_position(self.latest_odom)
        velocity = _pose_velocity(self.latest_odom)
        command = tracking_velocity(
            position,
            velocity,
            reference,
            self.control_params,
            dt,
        )

        msg = Twist()
        msg.linear.x = command.vx
        msg.linear.y = command.vy
        msg.linear.z = command.vz
        self.cmd_pub.publish(msg)
        self._record(elapsed, reference, position, command)
        self._publish_actual(position)
        self._status(
            "RUN mode=%s seg=%d t=%.2f/%.2f err=%.3f "
            "ref_v=%.3f ref_a=%.3f ref_j=%.3f"
            % (
                self.mode,
                reference.segment_index,
                reference.elapsed,
                reference.total_duration,
                command.position_error,
                float(np.linalg.norm(reference.velocity)),
                float(np.linalg.norm(reference.acceleration)),
                float(np.linalg.norm(reference.jerk)),
            )
        )

        tolerance = float(self.control_params.get("goal_tolerance", 0.25))
        settle_time = float(self.control_params.get("settle_time", 0.8))
        final_wait_timeout = float(
            self.control_params.get("final_wait_timeout", 18.0)
        )
        if (
            elapsed >= reference.total_duration + final_wait_timeout
            and command.position_error > tolerance
        ):
            self.finished = True
            self._publish_stop()
            self._status(
                "FAILED_STALLED mission=%s mode=%s error=%.3f "
                "elapsed=%.2f ref_duration=%.2f timeout=%.1f"
                % (
                    self.mission,
                    self.mode,
                    command.position_error,
                    elapsed,
                    reference.total_duration,
                    final_wait_timeout,
                ),
                force=True,
            )
            return
        if elapsed >= reference.total_duration and command.position_error <= tolerance:
            if self.settled_since is None:
                self.settled_since = now
            if now - self.settled_since >= settle_time:
                self.finished = True
                self._publish_stop()
                self._status(
                    "DONE mission=%s mode=%s error=%.3f "
                    "elapsed=%.2f ref_duration=%.2f"
                    % (
                        self.mission,
                        self.mode,
                        command.position_error,
                        elapsed,
                        reference.total_duration,
                    ),
                    force=True,
                )
        else:
            self.settled_since = None

    def _record(self, elapsed, reference, position, command):
        if self.csv_writer is None:
            return
        self.csv_writer.writerow(
            [
                elapsed,
                reference.segment_index,
                reference.position[0],
                reference.position[1],
                reference.position[2],
                position[0],
                position[1],
                position[2],
                command.position_error,
                reference.velocity[0],
                reference.velocity[1],
                reference.velocity[2],
                reference.acceleration[0],
                reference.acceleration[1],
                reference.acceleration[2],
                reference.jerk[0],
                reference.jerk[1],
                reference.jerk[2],
                command.vx,
                command.vy,
                command.vz,
            ]
        )
        self.csv_handle.flush()

    def _publish_actual(self, position):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "world"
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.w = 1.0
        self.actual_path.header = pose.header
        self.actual_path.poses.append(pose)
        self.actual_pub.publish(self.actual_path)

    def _shutdown(self):
        self._publish_stop()
        if self.csv_handle is not None:
            self.csv_handle.close()
            self.csv_handle = None

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                self._step_once()
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn_throttle(2.0, "轨迹控制异常: %s", exc)
                self._publish_stop()
                self._status("ERROR %s" % exc)
            rate.sleep()


def main():
    rospy.init_node("uav_minimum_jerk_tracking", anonymous=False)
    MinimumJerkTrackingNode().run()


if __name__ == "__main__":
    main()
