#!/usr/bin/env python3
"""AirSim adapter for the SafeDRL navigator.

The navigator speaks the simulator-neutral ROS interface below:

    /drone/odom       nav_msgs/Odometry        ENU world, FLU body
    /drone/scan       sensor_msgs/LaserScan    2-D horizontal ranges
    /drone/cmd_vel    geometry_msgs/Twist      (vx, vy, vz, yaw_rate), ENU/FLU
    /drone/nav_status std_msgs/String          JSON diagnostics

AirSim works in NED / FRD, so this node converts frames in both directions.
Two backends are supported:

``backend:=ros``
    ``airsim_ros_pkgs`` already runs and exposes
    ``/airsim_node/<vehicle>/odom_local_ned`` and
    ``/airsim_node/<vehicle>/vel_cmd_world_frame``.  The node remaps those to
    the standard topics and performs the frame conversion.  An optional
    ``sensor_msgs/PointCloud2`` lidar stream can be flattened into a LaserScan.

``backend:=api``
    The node talks to the AirSim Python client directly
    (``airsim.MultirotorClient``), which is useful when the ROS wrapper is not
    built.  Odometry is read from ``getMultirotorState`` and velocity commands
    are sent with ``moveByVelocityAsync``.

Run it together with the navigator through ``airsim_navigate.launch``.
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from sim_frames import (  # noqa: E402
    airsim_orientation_to_rpy,
    enu_to_ned,
    ned_to_enu,
    rpy_to_quat_xyzw,
)

try:
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, PointCloud2
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:  # pragma: no cover - keeps the frame helpers testable
    ROS_AVAILABLE = False

try:
    from airsim_ros_pkgs.msg import VelCmd
    AIRSIM_ROS_MSGS = True
except Exception:  # pragma: no cover - package only exists on the ROS side
    VelCmd = None
    AIRSIM_ROS_MSGS = False

try:
    import airsim as airsim_client
    AIRSIM_API = True
except Exception:  # pragma: no cover - optional dependency
    airsim_client = None
    AIRSIM_API = False


# --------------------------------------------------------------------------
# Pure helpers (unit-tested without ROS / AirSim)
# --------------------------------------------------------------------------


def pointcloud_to_scan(points, n_bins=180, range_min=0.10, range_max=20.0):
    """Flatten an ``(N, 3)`` point cloud in FLU coordinates into 2-D ranges.

    Points are binned by bearing in ``[-pi, pi)``; each bin keeps the closest
    return.  Bins without a return are set to ``range_max``.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    ranges = np.full(int(n_bins), float(range_max), dtype=float)
    if pts.size == 0:
        return ranges
    r = np.hypot(pts[:, 0], pts[:, 1])
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    if not np.any(valid):
        return ranges
    bearing = np.arctan2(pts[valid, 1], pts[valid, 0])
    idx = np.floor((bearing + math.pi) / (2.0 * math.pi) * n_bins).astype(int)
    idx = np.clip(idx, 0, int(n_bins) - 1)
    np.minimum.at(ranges, idx, r[valid])
    return ranges


def twist_to_airsim_velocity(twist_linear, yaw_rate_enu):
    """ENU/FLU ``(vx, vy, vz)`` + CCW yaw rate -> AirSim NED velocity command."""
    ned = enu_to_ned(np.asarray(twist_linear, dtype=float))
    return float(ned[0]), float(ned[1]), float(ned[2]), -float(yaw_rate_enu)


def ros_odom_from_airsim(position_ned, velocity_ned, w, x, y, z):
    """Build the fields of a standard ENU/FLU Odometry message."""
    position = ned_to_enu(position_ned)
    velocity = ned_to_enu(velocity_ned)
    roll, pitch, yaw = airsim_orientation_to_rpy(w, x, y, z)
    qx, qy, qz, qw = rpy_to_quat_xyzw(roll, pitch, yaw)
    return position, velocity, (qx, qy, qz, qw), (roll, pitch, yaw)


# --------------------------------------------------------------------------
# ROS backend
# --------------------------------------------------------------------------


class AirsimRosBridge:
    """Bridge between ``airsim_ros_pkgs`` topics and the standard interface."""

    def __init__(self):
        if not ROS_AVAILABLE:
            raise RuntimeError("rospy is not available")
        if not AIRSIM_ROS_MSGS:
            raise RuntimeError(
                "airsim_ros_pkgs is not on the ROS package path; build it first "
                "or use backend:=api"
            )

        vehicle = rospy.get_param("~vehicle_name", "drone_1")
        self.odom_in_topic = rospy.get_param(
            "~odom_in_topic", "/airsim_node/%s/odom_local_ned" % vehicle
        )
        self.odom_out_topic = rospy.get_param("~odom_out_topic", "/drone/odom")
        self.cmd_in_topic = rospy.get_param("~cmd_in_topic", "/drone/cmd_vel")
        self.cmd_out_topic = rospy.get_param(
            "~cmd_out_topic", "/airsim_node/%s/vel_cmd_world_frame" % vehicle
        )
        self.lidar_topic = rospy.get_param("~lidar_topic", "")
        self.scan_topic = rospy.get_param("~scan_topic", "/drone/scan")
        self.scan_bins = int(rospy.get_param("~scan_bins", 180))
        self.scan_range = float(rospy.get_param("~scan_range", 20.0))
        # airsim_ros_pkgs publishes the lidar cloud in the sensor FRD frame.
        self.lidar_frd = bool(rospy.get_param("~lidar_frd", True))

        self.pub_odom = rospy.Publisher(self.odom_out_topic, Odometry, queue_size=1)
        self.pub_scan = rospy.Publisher(self.scan_topic, LaserScan, queue_size=1)
        self.pub_cmd = rospy.Publisher(self.cmd_out_topic, VelCmd, queue_size=1)
        rospy.Subscriber(self.odom_in_topic, Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber(self.cmd_in_topic, Twist, self._on_cmd, queue_size=1)
        if self.lidar_topic:
            rospy.Subscriber(
                self.lidar_topic, PointCloud2, self._on_lidar, queue_size=1
            )

        rospy.loginfo(
            "AirSim ROS bridge: odom %s -> %s, cmd %s -> %s",
            self.odom_in_topic,
            self.odom_out_topic,
            self.cmd_in_topic,
            self.cmd_out_topic,
        )

    def _on_odom(self, msg: "Odometry"):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        position, velocity, quat, _ = ros_odom_from_airsim(
            [p.x, p.y, p.z], [v.x, v.y, v.z], q.w, q.x, q.y, q.z
        )

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "world"
        out.child_frame_id = "base_link"
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = position
        out.pose.pose.orientation.x = quat[0]
        out.pose.pose.orientation.y = quat[1]
        out.pose.pose.orientation.z = quat[2]
        out.pose.pose.orientation.w = quat[3]
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = velocity
        out.twist.twist.angular.z = -float(msg.twist.twist.angular.z)
        self.pub_odom.publish(out)

    def _on_cmd(self, msg: "Twist"):
        vx, vy, vz, yaw_rate = twist_to_airsim_velocity(
            [msg.linear.x, msg.linear.y, msg.linear.z], msg.angular.z
        )
        cmd = VelCmd()
        cmd.x, cmd.y, cmd.z = vx, vy, vz
        if hasattr(cmd, "yaw_rate"):
            cmd.yaw_rate = yaw_rate
        elif hasattr(cmd, "yaw"):
            cmd.yaw = yaw_rate  # legacy message with absolute yaw only
        self.pub_cmd.publish(cmd)

    def _on_lidar(self, msg: "PointCloud2"):
        try:
            from sensor_msgs import point_cloud2

            points = list(
                point_cloud2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True
                )
            )
        except Exception as exc:  # pragma: no cover - malformed cloud
            rospy.logwarn_throttle(5.0, "cannot decode PointCloud2: %s", exc)
            return

        if self.lidar_frd:  # FRD -> FLU before the horizontal projection
            points = [(p[0], -p[1], -p[2]) for p in points]
        ranges = pointcloud_to_scan(
            points, self.scan_bins, range_max=self.scan_range
        )
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / self.scan_bins
        scan.range_min = 0.10
        scan.range_max = self.scan_range
        scan.ranges = ranges.tolist()
        self.pub_scan.publish(scan)

    def spin(self):
        rospy.spin()


# --------------------------------------------------------------------------
# Direct AirSim Python API backend
# --------------------------------------------------------------------------


class AirsimApiBridge:
    """Bridge using ``airsim.MultirotorClient`` directly (no ROS wrapper)."""

    def __init__(self):
        if not ROS_AVAILABLE:
            raise RuntimeError("rospy is not available")
        if not AIRSIM_API:
            raise RuntimeError(
                "the airsim Python package is not installed; "
                "run `pip install airsim==1.8.1`"
            )

        ip = rospy.get_param("~airsim_ip", "")
        port = int(rospy.get_param("~airsim_port", 41451))
        self.vehicle = rospy.get_param("~vehicle_name", "drone_1")
        self.control_rate = float(rospy.get_param("~control_rate", 30.0))
        self.auto_arm = bool(rospy.get_param("~auto_arm", True))
        self.odom_topic = rospy.get_param("~odom_out_topic", "/drone/odom")
        self.cmd_topic = rospy.get_param("~cmd_in_topic", "/drone/cmd_vel")
        self.status_topic = rospy.get_param("~status_topic", "/drone/nav_status")

        kwargs = {"timeout_value": 10.0}
        if ip:
            kwargs["ip"] = ip
            kwargs["port"] = port
        self.client = airsim_client.MultirotorClient(**kwargs)
        self.client.confirmConnection()
        self.client.enableApiControl(True, self.vehicle)
        if self.auto_arm:
            self.client.armDisarm(True, self.vehicle)

        self._lock = threading.Lock()
        self._cmd = np.zeros(4)
        self._last_cmd_time = 0.0
        self._stop = threading.Event()

        self.pub_odom = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)
        self.pub_status = rospy.Publisher(self.status_topic, String, queue_size=1)
        rospy.Subscriber(self.cmd_topic, Twist, self._on_cmd, queue_size=1)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo("AirSim API bridge connected for vehicle %s", self.vehicle)

    def _on_cmd(self, msg: "Twist"):
        with self._lock:
            self._cmd = np.array(
                [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z],
                dtype=float,
            )
            self._last_cmd_time = rospy.get_time()

    def _command(self):
        with self._lock:
            cmd = self._cmd.copy()
            stale = rospy.get_time() - self._last_cmd_time > 0.5
        if stale:  # navigator died -> stop instead of flying on forever
            cmd[:] = 0.0
        vx, vy, vz, yaw_rate = twist_to_airsim_velocity(cmd[:3], cmd[3])
        yaw_mode = airsim_client.YawMode(is_rate=True, yaw_or_rate=math.degrees(yaw_rate))
        self.client.moveByVelocityAsync(
            vx,
            vy,
            vz,
            duration=1.5 / self.control_rate,
            drivetrain=airsim_client.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=yaw_mode,
            vehicle_name=self.vehicle,
        )

    def _publish_odom(self):
        state = self.client.getMultirotorState(vehicle_name=self.vehicle)
        k = state.kinematics_estimated
        q = k.orientation
        position, velocity, quat, _ = ros_odom_from_airsim(
            [k.position.x_val, k.position.y_val, k.position.z_val],
            [k.linear_velocity.x_val, k.linear_velocity.y_val, k.linear_velocity.z_val],
            q.w_val,
            q.x_val,
            q.y_val,
            q.z_val,
        )
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = position
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = quat[0], quat[1]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = quat[2], quat[3]
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = velocity
        self.pub_odom.publish(msg)

        status = json.dumps(
            {
                "t": round(rospy.get_time(), 3),
                "position": [round(float(v), 3) for v in position],
                "velocity": [round(float(v), 3) for v in velocity],
                "frame": "ENU/FLU",
            }
        )
        self.pub_status.publish(String(data=status))

    def _shutdown(self):
        self._stop.set()
        try:
            self.client.moveByVelocityAsync(0.0, 0.0, 0.0, 1.0, vehicle_name=self.vehicle)
            self.client.armDisarm(False, self.vehicle)
            self.client.enableApiControl(False, self.vehicle)
        except Exception:  # pragma: no cover - best effort on shutdown
            pass

    def spin(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown() and not self._stop.is_set():
            self._command()
            self._publish_odom()
            rate.sleep()


def main():
    if not ROS_AVAILABLE:
        print(
            "ERROR: rospy not found. Source your ROS workspace first:\n"
            "  source /opt/ros/noetic/setup.bash",
            file=sys.stderr,
        )
        return 1
    rospy.init_node("airsim_bridge", anonymous=False)
    backend = rospy.get_param("~backend", "ros")
    if backend not in ("ros", "api"):
        rospy.logfatal("~backend must be 'ros' or 'api', got %r", backend)
        return 2
    try:
        bridge = AirsimRosBridge() if backend == "ros" else AirsimApiBridge()
    except Exception as exc:  # noqa: BLE001
        rospy.logfatal("AirSim bridge failed to start: %s", exc)
        return 2
    try:
        bridge.spin()
    except rospy.ROSInterruptException:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
