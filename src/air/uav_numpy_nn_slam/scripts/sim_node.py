#!/usr/bin/env python3
"""UVNN UAV simulation node (MuJoCo physics owner).

Publishes every sensor topic documented in ``docs/air/uav_numpy_nn_slam.md（模块文档的 ROS 话题一节）``:

=============================  =============================  ==========
topic                          type                           rate
=============================  =============================  ==========
/uav/scan                      sensor_msgs/LaserScan          10 Hz
/uav/odom                      nav_msgs/Odometry              20 Hz
/uav/altitude                  std_msgs/Float64               20 Hz
/uav/imu                       sensor_msgs/Imu                20 Hz
/uav/camera/image_raw          sensor_msgs/Image (bgr8)       10 Hz
/uav/ground_truth              geometry_msgs/PoseStamped      20 Hz
/uav/state                     std_msgs/String (JSON)         20 Hz
=============================  =============================  ==========

It subscribes to ``/uav/cmd_vel`` (geometry_msgs/Twist, ``linear.z`` is the
climb rate, positive = up) and optionally to ``/uav/key`` (std_msgs/String,
single key label for the HUD).  The process re-execs itself with the project
virtual environment, which is the only interpreter that can import MuJoCo 3.12
(see ``uav_numpy_nn_slam/runtime.py``).
"""

from __future__ import annotations

import json
import math
import os
import sys
import time


def _repo_root() -> str:
    """Locate the repository root even when started through a catkin relay."""
    env = os.environ.get("UVNN_ROOT", "")
    if env:
        cand = os.path.realpath(env)
        if os.path.isfile(os.path.join(cand, "uav_numpy_nn_slam", "runtime.py")):
            return cand
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.dirname(here), here):
        if os.path.isfile(os.path.join(cand, "uav_numpy_nn_slam", "runtime.py")):
            return cand
    for entry in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if not entry:
            continue
        pkg = os.path.join(entry, "uav_numpy_nn_slam")
        if os.path.isfile(os.path.join(pkg, "package.xml")):
            return os.path.dirname(pkg)
    return os.path.dirname(here)


ROOT = _repo_root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from uav_numpy_nn_slam.runtime import ensure_venv

ensure_venv(__file__)

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Float64, String

from uav_numpy_nn_slam import config as C
from uav_numpy_nn_slam.geometry import wrap_angle
from uav_numpy_nn_slam.sim import UAVSim

# --------------------------------------------------------------------- config
BEAMS = int(getattr(C, "LIDAR_BEAMS", 72))
RANGE_MIN = float(getattr(C, "LIDAR_RANGE_MIN", 0.5))
RANGE_MAX = float(getattr(C, "LIDAR_RANGE_MAX", 60.0))
MAX_VX = float(getattr(C, "MAX_VX", 8.0))
MAX_VY = float(getattr(C, "MAX_VY", 5.0))
MAX_VZ = float(getattr(C, "MAX_VZ", 3.0))
MAX_YAW_RATE = float(getattr(C, "MAX_YAW_RATE", 1.2))


def _yaw_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0,
                      z=math.sin(0.5 * float(yaw)),
                      w=math.cos(0.5 * float(yaw)))


def _diag(n: int, values) -> list:
    """Row-major diagonal covariance matrix as a plain python list."""
    out = [0.0] * (n * n)
    for i, v in enumerate(values):
        out[i * n + i] = float(v)
    return out


def _yaw_from_quaternion(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimNode:
    """Owns the physics loop and publishes the ROS sensor interface."""

    def __init__(self) -> None:
        self.scene_name = str(rospy.get_param("~scene", "city"))
        self.seed = int(rospy.get_param("~seed", C.DEFAULT_SEED))
        self.noise = bool(rospy.get_param("~noise", True))
        self.duration = float(rospy.get_param("~duration", 0.0))
        self.mode = str(rospy.get_param("~mode", "TELEOP"))
        self.state_hz = max(1.0, float(rospy.get_param("~state_hz", 20.0)))
        self.sensor_hz = max(1.0, float(rospy.get_param("~sensor_hz",
                                                         C.SENSOR_HZ)))
        self.cmd_timeout = max(0.05, float(rospy.get_param("~cmd_timeout", 0.5)))

        try:
            self.sim = UAVSim(self.scene_name, seed=self.seed, noise=self.noise)
        except Exception as exc:  # pragma: no cover - environment dependent
            rospy.logfatal("cannot create UAVSim for scene %r: %s",
                           self.scene_name, exc)
            raise SystemExit(1)

        self.beam_angles = np.asarray(self.sim.lidar_beam_angles(),
                                      dtype=np.float64).ravel()
        self._cmd = np.zeros(4)
        self._cmd_stamp = -1e9
        self._last_cmd = np.zeros(4)
        self._key = "-"
        self._scan = np.full(BEAMS, RANGE_MAX, dtype=np.float64)
        self._odom = np.zeros(4)
        self._odom_prev = None
        self._altitude = 0.0
        self._pose_prev = None
        self._yaw = 0.0
        self._yaw_rate = 0.0
        self._speed = 0.0
        self._v_body = np.zeros(3)
        self._summary_done = False
        # fractional-time accumulators keep the publish rates exact even when
        # the 20 Hz loop is not an integer multiple of the lower sensor rate
        self._sensor_accum = 0.0
        self._state_accum = 0.0

        # ---------------------------------------------------------- publishers
        self.prefix = str(rospy.get_param("~topic_prefix", "/uav"))
        prefix = self.prefix
        self.pub_scan = rospy.Publisher(prefix + "/scan", LaserScan,
                                        queue_size=5)
        self.pub_odom = rospy.Publisher(prefix + "/odom", Odometry,
                                        queue_size=10)
        self.pub_altitude = rospy.Publisher(prefix + "/altitude", Float64,
                                            queue_size=10)
        self.pub_imu = rospy.Publisher(prefix + "/imu", Imu, queue_size=10)
        self.pub_camera = rospy.Publisher(prefix + "/camera/image_raw", Image,
                                          queue_size=2)
        self.pub_truth = rospy.Publisher(prefix + "/ground_truth",
                                         PoseStamped, queue_size=10)
        self.pub_state = rospy.Publisher(prefix + "/state", String,
                                         queue_size=5)
        # ------------------------------------------------------- subscriptions
        rospy.Subscriber(prefix + "/cmd_vel", Twist, self._on_cmd, queue_size=5)
        rospy.Subscriber(prefix + "/key", String, self._on_key, queue_size=5)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("sim_node: scene=%s seed=%d noise=%s duration=%.1f mode=%s",
                      self.scene_name, self.seed, self.noise, self.duration,
                      self.mode)
        rospy.loginfo("sim_node: %d lidar beams, range %.1f..%.1f m, "
                      "altitude %.1f m",
                      BEAMS, RANGE_MIN, RANGE_MAX, float(self.sim.pose[2]))

    # ------------------------------------------------------------- callbacks
    def _on_cmd(self, msg) -> None:
        self._cmd = np.array([float(msg.linear.x), float(msg.linear.y),
                              float(msg.linear.z), float(msg.angular.z)])
        self._cmd_stamp = time.time()

    def _on_key(self, msg: String) -> None:
        text = str(msg.data).strip()
        self._key = text if text else "-"

    def _current_cmd(self) -> np.ndarray:
        if time.time() - self._cmd_stamp > self.cmd_timeout:
            return np.zeros(4)
        return self._cmd.copy()

    # ------------------------------------------------------------- publishing
    def _publish_odom(self, stamp) -> None:
        od = self._odom
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.child_frame_id = "uav/base_link"
        msg.pose.pose.position = Point(x=float(od[0]), y=float(od[1]),
                                       z=float(od[2]))
        msg.pose.pose.orientation = _yaw_quaternion(float(od[3]))
        msg.pose.covariance = _diag(6, [4e-2, 4e-2, 1e-2, 1e-3, 1e-3, 1e-2])
        msg.twist.twist.linear = Vector3(x=float(self._v_body[0]),
                                         y=float(self._v_body[1]),
                                         z=float(self._v_body[2]))
        msg.twist.twist.angular = Vector3(x=0.0, y=0.0,
                                          z=float(self._yaw_rate))
        msg.twist.covariance = _diag(6, [2e-2, 2e-2, 2e-2, 1e-3, 1e-3, 1e-3])
        self.pub_odom.publish(msg)

    def _publish_imu(self, stamp) -> None:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "uav/base_link"
        msg.orientation = _yaw_quaternion(float(self._yaw))
        msg.orientation_covariance = _diag(3, [1e-2, 1e-2, 2e-2])
        msg.angular_velocity = Vector3(x=0.0, y=0.0,
                                       z=float(self._yaw_rate))
        msg.angular_velocity_covariance = _diag(3, [1e-4, 1e-4, 1e-4])
        msg.linear_acceleration_covariance = _diag(3, [1e-2, 1e-2, 1e-2])
        self.pub_imu.publish(msg)

    def _publish_scan(self, stamp) -> None:
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = "uav/base_link"
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2.0 * math.pi / max(1, BEAMS)
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.sensor_hz
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = [float(v) for v in self._scan]
        msg.intensities = []
        self.pub_scan.publish(msg)

    def _publish_camera(self, stamp) -> None:
        image = self.sim.camera()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "uav/camera"
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(image.shape[1] * 3)
        msg.data = np.ascontiguousarray(image, dtype=np.uint8).tobytes()
        self.pub_camera.publish(msg)

    def _publish_truth(self, stamp) -> None:
        pose = self.sim.pose
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.pose.position = Point(x=float(pose[0]), y=float(pose[1]),
                                  z=float(pose[2]))
        msg.pose.orientation = _yaw_quaternion(float(pose[3]))
        self.pub_truth.publish(msg)

    def _publish_state(self) -> None:
        pose = self.sim.pose
        payload = {
            "time": round(float(self.sim.time), 3),
            "truth": [round(float(v), 4) for v in pose],
            "odom": [round(float(v), 4) for v in self._odom],
            "cmd": [round(float(v), 4) for v in self._last_cmd],
            "scan": [round(float(v), 3) for v in self._scan],
            "collisions": int(self.sim.collisions),
            "nfz_violations": int(self.sim.nfz_violations),
            "nfz_clearance": round(float(self.sim.nfz_clearance()), 3),
            "scene": self.scene_name,
            "key": self._key,
            "mode": self.mode,
            "speed": round(float(self._speed), 3),
            "altitude": round(float(pose[2]), 3),
        }
        self.pub_state.publish(String(data=json.dumps(payload,
                                                      separators=(",", ":"))))

    # -------------------------------------------------------------- main loop
    def _world_cmd(self, cmd: np.ndarray, yaw: float) -> np.ndarray:
        """Rotate a body-frame ``(forward, strafe)`` command into world frame.

        ``/uav/cmd_vel`` uses the intuitive body frame (``linear.x`` = nose
        direction, ``linear.y`` = right), while :meth:`UAVSim.step` tracks a
        world-frame velocity.  The rotated command is clamped in the world
        frame so the downstream per-axis limits stay meaningful.
        """
        c, s = math.cos(yaw), math.sin(yaw)
        wx = c * float(cmd[0]) - s * float(cmd[1])
        wy = s * float(cmd[0]) + c * float(cmd[1])
        wx = float(np.clip(wx, -MAX_VX, MAX_VX))
        wy = float(np.clip(wy, -MAX_VY, MAX_VY))
        return np.array([wx, wy, float(cmd[2]), float(cmd[3])],
                        dtype=np.float64)

    def _body_velocity(self, pose, dt: float) -> np.ndarray:
        """Body-frame velocity of the vehicle (m/s)."""
        try:
            v_world = np.asarray(self.sim.velocity(), dtype=np.float64).ravel()
        except Exception:
            if self._pose_prev is None or dt <= 0.0:
                return np.zeros(3)
            v_world = np.array([(float(pose[0]) - self._pose_prev[0]) / dt,
                                (float(pose[1]) - self._pose_prev[1]) / dt,
                                (float(pose[2]) - self._pose_prev[2]) / dt],
                               dtype=np.float64)
        yaw = float(pose[3])
        c, s = math.cos(yaw), math.sin(yaw)
        if v_world.size < 3:
            v_world = np.pad(v_world, (0, 3 - v_world.size))
        return np.array([c * v_world[0] + s * v_world[1],
                         -s * v_world[0] + c * v_world[1],
                         v_world[2]], dtype=np.float64)

    def spin(self) -> None:
        rate = rospy.Rate(C.CONTROL_HZ)
        dt = 1.0 / float(C.CONTROL_HZ)
        while not rospy.is_shutdown():
            cmd = self._current_cmd()
            self._last_cmd = cmd.copy()
            yaw = float(self.sim.pose[3])
            self.sim.step(self._world_cmd(cmd, yaw), dt)
            self._odom = np.asarray(self.sim.integrate_odometry(dt),
                                    dtype=np.float64)
            pose = self.sim.pose
            self._altitude = float(self.sim.altitude())
            self._yaw, self._yaw_rate = self.sim.imu()
            self._v_body = self._body_velocity(pose, dt)
            self._speed = float(np.linalg.norm(self._v_body))
            self._pose_prev = np.array([float(pose[0]), float(pose[1]),
                                        float(pose[2]), float(pose[3])])

            stamp = rospy.Time.now()
            self._publish_odom(stamp)
            self._publish_imu(stamp)
            self.pub_altitude.publish(Float64(data=self._altitude))
            self._publish_truth(stamp)

            self._sensor_accum += dt
            if self._sensor_accum >= 1.0 / self.sensor_hz:
                self._sensor_accum -= 1.0 / self.sensor_hz
                self._scan = np.asarray(self.sim.lidar(), dtype=np.float64)
                self._publish_scan(stamp)
                self._publish_camera(stamp)

            self._state_accum += dt
            if self._state_accum >= 1.0 / self.state_hz:
                self._state_accum -= 1.0 / self.state_hz
                self._publish_state()

            if self.duration > 0.0 and self.sim.time >= self.duration:
                rospy.loginfo("sim_node: duration %.2f s reached, shutting down",
                              self.duration)
                self._print_summary()
                self.pub_state.publish(String(data=json.dumps(
                    {"time": round(float(self.sim.time), 3),
                     "scene": self.scene_name,
                     "collisions": int(self.sim.collisions),
                     "nfz_violations": int(self.sim.nfz_violations),
                     "mode": self.mode, "status": "FINISHED"},
                    separators=(",", ":"))))
                rospy.signal_shutdown("duration reached")
                break

            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    # ---------------------------------------------------------------- summary
    def _print_summary(self) -> None:
        if self._summary_done:
            return
        self._summary_done = True
        pose = self.sim.pose
        payload = {
            "event": "sim_summary",
            "scene": self.scene_name,
            "mode": self.mode,
            "time": round(float(self.sim.time), 3),
            "collisions": int(self.sim.collisions),
            "nfz_violations": int(self.sim.nfz_violations),
            "nfz_clearance": round(float(self.sim.nfz_clearance()), 3),
            "final_pose": [round(float(v), 4) for v in pose],
            "odom": [round(float(v), 4) for v in self._odom],
        }
        line = json.dumps(payload, separators=(",", ":"))
        print(line)
        sys.stdout.flush()

    def _on_shutdown(self) -> None:
        self._print_summary()


def main() -> int:
    rospy.init_node("sim_node", anonymous=False)
    node = SimNode()
    try:
        node.spin()
    finally:
        node._print_summary()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
