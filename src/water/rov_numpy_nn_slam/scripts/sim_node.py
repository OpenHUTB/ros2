#!/usr/bin/env python3
"""RVNN simulation node (MuJoCo physics owner).

Publishes every sensor topic documented in ``docs/ros_api.md``:

=============================  =============================  ==========
topic                          type                           rate
=============================  =============================  ==========
/rov/scan                      sensor_msgs/LaserScan          10 Hz
/rov/odom                      nav_msgs/Odometry              50 Hz
/rov/depth                     std_msgs/Float64               50 Hz
/rov/imu                       sensor_msgs/Imu                50 Hz
/rov/camera/image_raw          sensor_msgs/Image (bgr8)       10 Hz
/rov/ground_truth              geometry_msgs/PoseStamped      50 Hz
/rov/state                     std_msgs/String (JSON)         20 Hz
=============================  =============================  ==========

It subscribes to ``/rov/cmd_vel`` (geometry_msgs/Twist) and optionally to
``/rov/key`` (std_msgs/String, single key label for the HUD).  The process
re-execs itself with the project virtual environment, which is the only
interpreter that can import MuJoCo 3.12 (see ``rov_numpy_nn_slam/runtime.py``).
"""

from __future__ import annotations

import json
import math
import os
import sys
import time


def _repo_root() -> str:
    """Locate the repository root even when started through a catkin relay."""
    env = os.environ.get("RVNN_ROOT", "")
    if env:
        cand = os.path.realpath(env)
        if os.path.isfile(os.path.join(cand, "rov_numpy_nn_slam", "runtime.py")):
            return cand
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.dirname(here), here):
        if os.path.isfile(os.path.join(cand, "rov_numpy_nn_slam", "runtime.py")):
            return cand
    for entry in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if not entry:
            continue
        pkg = os.path.join(entry, "rov_numpy_nn_slam")
        if os.path.isfile(os.path.join(pkg, "package.xml")):
            return os.path.dirname(pkg)
    return os.path.dirname(here)


ROOT = _repo_root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from rov_numpy_nn_slam.runtime import ensure_venv

ensure_venv(__file__)

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Float64, String

from rov_numpy_nn_slam import config as C
from rov_numpy_nn_slam.geometry import wrap_angle
from rov_numpy_nn_slam.sim import ROVSim


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
        self.scene_name = str(rospy.get_param("~scene", "pool"))
        self.seed = int(rospy.get_param("~seed", C.DEFAULT_SEED))
        self.noise = bool(rospy.get_param("~noise", True))
        self.duration = float(rospy.get_param("~duration", 0.0))
        self.mode = str(rospy.get_param("~mode", "TELEOP"))
        self.state_hz = max(1.0, float(rospy.get_param("~state_hz", 20.0)))
        self.sensor_hz = max(1.0, float(rospy.get_param("~sensor_hz",
                                                         C.SENSOR_HZ)))
        self.cmd_timeout = max(0.05, float(rospy.get_param("~cmd_timeout", 0.5)))

        try:
            self.sim = ROVSim(self.scene_name, seed=self.seed, noise=self.noise)
        except Exception as exc:  # pragma: no cover - environment dependent
            rospy.logfatal("cannot create ROVSim for scene %r: %s",
                           self.scene_name, exc)
            raise SystemExit(1)

        self.beam_angles = self.sim.sonar_beam_angles()
        self._cmd = np.zeros(4)
        self._cmd_stamp = -1e9
        self._last_cmd = np.zeros(4)
        self._key = "-"
        self._scan = np.full(C.SONAR_BEAMS, C.SONAR_RANGE_MAX)
        self._odom = np.zeros(4)
        self._depth = 0.0
        self._yaw = 0.0
        self._yaw_rate = 0.0
        self._v_body = np.zeros(3)
        self._summary_done = False
        # fractional-time accumulators keep the publish rates exact even when
        # the 50 Hz loop is not an integer multiple of the lower sensor rate
        self._sensor_accum = 0.0
        self._state_accum = 0.0

        # ---------------------------------------------------------- publishers
        prefix = rospy.get_param("~topic_prefix", "/rov")
        self.pub_scan = rospy.Publisher(prefix + "/scan", LaserScan,
                                        queue_size=5)
        self.pub_odom = rospy.Publisher(prefix + "/odom", Odometry,
                                        queue_size=10)
        self.pub_depth = rospy.Publisher(prefix + "/depth", Float64,
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
        msg.child_frame_id = "rov/base_link"
        msg.pose.pose.position = Point(x=float(od[0]), y=float(od[1]),
                                       z=-float(od[2]))
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
        msg.header.frame_id = "rov/base_link"
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
        msg.header.frame_id = "rov/base_link"
        msg.angle_min = -float(C.SONAR_FOV) / 2.0
        msg.angle_max = float(C.SONAR_FOV) / 2.0
        msg.angle_increment = float(C.SONAR_FOV) / max(1, C.SONAR_BEAMS - 1)
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.sensor_hz
        msg.range_min = float(C.SONAR_RANGE_MIN)
        msg.range_max = float(C.SONAR_RANGE_MAX)
        msg.ranges = [float(v) for v in self._scan]
        msg.intensities = []
        self.pub_scan.publish(msg)

    def _publish_camera(self, stamp) -> None:
        image = self.sim.camera()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "rov/camera"
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
                                  z=-float(pose[2]))
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
            "scene": self.scene_name,
            "key": self._key,
            "mode": self.mode,
        }
        self.pub_state.publish(String(data=json.dumps(payload,
                                                      separators=(",", ":"))))

    # -------------------------------------------------------------- main loop
    def spin(self) -> None:
        rate = rospy.Rate(C.CONTROL_HZ)
        dt = 1.0 / float(C.CONTROL_HZ)
        while not rospy.is_shutdown():
            cmd = self._current_cmd()
            self._last_cmd = cmd.copy()
            self.sim.step(cmd, dt)
            self._odom = self.sim.integrate_odometry(dt)
            self._depth = float(self.sim.depth())
            self._yaw, self._yaw_rate = self.sim.imu()
            try:
                v_body, _ = self.sim._body_velocity()
                self._v_body = np.asarray(v_body, dtype=np.float64)
            except Exception:
                self._v_body = np.asarray(self.sim.dvl(), dtype=np.float64)

            stamp = rospy.Time.now()
            self._publish_odom(stamp)
            self._publish_imu(stamp)
            self.pub_depth.publish(Float64(data=self._depth))
            self._publish_truth(stamp)

            self._sensor_accum += dt
            if self._sensor_accum >= 1.0 / self.sensor_hz:
                self._sensor_accum -= 1.0 / self.sensor_hz
                self._scan = np.asarray(self.sim.sonar(), dtype=np.float64)
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
                     "scene": self.scene_name, "collisions": int(self.sim.collisions),
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
