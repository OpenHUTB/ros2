#!/usr/bin/env python3
"""HNN humanoid simulation node (walking model owner).

Publishes every sensor topic documented in ``docs/ros_api.md``:

==========================  ===============================  ==========
topic                       type                             rate
==========================  ===============================  ==========
/humanoid/scan              sensor_msgs/LaserScan            10 Hz
/humanoid/odom              nav_msgs/Odometry                20 Hz
/humanoid/gnss              geometry_msgs/PointStamped       20 Hz
/humanoid/imu               sensor_msgs/Imu                  20 Hz
/humanoid/joints            sensor_msgs/JointState           20 Hz
/humanoid/camera/depth      sensor_msgs/Image (32FC1)        5 Hz
/humanoid/camera/image_raw  sensor_msgs/Image (bgr8)         5 Hz
/humanoid/ground_truth      geometry_msgs/PoseStamped        20 Hz
/humanoid/state             std_msgs/String (JSON)           20 Hz
==========================  ===============================  ==========

It subscribes to ``/humanoid/cmd`` (geometry_msgs/Twist: ``linear.x`` = walk
speed in m/s, ``angular.z`` = turn rate in rad/s) and optionally to
``/humanoid/key`` (std_msgs/String, single key label for the HUD).  The
simulator maps the Twist directly onto ``(walk_speed, turn_rate)``.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time


def _repo_root() -> str:
    """Locate the repository root even when started through a catkin relay."""
    env = os.environ.get("HNN_ROOT", "")
    if env:
        cand = os.path.realpath(env)
        if os.path.isfile(os.path.join(cand, "humanoid_walk_nn", "runtime.py")):
            return cand
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.dirname(here), here):
        if os.path.isfile(os.path.join(cand, "humanoid_walk_nn", "runtime.py")):
            return cand
    for entry in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if not entry:
            continue
        pkg = os.path.join(entry, "humanoid_walk_nn")
        if os.path.isfile(os.path.join(pkg, "package.xml")):
            return os.path.dirname(pkg)
    return os.path.dirname(here)


ROOT = _repo_root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from humanoid_walk_nn.runtime import ensure_venv

ensure_venv(__file__)

import numpy as np
import rospy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
from std_msgs.msg import String

from humanoid_walk_nn import config as C

# --------------------------------------------------------------- simulation
try:  # primary class name for the humanoid simulator
    from humanoid_walk_nn.sim import HumanoidSim
except Exception:  # pragma: no cover - defensive against module churn
    HumanoidSim = None
if HumanoidSim is None:  # pragma: no cover - defensive
    try:
        from humanoid_walk_nn import sim as _sim_module
        HumanoidSim = (getattr(_sim_module, "HumanoidSim", None)
                       or getattr(_sim_module, "WalkingSim", None)
                       or getattr(_sim_module, "BipedSim", None))
    except Exception:
        HumanoidSim = None

# --------------------------------------------------------------------- config
BEAMS = int(getattr(C, "LIDAR_BEAMS", 72))
RANGE_MIN = float(getattr(C, "LIDAR_RANGE_MIN", 0.15))
RANGE_MAX = float(getattr(C, "LIDAR_RANGE_MAX", 8.0))
MAX_SPEED = float(getattr(C, "MAX_SPEED", 1.25))
MAX_BACKWARD = float(getattr(C, "MAX_BACKWARD", 0.5))
MAX_TURN_RATE = float(getattr(C, "MAX_TURN_RATE", 1.5))
JOINT_NAMES = ("hip_l", "knee_l", "ankle_l", "hip_r", "knee_r", "ankle_r")


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(0.5 * roll), math.sin(0.5 * roll)
    cp, sp = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
    cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    return Quaternion(x=sr * cp * cy - cr * sp * sy,
                      y=cr * sp * cy + sr * cp * sy,
                      z=cr * cp * sy - sr * sp * cy,
                      w=cr * cp * cy + sr * sp * sy)


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


class SimNode:
    """Owns the walking loop and publishes the ROS sensor interface."""

    def __init__(self) -> None:
        self.scene_name = str(rospy.get_param("~scene", "hall"))
        self.seed = int(rospy.get_param("~seed", C.DEFAULT_SEED))
        self.noise = bool(rospy.get_param("~noise", True))
        self.duration = float(rospy.get_param("~duration", 0.0))
        self.mode = str(rospy.get_param("~mode", "TELEOP"))
        self.state_hz = max(1.0, float(rospy.get_param("~state_hz", 20.0)))
        self.sensor_hz = max(1.0, float(rospy.get_param("~sensor_hz",
                                                         C.SENSOR_HZ)))
        # the depth camera ray-marches the floor plan, so it is decimated
        self.camera_hz = max(0.5, float(rospy.get_param("~camera_hz", 5.0)))
        self.cmd_timeout = max(0.05, float(rospy.get_param("~cmd_timeout", 0.5)))

        if HumanoidSim is None:  # pragma: no cover - defensive
            rospy.logfatal("cannot import the humanoid simulator from "
                           "humanoid_walk_nn.sim")
            raise SystemExit(1)
        try:
            self.sim = HumanoidSim(self.scene_name, seed=self.seed,
                                   noise=self.noise)
        except Exception as exc:  # pragma: no cover - environment dependent
            rospy.logfatal("cannot create HumanoidSim for scene %r: %s",
                           self.scene_name, exc)
            raise SystemExit(1)

        self.beam_angles = np.asarray(self.sim.lidar_beam_angles(),
                                      dtype=np.float64).ravel()
        self._cmd = np.zeros(2)          # (walk speed m/s, turn rate rad/s)
        self._cmd_stamp = -1e9
        self._last_cmd = np.zeros(2)
        self._key = "-"
        self._scan = np.full(BEAMS, RANGE_MAX, dtype=np.float64)
        self._odom = np.zeros(4)
        self._gnss = np.zeros(2)
        self._speed = 0.0
        self._balance = 0.0
        self._falls = 0
        self._collisions = 0
        self._phase = 0.0
        self._imu = (0.0, 0.0, 0.0)
        self._joints = np.zeros(6)
        self._summary_done = False
        # fractional-time accumulators keep the publish rates exact
        self._scan_accum = 0.0
        self._camera_accum = 0.0
        self._state_accum = 0.0

        # ---------------------------------------------------------- publishers
        self.prefix = str(rospy.get_param("~topic_prefix", "/humanoid"))
        prefix = self.prefix
        self.pub_scan = rospy.Publisher(prefix + "/scan", LaserScan,
                                        queue_size=5)
        self.pub_odom = rospy.Publisher(prefix + "/odom", Odometry,
                                        queue_size=10)
        self.pub_gnss = rospy.Publisher(prefix + "/gnss", PointStamped,
                                        queue_size=10)
        self.pub_imu = rospy.Publisher(prefix + "/imu", Imu, queue_size=10)
        self.pub_joints = rospy.Publisher(prefix + "/joints", JointState,
                                          queue_size=10)
        self.pub_depth = rospy.Publisher(prefix + "/camera/depth", Image,
                                         queue_size=2)
        self.pub_camera = rospy.Publisher(prefix + "/camera/image_raw", Image,
                                          queue_size=2)
        self.pub_truth = rospy.Publisher(prefix + "/ground_truth",
                                         PoseStamped, queue_size=10)
        self.pub_state = rospy.Publisher(prefix + "/state", String,
                                         queue_size=5)
        # ------------------------------------------------------- subscriptions
        rospy.Subscriber(prefix + "/cmd", Twist, self._on_cmd, queue_size=5)
        rospy.Subscriber(prefix + "/key", String, self._on_key, queue_size=5)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("sim_node: scene=%s seed=%d noise=%s duration=%.1f mode=%s",
                      self.scene_name, self.seed, self.noise, self.duration,
                      self.mode)
        rospy.loginfo("sim_node: %d lidar beams, range %.2f..%.2f m, "
                      "MAX_SPEED=%.2f m/s, MAX_TURN_RATE=%.2f rad/s",
                      BEAMS, RANGE_MIN, RANGE_MAX, MAX_SPEED, MAX_TURN_RATE)

    # ------------------------------------------------------------- callbacks
    def _on_cmd(self, msg) -> None:
        self._cmd = np.array([float(msg.linear.x), float(msg.angular.z)])
        self._cmd_stamp = time.time()

    def _on_key(self, msg: String) -> None:
        text = str(msg.data).strip()
        self._key = text if text else "-"

    def _current_cmd(self) -> np.ndarray:
        if time.time() - self._cmd_stamp > self.cmd_timeout:
            return np.zeros(2)
        return self._cmd.copy()

    # ------------------------------------------------------------- publishing
    def _publish_odom(self, stamp) -> None:
        od = self._odom
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.child_frame_id = "humanoid/base_link"
        msg.pose.pose.position = Point(x=float(od[0]), y=float(od[1]), z=0.0)
        msg.pose.pose.orientation = _yaw_quaternion(float(od[3]))
        msg.pose.covariance = _diag(6, [4e-2, 4e-2, 1e-6, 1e-3, 1e-3, 2e-3])
        msg.twist.twist.linear = Vector3(x=float(self._speed), y=0.0, z=0.0)
        msg.twist.twist.angular = Vector3(x=0.0, y=0.0,
                                          z=float(self._imu[2]))
        msg.twist.covariance = _diag(6, [2e-2, 2e-2, 1e-6, 1e-3, 1e-3, 1e-3])
        self.pub_odom.publish(msg)

    def _publish_imu(self, stamp) -> None:
        roll, pitch, yaw_rate = self._imu
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "humanoid/base_link"
        msg.orientation = _quaternion_from_rpy(float(roll), float(pitch),
                                               float(self._odom[3]))
        msg.orientation_covariance = _diag(3, [1e-2, 1e-2, 2e-2])
        msg.angular_velocity = Vector3(x=0.0, y=0.0, z=float(yaw_rate))
        msg.angular_velocity_covariance = _diag(3, [1e-4, 1e-4, 1e-4])
        msg.linear_acceleration_covariance = _diag(3, [1e-2, 1e-2, 1e-2])
        self.pub_imu.publish(msg)

    def _publish_joints(self, stamp) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in self._joints]
        msg.velocity = []
        msg.effort = []
        self.pub_joints.publish(msg)

    def _publish_gnss(self, stamp) -> None:
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.point = Point(x=float(self._gnss[0]), y=float(self._gnss[1]),
                          z=0.0)
        self.pub_gnss.publish(msg)

    def _publish_scan(self, stamp) -> None:
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = "humanoid/base_link"
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

    def _publish_depth(self, stamp, depth=None) -> np.ndarray:
        if depth is None:
            depth = self.sim.camera_depth()
        depth = np.asarray(depth, dtype=np.float32)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "humanoid/camera"
        msg.height = int(depth.shape[0])
        msg.width = int(depth.shape[1])
        msg.encoding = "32FC1"
        msg.is_bigendian = 0
        msg.step = int(depth.shape[1] * 4)
        msg.data = np.ascontiguousarray(depth, dtype=np.float32).tobytes()
        self.pub_depth.publish(msg)
        return depth

    def _publish_camera(self, stamp, depth=None) -> None:
        # the synthetic RGB frame is the depth image replicated on three
        # channels; reusing the depth plane avoids a second ray-marching pass
        if depth is None:
            depth = self.sim.camera_depth()
        depth = np.asarray(depth, dtype=np.float32)
        image = np.clip(depth * 255.0, 0, 255).astype(np.uint8)
        image = np.repeat(image[:, :, None], 3, axis=2)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "humanoid/camera"
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
        msg.pose.position = Point(x=float(pose[0]), y=float(pose[1]), z=0.0)
        msg.pose.orientation = _yaw_quaternion(float(pose[3]))
        self.pub_truth.publish(msg)

    def _publish_state(self) -> None:
        pose = self.sim.pose
        payload = {
            "time": round(float(self.sim.time), 3),
            "truth": [round(float(v), 4) for v in pose],
            "odom": [round(float(v), 4) for v in self._odom],
            "imu": [round(float(self._imu[0]), 4),
                    round(float(self._imu[1]), 4),
                    round(float(self._imu[2]), 4)],
            "joints": [round(float(v), 4) for v in self._joints],
            "speed": round(float(self._speed), 3),
            "turn_rate": round(float(self._imu[2]), 3),
            "balance": round(float(self._balance), 4),
            "falls": int(self._falls),
            "collisions": int(self._collisions),
            "phase": round(float(self._phase), 3),
            "steps": int(getattr(self.sim, "steps", 0)),
            "distance": round(float(getattr(self.sim, "distance", 0.0)), 3),
            "scene": self.scene_name,
            "key": self._key,
            "mode": self.mode,
            "scan": [round(float(v), 3) for v in self._scan],
        }
        try:  # moving carts for the top-down viewer
            payload["carts"] = [
                [round(float(cx), 2), round(float(cy), 2),
                 round(float(hw), 2), round(float(hh), 2)]
                for cx, cy, hw, hh in self.sim.scene.cart_positions(
                    float(self.sim.time))]
        except Exception:
            payload["carts"] = []
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
            self._odom = np.asarray(self.sim.integrate_odometry(dt),
                                    dtype=np.float64)
            pose = self.sim.pose
            self._speed = float(self.sim.speed)
            self._balance = float(getattr(self.sim, "balance", 0.0))
            self._falls = int(getattr(self.sim, "falls", 0))
            self._collisions = int(getattr(self.sim, "collisions", 0))
            self._phase = float(getattr(self.sim, "phase", 0.0))
            self._imu = tuple(float(v) for v in self.sim.imu())
            self._joints = np.asarray(self.sim.joints(), dtype=np.float64)
            self._gnss = np.asarray(self.sim.gnss(), dtype=np.float64)

            stamp = rospy.Time.now()
            self._publish_odom(stamp)
            self._publish_imu(stamp)
            self._publish_joints(stamp)
            self._publish_gnss(stamp)
            self._publish_truth(stamp)

            self._scan_accum += dt
            if self._scan_accum >= 1.0 / self.sensor_hz:
                self._scan_accum -= 1.0 / self.sensor_hz
                self._scan = np.asarray(self.sim.lidar(), dtype=np.float64)
                self._publish_scan(stamp)

            self._camera_accum += dt
            if self._camera_accum >= 1.0 / self.camera_hz:
                self._camera_accum -= 1.0 / self.camera_hz
                depth = self._publish_depth(stamp)
                self._publish_camera(stamp, depth)

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
                     "falls": int(self._falls),
                     "collisions": int(self._collisions),
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
            "falls": int(self._falls),
            "collisions": int(self._collisions),
            "distance": round(float(getattr(self.sim, "distance", 0.0)), 3),
            "steps": int(getattr(self.sim, "steps", 0)),
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
