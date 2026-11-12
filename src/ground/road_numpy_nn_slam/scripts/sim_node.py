#!/usr/bin/env python3
"""GRNN road-vehicle simulation node (physics owner).

Publishes every sensor topic documented in ``docs/ros_api.md``:

=======================  =============================  ==========
topic                    type                           rate
=======================  =============================  ==========
/car/scan                sensor_msgs/LaserScan          10 Hz
/car/odom                nav_msgs/Odometry              20 Hz
/car/gnss                geometry_msgs/PointStamped     20 Hz
/car/imu                 sensor_msgs/Imu                20 Hz
/car/camera/image_raw    sensor_msgs/Image (bgr8)       10 Hz
/car/ground_truth        geometry_msgs/PoseStamped      20 Hz
/car/state               std_msgs/String (JSON)         20 Hz
=======================  =============================  ==========

It subscribes to ``/car/cmd`` (geometry_msgs/Twist: ``linear.x`` = target
speed in m/s, ``angular.z`` = steering angle in rad, positive = left) and
optionally to ``/car/key`` (std_msgs/String, single key label for the HUD).
The Ackermann image is converted to the simulator's normalised
``(throttle, steer)`` command::

    throttle = clip((v_target - v) / 2.0, -1, 1)
    steer    = clip(steer_angle / MAX_STEER, -1, 1)

The process re-execs itself with the project virtual environment (see
``road_numpy_nn_slam/runtime.py``); the kinematic bicycle model itself is pure NumPy
and CPU-only.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time


def _repo_root() -> str:
    """Locate the repository root even when started through a catkin relay."""
    env = os.environ.get("GRNN_ROOT", "")
    if env:
        cand = os.path.realpath(env)
        if os.path.isfile(os.path.join(cand, "road_numpy_nn_slam", "runtime.py")):
            return cand
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.dirname(here), here):
        if os.path.isfile(os.path.join(cand, "road_numpy_nn_slam", "runtime.py")):
            return cand
    for entry in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if not entry:
            continue
        pkg = os.path.join(entry, "road_numpy_nn_slam")
        if os.path.isfile(os.path.join(pkg, "package.xml")):
            return os.path.dirname(pkg)
    return os.path.dirname(here)


ROOT = _repo_root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from road_numpy_nn_slam.runtime import ensure_venv

ensure_venv(__file__)

import numpy as np
import rospy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import String

from road_numpy_nn_slam import config as C
from road_numpy_nn_slam.geometry import wrap_angle

# --------------------------------------------------------------- simulation
try:  # primary class name for the road simulator
    from road_numpy_nn_slam.sim import RoadSim
except Exception:  # pragma: no cover - defensive against module churn
    RoadSim = None
if RoadSim is None:  # pragma: no cover - defensive
    try:
        from road_numpy_nn_slam import sim as _sim_module
        RoadSim = (getattr(_sim_module, "RoadSim", None)
                   or getattr(_sim_module, "GVSim", None)
                   or getattr(_sim_module, "VehicleSim", None))
    except Exception:
        RoadSim = None

# --------------------------------------------------------------------- config
BEAMS = int(getattr(C, "LIDAR_BEAMS", 72))
RANGE_MIN = float(getattr(C, "LIDAR_RANGE_MIN", 0.3))
RANGE_MAX = float(getattr(C, "LIDAR_RANGE_MAX", 40.0))
MAX_SPEED = float(getattr(C, "MAX_SPEED", 12.0))
MAX_STEER = float(getattr(C, "MAX_STEER", 0.6))


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
    """Owns the vehicle loop and publishes the ROS sensor interface."""

    def __init__(self) -> None:
        self.scene_name = str(rospy.get_param("~scene", "downtown"))
        self.seed = int(rospy.get_param("~seed", C.DEFAULT_SEED))
        self.noise = bool(rospy.get_param("~noise", True))
        self.duration = float(rospy.get_param("~duration", 0.0))
        self.mode = str(rospy.get_param("~mode", "TELEOP"))
        self.state_hz = max(1.0, float(rospy.get_param("~state_hz", 20.0)))
        self.sensor_hz = max(1.0, float(rospy.get_param("~sensor_hz",
                                                         C.SENSOR_HZ)))
        # The lidar raymarcher costs ~30 ms per scan on the course VM (the
        # camera ~65 ms because it re-marches the lidar), so the camera is
        # published at a lower rate by default to keep the 20 Hz control loop
        # and the odom/state rates stable.  Pass ~camera_hz:=10 for e2e runs.
        self.camera_hz = max(0.5, float(rospy.get_param("~camera_hz", 5.0)))
        self.cmd_timeout = max(0.05, float(rospy.get_param("~cmd_timeout", 0.5)))

        if RoadSim is None:  # pragma: no cover - defensive
            rospy.logfatal("cannot import the road simulator from "
                           "road_numpy_nn_slam.sim")
            raise SystemExit(1)
        try:
            self.sim = RoadSim(self.scene_name, seed=self.seed,
                               noise=self.noise)
        except Exception as exc:  # pragma: no cover - environment dependent
            rospy.logfatal("cannot create RoadSim for scene %r: %s",
                           self.scene_name, exc)
            raise SystemExit(1)

        self.beam_angles = np.asarray(self.sim.lidar_beam_angles(),
                                      dtype=np.float64).ravel()
        self._cmd = np.zeros(2)          # (target speed m/s, steering rad)
        self._cmd_stamp = -1e9
        self._last_cmd = np.zeros(2)
        self._key = "-"
        self._scan = np.full(BEAMS, RANGE_MAX, dtype=np.float64)
        self._odom = np.zeros(4)
        self._gnss = np.zeros(2)
        self._speed = 0.0
        self._steering = 0.0
        self._yaw = 0.0
        self._yaw_rate = 0.0
        self._light = None
        self._summary_done = False
        # fractional-time accumulators keep the publish rates exact even when
        # the 20 Hz loop is not an integer multiple of the lower sensor rate
        self._scan_accum = 0.0
        self._camera_accum = 0.0
        self._state_accum = 0.0

        # ---------------------------------------------------------- publishers
        self.prefix = str(rospy.get_param("~topic_prefix", "/car"))
        prefix = self.prefix
        self.pub_scan = rospy.Publisher(prefix + "/scan", LaserScan,
                                        queue_size=5)
        self.pub_odom = rospy.Publisher(prefix + "/odom", Odometry,
                                        queue_size=10)
        self.pub_gnss = rospy.Publisher(prefix + "/gnss", PointStamped,
                                        queue_size=10)
        self.pub_imu = rospy.Publisher(prefix + "/imu", Imu, queue_size=10)
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
        rospy.loginfo("sim_node: %d lidar beams, range %.1f..%.1f m, "
                      "MAX_SPEED=%.1f m/s, MAX_STEER=%.2f rad",
                      BEAMS, RANGE_MIN, RANGE_MAX, MAX_SPEED, MAX_STEER)

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

    def _sim_cmd(self) -> np.ndarray:
        """Ackermann ``(target speed, steering)`` -> normalised sim command."""
        speed = float(self.sim.speed)
        throttle = (float(self._cmd[0]) - speed) / 2.0
        steer = float(self._cmd[1]) / max(1e-6, MAX_STEER)
        return np.array([np.clip(throttle, -1.0, 1.0),
                         np.clip(steer, -1.0, 1.0)], dtype=np.float64)

    # ------------------------------------------------------------- publishing
    def _publish_odom(self, stamp) -> None:
        od = self._odom
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.child_frame_id = "car/base_link"
        msg.pose.pose.position = Point(x=float(od[0]), y=float(od[1]), z=0.0)
        msg.pose.pose.orientation = _yaw_quaternion(float(od[3]))
        msg.pose.covariance = _diag(6, [4e-2, 4e-2, 1e-6, 1e-3, 1e-3, 2e-3])
        msg.twist.twist.linear = Vector3(x=float(self._speed), y=0.0,
                                         z=0.0)
        msg.twist.twist.angular = Vector3(x=0.0, y=0.0,
                                          z=float(self._yaw_rate))
        msg.twist.covariance = _diag(6, [2e-2, 2e-2, 1e-6, 1e-3, 1e-3, 1e-3])
        self.pub_odom.publish(msg)

    def _publish_imu(self, stamp) -> None:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "car/base_link"
        msg.orientation = _yaw_quaternion(float(self._yaw))
        msg.orientation_covariance = _diag(3, [1e-2, 1e-2, 2e-2])
        msg.angular_velocity = Vector3(x=0.0, y=0.0,
                                       z=float(self._yaw_rate))
        msg.angular_velocity_covariance = _diag(3, [1e-4, 1e-4, 1e-4])
        msg.linear_acceleration_covariance = _diag(3, [1e-2, 1e-2, 1e-2])
        self.pub_imu.publish(msg)

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
        msg.header.frame_id = "car/base_link"
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
        msg.header.frame_id = "car/camera"
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

    def _light_payload(self):
        light = self.sim.traffic_light()
        if not light:
            return "", None
        state = str(light.get("state", ""))
        payload = {k: (round(float(v), 3) if isinstance(v, (int, float))
                       else v) for k, v in light.items()}
        return state, payload

    def _publish_state(self) -> None:
        pose = self.sim.pose
        self._light = self.sim.traffic_light()
        light_state = str(self._light.get("state", "")) if self._light else ""
        light_payload = None
        if self._light:
            light_payload = {k: (round(float(v), 3)
                                 if isinstance(v, (int, float)) else v)
                             for k, v in self._light.items()}
        payload = {
            "time": round(float(self.sim.time), 3),
            "truth": [round(float(v), 4) for v in pose],
            "odom": [round(float(v), 4) for v in self._odom],
            "cmd": [round(float(v), 4) for v in self._last_cmd],
            "speed": round(float(self._speed), 3),
            "steering": round(float(self._steering), 4),
            "scan": [round(float(v), 3) for v in self._scan],
            "collisions": int(self.sim.collisions),
            "red_light_violations": int(self.sim.red_light_violations),
            "pedestrians_hit": int(self.sim.pedestrians_hit),
            "off_road_time": round(float(self.sim.off_road_time), 3),
            "light_state": light_state,
            "light": light_payload,
            "scene": self.scene_name,
            "key": self._key,
            "mode": self.mode,
        }
        try:  # ground-truth pedestrian positions for the controller / viewer
            payload["pedestrians"] = [
                [round(float(px), 2), round(float(py), 2)]
                for px, py in self.sim.scene.pedestrian_positions(
                    float(self.sim.time))]
        except Exception:
            payload["pedestrians"] = []
        self.pub_state.publish(String(data=json.dumps(payload,
                                                      separators=(",", ":"))))

    # -------------------------------------------------------------- main loop
    def spin(self) -> None:
        rate = rospy.Rate(C.CONTROL_HZ)
        dt = 1.0 / float(C.CONTROL_HZ)
        while not rospy.is_shutdown():
            cmd = self._current_cmd()
            self._last_cmd = cmd.copy()
            self.sim.step(self._sim_cmd(), dt)
            self._odom = np.asarray(self.sim.integrate_odometry(dt),
                                    dtype=np.float64)
            pose = self.sim.pose
            self._speed = float(self.sim.speed)
            self._steering = float(self.sim.steering)
            self._yaw, self._yaw_rate = self.sim.imu()
            self._gnss = np.asarray(self.sim.gnss(), dtype=np.float64)

            stamp = rospy.Time.now()
            self._publish_odom(stamp)
            self._publish_imu(stamp)
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
                     "red_light_violations": int(self.sim.red_light_violations),
                     "pedestrians_hit": int(self.sim.pedestrians_hit),
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
            "red_light_violations": int(self.sim.red_light_violations),
            "pedestrians_hit": int(self.sim.pedestrians_hit),
            "off_road_time": round(float(self.sim.off_road_time), 3),
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
