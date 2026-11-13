#!/usr/bin/env python3
"""UVNN UAV autonomous navigation node (A* + pure pursuit + safety filter).

Subscribes
    /uav/scan        (sensor_msgs/LaserScan, 72 beams)
    /uav/slam_pose   (geometry_msgs/PoseStamped)  localisation from SLAM
    /uav/altitude    (std_msgs/Float64)           measured altitude
    /uav/odom        (nav_msgs/Odometry)          fallback when SLAM is absent

Publishes
    /uav/cmd_vel     (geometry_msgs/Twist, linear.z = climb rate)
    /uav/plan        (nav_msgs/Path)
    /uav/status      (std_msgs/String, JSON; ``GOAL_REACHED`` when done)

The mission controller comes from ``uav_numpy_nn_slam.control`` (owned by the
navigation engineer); when that module is missing the node degrades to a
status-only observer so the rest of the bringup keeps working.  With
``~use_nn:=true`` the trained ``control_mlp`` network replaces the geometric
heading policy when a compatible model is available.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time


def _repo_root() -> str:
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
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String

from uav_numpy_nn_slam import config as C
from uav_numpy_nn_slam.scenes import get_scene

try:  # independent module owned by nav-eng
    from uav_numpy_nn_slam.control import MissionController
    CONTROL_AVAILABLE = True
    CONTROL_ERROR = ""
except Exception as _exc:  # pragma: no cover - concurrent development
    MissionController = None
    CONTROL_AVAILABLE = False
    CONTROL_ERROR = str(_exc)

LIDAR_BEAMS = int(getattr(C, "LIDAR_BEAMS", 72))
MAX_VX = float(getattr(C, "MAX_VX", 8.0))
MAX_VY = float(getattr(C, "MAX_VY", 5.0))
MAX_VZ = float(getattr(C, "MAX_VZ", 3.0))
MAX_YAW_RATE = float(getattr(C, "MAX_YAW_RATE", 1.2))
PLAN_RESOLUTION = float(getattr(C, "PLAN_RESOLUTION", 0.40))
# 360 deg lidar: beams every 2*pi / N, first beam at -pi (matches sim_node)
BEAM_ANGLES = np.linspace(-math.pi, math.pi, LIDAR_BEAMS, endpoint=False)


def _yaw_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0,
                      z=math.sin(0.5 * float(yaw)),
                      w=math.cos(0.5 * float(yaw)))


def _yaw_from_quaternion(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _scan_angles(msg: LaserScan) -> np.ndarray:
    """Beam angles of a LaserScan in the vehicle frame (rad)."""
    n = len(msg.ranges)
    if n <= 0:
        return BEAM_ANGLES
    inc = float(msg.angle_increment)
    if not np.isfinite(inc) or inc == 0.0:
        inc = 2.0 * math.pi / LIDAR_BEAMS
    return float(msg.angle_min) + np.arange(n, dtype=np.float64) * inc


class NavNode:
    """Mission controller ROS wrapper."""

    def __init__(self) -> None:
        self.scene_name = str(rospy.get_param("~scene", "city"))
        self.use_nn = bool(rospy.get_param("~use_nn", False))
        self.replan = float(rospy.get_param("~replan", 2.0))
        self.control_hz = max(1.0, float(rospy.get_param("~control_hz", 20.0)))
        self.goal_radius = float(rospy.get_param("~goal_radius", 1.0))
        self.alt_radius = float(rospy.get_param("~alt_radius", 1.5))
        self.slam_stale = max(0.2, float(rospy.get_param("~slam_stale", 1.5)))

        self.scene = get_scene(self.scene_name)
        self.goal = np.array([
            self._float_param("~goal_x", self.scene.goal[0]),
            self._float_param("~goal_y", self.scene.goal[1]),
            self._float_param("~goal_alt", self.scene.goal[2]),
        ], dtype=np.float64)

        self.controller = None
        self._nn = None
        self._nn_feature = ""
        self._scan = None
        self._scan_beam_angles = BEAM_ANGLES
        self._scan_stamp = -1e9
        self._slam = None
        self._slam_stamp = -1e9
        self._odom = None
        self._odom_stamp = -1e9
        self._altitude = None
        self._altitude_stamp = -1e9
        self._reached = False
        self._zero = Twist()

        if CONTROL_AVAILABLE:
            try:
                try:
                    self.controller = MissionController(
                        self.scene, resolution=PLAN_RESOLUTION,
                        replan_interval=self.replan, world_frame=True)
                except TypeError:
                    # older signature without the world_frame keyword
                    self.controller = MissionController(
                        self.scene, resolution=PLAN_RESOLUTION,
                        replan_interval=self.replan)
                rospy.loginfo("nav_node: MissionController ready "
                              "(replan %.1f s, goal %s)", self.replan,
                              np.round(self.goal, 2).tolist())
            except Exception as exc:  # pragma: no cover - defensive
                rospy.logerr("nav_node: MissionController init failed: %s", exc)
                self.controller = None
        else:
            rospy.logwarn("nav_node: uav_numpy_nn_slam.control unavailable (%s) - "
                          "status-only mode", CONTROL_ERROR)
        if self.use_nn:
            self._load_nn()

        self.pub_cmd = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=5)
        self.pub_plan = rospy.Publisher("/uav/plan", Path, queue_size=2)
        self.pub_status = rospy.Publisher("/uav/status", String, queue_size=5)
        rospy.Subscriber("/uav/scan", LaserScan, self._on_scan, queue_size=2)
        rospy.Subscriber("/uav/slam_pose", PoseStamped, self._on_slam_pose,
                         queue_size=2)
        rospy.Subscriber("/uav/odom", Odometry, self._on_odom, queue_size=5)
        rospy.Subscriber("/uav/altitude", Float64, self._on_altitude,
                         queue_size=5)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("nav_node: scene=%s goal=%s use_nn=%s",
                      self.scene_name, np.round(self.goal, 2).tolist(),
                      self.use_nn)

    # ---------------------------------------------------------------- helpers
    @staticmethod
    def _float_param(name: str, default: float) -> float:
        value = rospy.get_param(name, "")
        if value is None or value == "":
            return float(default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    def _load_nn(self) -> None:
        path = str(rospy.get_param(
            "~nn_model", os.path.join(C.MODELS_DIR, "control_mlp.npz")))
        if not os.path.isfile(path):
            rospy.logwarn("nav_node: ~use_nn=true but model %s not found - "
                          "falling back to the geometric controller", path)
            return
        try:
            from uav_numpy_nn_slam.nn.mlp import MLP
            from uav_numpy_nn_slam import datasets as D  # noqa: F401
            net = MLP.load(path)
            in_dim = int(net.sizes[0])
            if in_dim == 20:
                self._nn_feature = "sectors+goal"
            elif in_dim == LIDAR_BEAMS:
                self._nn_feature = "scan"
            elif in_dim == 6:
                self._nn_feature = "obs"
            else:
                rospy.logwarn("nav_node: control_mlp input width %d is not "
                              "supported (expected 20/%d/6)", in_dim,
                              LIDAR_BEAMS)
                return
            self._nn = net
            rospy.loginfo("nav_node: control MLP loaded (%s, in=%d out=%d)",
                          path, in_dim, int(net.sizes[-1]))
        except Exception as exc:  # pragma: no cover - defensive
            rospy.logwarn("nav_node: cannot load control MLP: %s", exc)

    def _nn_cmd(self, pose, scan):
        if self._nn is None:
            return None
        try:
            from uav_numpy_nn_slam import datasets as D
            p = np.asarray(pose, dtype=np.float64).ravel()
            if self._nn_feature == "sectors+goal":
                x = np.concatenate([
                    D.sector_features(np.asarray(scan, dtype=np.float64)[None, :],
                                      16)[0],
                    D.goal_features(p, self.goal)])
            elif self._nn_feature == "scan":
                x = D.scan_features(np.asarray(scan, dtype=np.float64))
            else:
                x = D.obs_features(p, self.goal)
            out = np.asarray(self._nn.predict(x), dtype=np.float64).ravel()
            if out.size < 2:
                return None
            surge, wz = float(out[0]), float(out[1])
            if abs(surge) <= 1.5 and abs(wz) <= 1.5:
                surge *= MAX_VX
                wz *= MAX_YAW_RATE
            return (float(np.clip(surge, -MAX_VX, MAX_VX)),
                    float(np.clip(wz, -MAX_YAW_RATE, MAX_YAW_RATE)))
        except Exception as exc:  # pragma: no cover - defensive
            rospy.logwarn_throttle(5.0, "nav_node: control MLP inference failed: %s", exc)
            return None

    # ------------------------------------------------------------- callbacks
    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = np.asarray(msg.ranges, dtype=np.float64)
        self._scan_beam_angles = _scan_angles(msg)
        self._scan_stamp = time.time()

    def _on_slam_pose(self, msg: PoseStamped) -> None:
        self._slam = np.array([
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            _yaw_from_quaternion(msg.pose.orientation),
        ], dtype=np.float64)
        self._slam_stamp = time.time()

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = np.array([
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
            _yaw_from_quaternion(msg.pose.pose.orientation),
        ], dtype=np.float64)
        self._odom_stamp = time.time()

    def _on_altitude(self, msg: Float64) -> None:
        self._altitude = float(msg.data)
        self._altitude_stamp = time.time()

    # ------------------------------------------------------------------ state
    def _localisation(self):
        now = time.time()
        if self._slam is not None and now - self._slam_stamp < self.slam_stale:
            return self._slam.copy()
        if self._odom is not None and now - self._odom_stamp < self.slam_stale:
            return self._odom.copy()
        return None

    @staticmethod
    def _world_to_body(cmd: np.ndarray, yaw: float) -> np.ndarray:
        """Convert a world-frame drive ``[vx, vy, vz, wz]`` to body frame.

        ``MissionController`` returns world-frame velocities (its default
        ``world_frame=True``), while ``/uav/cmd_vel`` carries the intuitive
        body frame (``linear.x`` forward, ``linear.y`` right).
        """
        out = np.array(cmd, dtype=np.float64, copy=True)
        c, s = math.cos(yaw), math.sin(yaw)
        vx, vy = float(out[0]), float(out[1])
        out[0] = c * vx + s * vy
        out[1] = -s * vx + c * vy
        return out

    def _publish_plan(self) -> None:
        if self.controller is None:
            return
        path = list(getattr(self.controller, "path", []) or [])
        if len(path) < 2:
            return
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        for x, y in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position = Point(x=float(x), y=float(y), z=0.0)
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            msg.poses.append(pose)
        self.pub_plan.publish(msg)

    def _publish_status(self, status: str, extra=None) -> None:
        payload = {
            "node": "nav",
            "status": status,
            "scene": self.scene_name,
            "goal": [round(float(v), 3) for v in self.goal],
            "reached": bool(self._reached),
        }
        if self.controller is not None:
            payload["replans"] = int(getattr(self.controller, "replans", 0))
            payload["path_points"] = len(getattr(self.controller, "path", []) or [])
        if extra:
            payload.update(extra)
        self.pub_status.publish(String(data=json.dumps(payload,
                                                       separators=(",", ":"))))

    # -------------------------------------------------------------------- run
    def spin(self) -> None:
        rate = rospy.Rate(self.control_hz)
        dt = 1.0 / self.control_hz
        last_status = 0.0
        last_plan = 0.0
        while not rospy.is_shutdown():
            now = time.time()
            scan = self._scan
            beam_angles = self._scan_beam_angles
            pose = self._localisation()
            if (self.controller is None or scan is None or pose is None
                    or self._reached):
                # keep the vehicle stopped while waiting / after arrival
                self.pub_cmd.publish(self._zero)
                if not self._reached:
                    rospy.logwarn_throttle(5.0, "nav_node: waiting for "
                                           "scan/localisation")
                if now - last_status >= 1.0:
                    last_status = now
                    self._publish_status("GOAL_REACHED" if self._reached
                                         else "WAITING")
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break
                continue

            altitude = pose[2]
            if (self._altitude is not None
                    and now - self._altitude_stamp < self.slam_stale):
                altitude = self._altitude
            pose = np.array([pose[0], pose[1], altitude, pose[3]],
                            dtype=np.float64)
            cmd = np.zeros(4)
            cmd_is_body = False
            try:
                cmd = np.asarray(self.controller.update(
                    pose, self.goal, scan, beam_angles, dt),
                    dtype=np.float64).ravel()
                if cmd.size < 4:
                    cmd = np.pad(cmd, (0, 4 - cmd.size))
                if self._nn is not None:
                    nn = self._nn_cmd(pose, scan)
                    if nn is not None:
                        alt_ctrl = getattr(self.controller, "alt_hold", None)
                        if alt_ctrl is None:
                            alt_ctrl = getattr(self.controller, "depth", None)
                        climb = 0.0
                        if alt_ctrl is not None:
                            climb = float(alt_ctrl.update(
                                altitude, self.goal[2], dt))
                        base = np.array([nn[0], 0.0, climb, nn[1]])
                        cmd = np.asarray(self.controller.safety.apply(
                            base, scan, beam_angles), dtype=np.float64)
                        cmd_is_body = True
            except Exception as exc:  # pragma: no cover - defensive
                rospy.logwarn_throttle(2.0, "nav_node: controller update failed: %s", exc)
                cmd = np.zeros(4)
            if not cmd_is_body:
                cmd = self._world_to_body(cmd, float(pose[3]))

            dist = math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1])
            alt_err = abs(altitude - self.goal[2])
            if dist <= self.goal_radius and alt_err <= self.alt_radius:
                self._reached = True
                cmd = np.zeros(4)
                rospy.loginfo("nav_node: GOAL_REACHED (dist %.2f m, alt err "
                              "%.2f m)", dist, alt_err)
                self._publish_status("GOAL_REACHED",
                                     {"final_distance": round(float(dist), 3),
                                      "alt_error": round(float(alt_err), 3)})

            msg = Twist()
            msg.linear.x = float(np.clip(cmd[0], -MAX_VX, MAX_VX))
            msg.linear.y = float(np.clip(cmd[1], -MAX_VY, MAX_VY))
            msg.linear.z = float(np.clip(cmd[2], -MAX_VZ, MAX_VZ))
            msg.angular.z = float(np.clip(cmd[3], -MAX_YAW_RATE, MAX_YAW_RATE))
            self.pub_cmd.publish(msg)

            if now - last_plan >= 0.5:
                last_plan = now
                self._publish_plan()
            if now - last_status >= 1.0:
                last_status = now
                self._publish_status("GOAL_REACHED" if self._reached
                                     else "RUNNING",
                                     {"distance": round(float(dist), 3),
                                      "altitude": round(float(altitude), 3)})
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _on_shutdown(self) -> None:
        try:
            self.pub_cmd.publish(Twist())
            self._publish_status("SHUTDOWN")
        except Exception:
            pass


def main() -> int:
    rospy.init_node("nav_node", anonymous=False)
    node = NavNode()
    try:
        node.spin()
    finally:
        node._on_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
