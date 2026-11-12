#!/usr/bin/env python3
"""HNN humanoid autonomous navigation node (A* + walking controller).

Subscribes
    /humanoid/scan        (sensor_msgs/LaserScan, 72 beams)
    /humanoid/slam_pose   (geometry_msgs/PoseStamped)  localisation from SLAM
    /humanoid/odom        (nav_msgs/Odometry)          fallback when SLAM is absent

Publishes
    /humanoid/cmd         (geometry_msgs/Twist: linear.x = walk speed m/s,
                           angular.z = turn rate rad/s)
    /humanoid/plan        (nav_msgs/Path)
    /humanoid/status      (std_msgs/String, JSON; ``GOAL_REACHED`` when done)

The mission controller comes from ``humanoid_walk_nn.control`` (owned by the
navigation engineer); when that module is missing the node degrades to a
status-only observer so the rest of the bringup keeps working.  The controller
may return a world-frame velocity command ``[vx, vy, (vz), yaw_rate]``, a
direct walk pair ``[walk_speed, turn_rate]`` or a normalised pair
``[v_norm, w_norm]``; ``~cmd_format`` selects the interpretation
(``auto``/``world``/``walk``/``normalized``) and the node also prefers the
controller's ``target_speed`` / ``turn_rate`` telemetry attributes when
present.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time


def _repo_root() -> str:
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
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from humanoid_walk_nn import config as C
from humanoid_walk_nn.scenes import get_scene

try:  # independent module owned by nav-eng
    from humanoid_walk_nn.control import MissionController
    CONTROL_AVAILABLE = True
    CONTROL_ERROR = ""
except Exception as _exc:  # pragma: no cover - concurrent development
    MissionController = None
    CONTROL_AVAILABLE = False
    CONTROL_ERROR = str(_exc)

LIDAR_BEAMS = int(getattr(C, "LIDAR_BEAMS", 72))
MAX_SPEED = float(getattr(C, "MAX_SPEED", 1.25))
MAX_BACKWARD = float(getattr(C, "MAX_BACKWARD", 0.5))
MAX_TURN_RATE = float(getattr(C, "MAX_TURN_RATE", 1.5))
PLAN_RESOLUTION = float(getattr(C, "PLAN_RESOLUTION", 0.20))
GOAL_TOL = float(getattr(C, "GOAL_TOLERANCE", 0.45))
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
        self.scene_name = str(rospy.get_param("~scene", "hall"))
        self.use_nn = bool(rospy.get_param("~use_nn", False))
        self.replan = float(rospy.get_param("~replan", 2.0))
        self.control_hz = max(1.0, float(rospy.get_param("~control_hz", 20.0)))
        self.goal_radius = float(rospy.get_param("~goal_radius", 0.6))
        self.laser_stale = max(0.2, float(rospy.get_param("~laser_stale", 1.5)))
        self.cmd_format = str(rospy.get_param("~cmd_format", "auto")).lower()

        self.scene = get_scene(self.scene_name)
        self.goal = np.array([
            self._float_param("~goal_x", self.scene.goal[0]),
            self._float_param("~goal_y", self.scene.goal[1]),
            0.0,
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
        self._speed = 0.0
        self._balance = 0.0
        self._falls = 0
        self._reached = False
        self._zero = Twist()

        if CONTROL_AVAILABLE:
            try:
                try:
                    self.controller = MissionController(
                        self.scene, resolution=PLAN_RESOLUTION,
                        replan_interval=self.replan)
                except TypeError:
                    self.controller = MissionController(self.scene)
                rospy.loginfo("nav_node: MissionController ready "
                              "(replan %.1f s, goal %s)", self.replan,
                              np.round(self.goal, 2).tolist())
            except Exception as exc:  # pragma: no cover - defensive
                rospy.logerr("nav_node: MissionController init failed: %s", exc)
                self.controller = None
        else:
            rospy.logwarn("nav_node: humanoid_walk_nn.control unavailable (%s) - "
                          "status-only mode", CONTROL_ERROR)
        if self.use_nn:
            self._load_nn()

        self.pub_cmd = rospy.Publisher("/humanoid/cmd", Twist, queue_size=5)
        self.pub_plan = rospy.Publisher("/humanoid/plan", Path, queue_size=2)
        self.pub_status = rospy.Publisher("/humanoid/status", String,
                                          queue_size=5)
        rospy.Subscriber("/humanoid/scan", LaserScan, self._on_scan,
                         queue_size=2)
        rospy.Subscriber("/humanoid/slam_pose", PoseStamped,
                         self._on_slam_pose, queue_size=2)
        rospy.Subscriber("/humanoid/odom", Odometry, self._on_odom,
                         queue_size=5)
        rospy.Subscriber("/humanoid/state", String, self._on_state,
                         queue_size=5)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("nav_node: scene=%s goal=%s use_nn=%s cmd_format=%s",
                      self.scene_name, np.round(self.goal, 2).tolist(),
                      self.use_nn, self.cmd_format)

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
                          "falling back to the mission controller", path)
            return
        try:
            from humanoid_walk_nn.nn.mlp import MLP
            from humanoid_walk_nn import datasets as D  # noqa: F401
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
            from humanoid_walk_nn import datasets as D
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
            return np.asarray(self._nn.predict(x), dtype=np.float64).ravel()
        except Exception as exc:  # pragma: no cover - defensive
            rospy.logwarn_throttle(5.0, "nav_node: control MLP inference failed: %s", exc)
            return None

    # ------------------------------------------------------- command mapping
    def _controller_command(self, cmd, pose):
        """Map the mission controller output to ``(walk_speed, turn_rate)``.

        Preferred: the controller's telemetry attributes (``target_speed`` /
        ``walk_speed`` and ``turn_rate`` / ``yaw_rate`` / ``omega``).  Fallback
        formats: world-frame ``[vx, vy, (vz), yaw_rate]``, direct
        ``[walk_speed, turn_rate]`` or normalised ``[v_norm, w_norm]``.
        """
        def _attr(*names):
            for name in names:
                value = getattr(self.controller, name, None)
                if value is not None:
                    try:
                        return float(value)
                    except (TypeError, ValueError):
                        continue
            return None

        fmt = self.cmd_format
        arr = np.asarray(cmd, dtype=np.float64).ravel() if cmd is not None \
            else np.zeros(0)
        if fmt == "auto":
            if arr.size >= 4:
                fmt = "world"
            else:
                fmt = "walk"
        if fmt == "world" and arr.size >= 2:
            yaw = float(pose[3])
            speed = float(arr[0]) * math.cos(yaw) + float(arr[1]) * math.sin(yaw)
            turn = float(arr[-1])
            return (float(np.clip(speed, -MAX_BACKWARD, MAX_SPEED)),
                    float(np.clip(turn, -MAX_TURN_RATE, MAX_TURN_RATE)))
        # telemetry attributes are authoritative for the walk/normalized forms
        speed = _attr("target_speed", "walk_speed", "v_target", "speed_target")
        turn = _attr("target_turn", "turn_rate", "yaw_rate", "omega",
                     "omega_target", "steer_angle")
        if speed is not None and turn is not None:
            model = str(getattr(self.controller, "cmd_units", "")).lower()
            if fmt == "normalized" or model in ("normalized", "normalised", "nn"):
                speed *= MAX_SPEED
                turn *= MAX_TURN_RATE
            return (float(np.clip(speed, -MAX_BACKWARD, MAX_SPEED)),
                    float(np.clip(turn, -MAX_TURN_RATE, MAX_TURN_RATE)))
        if arr.size >= 2:
            speed, turn = float(arr[0]), float(arr[1])
            if fmt == "normalized" or (abs(speed) <= 1.001
                                       and abs(turn) <= 1.001
                                       and fmt == "walk"
                                       and self._speed < 0.05
                                       and abs(speed) > MAX_SPEED):
                speed *= MAX_SPEED
                turn *= MAX_TURN_RATE
            return (float(np.clip(speed, -MAX_BACKWARD, MAX_SPEED)),
                    float(np.clip(turn, -MAX_TURN_RATE, MAX_TURN_RATE)))
        if arr.size == 1:
            return (float(np.clip(float(arr[0]), -MAX_BACKWARD, MAX_SPEED)), 0.0)
        return 0.0, 0.0

    # ------------------------------------------------------------- callbacks
    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = np.asarray(msg.ranges, dtype=np.float64)
        self._scan_beam_angles = _scan_angles(msg)
        self._scan_stamp = time.time()

    def _on_slam_pose(self, msg: PoseStamped) -> None:
        self._slam = np.array([
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            0.0,
            _yaw_from_quaternion(msg.pose.orientation),
        ], dtype=np.float64)
        self._slam_stamp = time.time()

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = np.array([
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            0.0,
            _yaw_from_quaternion(msg.pose.pose.orientation),
        ], dtype=np.float64)
        self._odom_stamp = time.time()
        try:
            self._speed = float(msg.twist.twist.linear.x)
        except Exception:
            pass

    def _on_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        try:
            self._speed = float(data.get("speed", self._speed))
        except (TypeError, ValueError):
            pass
        try:
            self._balance = float(data.get("balance", self._balance) or 0.0)
        except (TypeError, ValueError):
            pass
        try:
            self._falls = int(data.get("falls", self._falls) or 0)
        except (TypeError, ValueError):
            pass

    # ------------------------------------------------------------------ state
    def _localisation(self):
        now = time.time()
        if self._slam is not None and now - self._slam_stamp < self.laser_stale:
            return self._slam.copy()
        if self._odom is not None and now - self._odom_stamp < self.laser_stale:
            return self._odom.copy()
        return None

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
            for key in ("target_speed", "turn_rate", "walk_speed", "yaw_rate",
                        "stopped_for_cart", "yields_for_cart"):
                value = getattr(self.controller, key, None)
                if value is not None:
                    try:
                        payload[key] = round(float(value), 3)
                    except (TypeError, ValueError):
                        pass
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
                # keep the robot stopped while waiting / after arrival
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

            cmd = np.zeros(2)
            try:
                try:
                    cmd = np.asarray(self.controller.update(
                        pose, self.goal, scan, beam_angles, dt,
                        balance=self._balance, falls=self._falls),
                        dtype=np.float64).ravel()
                except TypeError:
                    # older update() signature without balance/falls kwargs
                    cmd = np.asarray(self.controller.update(
                        pose, self.goal, scan, beam_angles, dt),
                        dtype=np.float64).ravel()
                if self._nn is not None:
                    nn = self._nn_cmd(pose, scan)
                    if nn is not None and nn.size >= 2:
                        cmd = np.asarray(nn, dtype=np.float64).ravel()
            except Exception as exc:  # pragma: no cover - defensive
                rospy.logwarn_throttle(2.0, "nav_node: controller update failed: %s", exc)
                cmd = np.zeros(2)

            speed, turn = self._controller_command(cmd, pose)

            dist = math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1])
            if dist <= max(self.goal_radius, GOAL_TOL):
                self._reached = True
                speed, turn = 0.0, 0.0
                rospy.loginfo("nav_node: GOAL_REACHED (dist %.2f m)", dist)
                self._publish_status("GOAL_REACHED",
                                     {"final_distance": round(float(dist), 3)})

            msg = Twist()
            msg.linear.x = float(speed)
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.z = float(turn)
            self.pub_cmd.publish(msg)

            if now - last_plan >= 0.5:
                last_plan = now
                self._publish_plan()
            if now - last_status >= 1.0:
                last_status = now
                self._publish_status("GOAL_REACHED" if self._reached
                                     else "RUNNING",
                                     {"distance": round(float(dist), 3),
                                      "speed": round(float(speed), 3),
                                      "turn_rate": round(float(turn), 3)})
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
