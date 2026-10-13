#!/usr/bin/env python3
"""
``drl_navigator_node`` - ROS inference node for the SafeDRL drone navigator.

This is the deployment counterpart of :mod:`safe_nav_env`: it runs the
*exact same* observation builder and the *exact same* CBF safety filter as
training, but sources its state from ROS topics instead of PyBullet.

Subscribed
----------
``~odom_topic``   (nav_msgs/Odometry)   - drone pose + twist
``~scan_topic``   (sensor_msgs/LaserScan) - 2-D virtual lidar / rangefinder

Published
---------
``~cmd_vel_topic`` (geometry_msgs/Twist) - 3-D velocity command for the
                                          low-level flight controller
``~status_topic``  (std_msgs/String)     - JSON status: gate index, safety
                                          margin, CBF intervention flag

The node is deliberately tolerant of a missing lidar (it falls back to the
obstacle list from the parameter server) so that the bundled
``dummy_state_publisher.py`` alone is enough to close the loop offline.

ROS1 (catkin).  A ROS2 port differs only in the import block and the
publisher construction, both isolated at the top of the file.
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time

import numpy as np

# --------------------------------------------------------------------------
# ROS imports (keep isolated so the module stays importable without ROS)
# --------------------------------------------------------------------------
try:
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:  # pragma: no cover - allows static analysis off-robot
    ROS_AVAILABLE = False

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from safety import CBFConfig, CBFFilter          # noqa: E402
from safe_nav_env import EnvConfig, _ray_directions  # noqa: E402

try:
    from stable_baselines3 import PPO
    SB3_AVAILABLE = True
except Exception:  # pragma: no cover
    SB3_AVAILABLE = False


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class DrlNavigatorNode:
    """Bridges ROS topics <-> the trained policy + CBF safety filter."""

    def __init__(self):
        cfg = EnvConfig()
        self.cfg = cfg

        # ---- parameters -------------------------------------------------
        self.odom_topic = rospy.get_param("~odom_topic", "/drone/odom")
        self.scan_topic = rospy.get_param("~scan_topic", "/drone/scan")
        self.cmd_topic = rospy.get_param("~cmd_vel_topic", "/drone/cmd_vel")
        self.status_topic = rospy.get_param("~status_topic", "/drone/nav_status")
        self.model_path = rospy.get_param("~model_path", "")
        self.use_cbf = bool(rospy.get_param("~use_cbf", True))
        self.control_rate = float(rospy.get_param("~control_rate", cfg.ctrl_freq))
        self.gate_positions = np.asarray(
            rospy.get_param(
                "~gates",
                cfg.gate_positions.flatten().tolist(),
            ),
            dtype=float,
        ).reshape(-1, 3)
        self.max_velocity = float(rospy.get_param("~max_velocity", cfg.max_velocity))
        self.max_yaw_rate = float(rospy.get_param("~max_yaw_rate", cfg.max_yaw_rate))
        self.dry_run = bool(rospy.get_param("~dry_run", False))

        # obstacles can be supplied by the parameter server instead of a lidar
        obs_param = rospy.get_param("~obstacles", [])
        self.obstacles = [
            {"centre": np.asarray(o["centre"], float), "radius": float(o["radius"])}
            for o in obs_param
        ]

        # ---- policy ------------------------------------------------------
        self.model = None
        if self.model_path and os.path.exists(self.model_path) and SB3_AVAILABLE:
            from lagrangian_ppo import LagrangianPPO  # noqa: F401  (class registration)
            try:
                self.model = PPO.load(self.model_path, device="cpu")
                rospy.loginfo("loaded policy from %s", self.model_path)
            except Exception as exc:  # noqa: BLE001
                rospy.logerr("failed to load policy %s: %s", self.model_path, exc)
        else:
            rospy.logwarn(
                "no policy loaded (model_path=%r) - running the geometric "
                "controller alone; commands will still be safety-filtered.",
                self.model_path,
            )

        # ---- safety filter ------------------------------------------------
        self.cbf = CBFFilter(
            CBFConfig(
                d_safe=cfg.cbf_d_safe, gamma=cfg.cbf_gamma, v_max=self.max_velocity
            ),
            dt=1.0 / self.control_rate,
        )

        # ---- state --------------------------------------------------------
        self._lock = threading.Lock()
        self._pos = np.zeros(3)
        self._vel = np.zeros(3)
        self._rpy = np.zeros(3)
        self._odom_received = False
        self._scan = None
        self._gate_idx = 0
        self._prev_action = np.zeros(4)
        self._last_odom_time = 0.0

        # ---- ROS I/O ------------------------------------------------------
        self.pub_cmd = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.pub_status = rospy.Publisher(self.status_topic, String, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self._on_scan, queue_size=1)

        rospy.loginfo(
            "drl_navigator_node up  odom=%s scan=%s cmd=%s  use_cbf=%s gates=%d",
            self.odom_topic, self.scan_topic, self.cmd_topic, self.use_cbf,
            len(self.gate_positions),
        )

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        rpy = _quat_to_rpy(q.x, q.y, q.z, q.w)
        with self._lock:
            self._pos = np.array([p.x, p.y, p.z], dtype=float)
            self._vel = np.array([v.x, v.y, v.z], dtype=float)
            self._rpy = np.asarray(rpy, dtype=float)
            self._odom_received = True
            self._last_odom_time = time.time()

    def _on_scan(self, msg: "LaserScan"):
        with self._lock:
            self._scan = msg

    # ------------------------------------------------------------------
    # observation - identical layout to SafeNavEnv._compute_obs
    # ------------------------------------------------------------------
    def _build_observation(self) -> np.ndarray:
        c = self.cfg
        with self._lock:
            pos = self._pos.copy()
            vel = self._vel.copy()
            rpy = self._rpy.copy()
            scan = self._scan

        gate = self.gate_positions[min(self._gate_idx, len(self.gate_positions) - 1)]
        pos_rel = (pos - gate) / np.array([c.arena_x, c.arena_y, c.z_max])
        vel_n = vel / self.max_velocity
        rpy_n = rpy / math.pi

        obs_feat = np.zeros((c.n_nearest_obs, 4))
        for j, ob in enumerate(sorted(
            self.obstacles,
            key=lambda o: np.linalg.norm(pos - o["centre"]) - o["radius"],
        )[: c.n_nearest_obs]):
            obs_feat[j, :3] = (ob["centre"] - pos) / c.lidar_max
            obs_feat[j, 3] = ob["radius"] / 0.5

        lidar = self._scan_to_lidar(scan, pos)
        obs = np.concatenate(
            [pos_rel, vel_n, rpy_n, obs_feat.ravel(), lidar, self._prev_action]
        )
        return np.nan_to_num(obs, nan=0.0, posinf=1.0, neginf=-1.0).astype(np.float32)

    def _scan_to_lidar(self, scan, pos) -> np.ndarray:
        """Fold a 2-D LaserScan into the ``n_rays``-bin virtual lidar vector."""
        c = self.cfg
        out = np.full(c.n_rays, 1.0)
        if scan is not None and len(scan.ranges) > 0:
            ranges = np.asarray(scan.ranges, dtype=float)
            ranges[~np.isfinite(ranges)] = scan.range_max if scan.range_max > 0 else c.lidar_max
            lo, hi = scan.angle_min, scan.angle_max
            bins = np.linspace(lo, hi, len(ranges))
            centres = np.linspace(-math.pi, math.pi, c.n_rays, endpoint=False)
            for i, a in enumerate(centres):
                m = np.abs(((bins - a + math.pi) % (2 * math.pi)) - math.pi) < (math.pi / c.n_rays)
                if np.any(m):
                    out[i] = min(float(np.min(ranges[m])), c.lidar_max) / c.lidar_max
            return out

        # no scan: synthesise ranges from the known obstacle list
        dirs = _ray_directions(c.n_rays)
        for i, d in enumerate(dirs):
            best = c.lidar_max
            for ob in self.obstacles:
                oc = pos - ob["centre"]
                b = float(d @ oc)
                cc = float(oc @ oc) - ob["radius"] ** 2
                disc = b * b - cc
                if disc < 0:
                    continue
                t = -b - math.sqrt(disc)
                if 1e-6 < t < best:
                    best = t
            out[i] = min(best, c.lidar_max) / c.lidar_max
        return out

    # ------------------------------------------------------------------
    def _update_gate_index(self):
        if self._gate_idx >= len(self.gate_positions):
            return
        g = self.gate_positions[self._gate_idx]
        radial = math.hypot(self._pos[1] - g[1], self._pos[2] - g[2])
        if self._pos[0] > g[0] and radial < self.cfg.gate_aperture:
            self._gate_idx += 1
            rospy.loginfo("gate %d passed -> targeting gate %d",
                          self._gate_idx, self._gate_idx + 1)

    def _barrier_terms(self):
        hs, gs = [], []
        for ob in self.obstacles:
            d = self._pos - ob["centre"]
            n = float(np.linalg.norm(d))
            hs.append(n - ob["radius"] - self.cfg.drone_radius - self.cfg.cbf_d_safe)
            gs.append(d / max(n, 1e-9))
        if not hs:
            return np.zeros(0), np.zeros((0, 3))
        return np.asarray(hs), np.asarray(gs)

    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(self.control_rate)
        warned = False
        while not rospy.is_shutdown():
            if not self._odom_received:
                if not warned:
                    rospy.logwarn_throttle(5.0, "waiting for odometry on %s ...", self.odom_topic)
                    warned = True
                rate.sleep()
                continue

            obs = self._build_observation()

            if self.model is not None:
                action, _ = self.model.predict(obs, deterministic=True)
                action = np.clip(np.asarray(action, float).reshape(-1), -1.0, 1.0)
            else:
                action = self._geometric_fallback()

            v_des = action[:3] * self.max_velocity
            h, g = self._barrier_terms()
            v_cmd = self.cbf.filter(v_des, h, g) if self.use_cbf else v_des

            yaw_rate = float(action[3]) * self.max_yaw_rate
            self._publish(v_cmd, yaw_rate)
            self._prev_action = action
            self._update_gate_index()
            self._publish_status(v_des, v_cmd, h)
            rate.sleep()

    def _geometric_fallback(self) -> np.ndarray:
        """Pure pursuit towards the active gate - keeps the node useful
        even before a policy has been trained."""
        gate = self.gate_positions[min(self._gate_idx, len(self.gate_positions) - 1)]
        delta = gate - self._pos
        n = np.linalg.norm(delta)
        v = delta / n if n > 1e-6 else np.zeros(3)
        return np.array([v[0], v[1], v[2], 0.0])

    def _publish(self, v_cmd: np.ndarray, yaw_rate: float):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = (float(v_cmd[0]), float(v_cmd[1]), float(v_cmd[2]))
        msg.angular.z = float(yaw_rate)
        if self.dry_run:
            rospy.loginfo_throttle(
                1.0, "DRY-RUN cmd_vel=(%.2f, %.2f, %.2f) yaw_rate=%.2f",
                v_cmd[0], v_cmd[1], v_cmd[2], yaw_rate,
            )
        self.pub_cmd.publish(msg)

    def _publish_status(self, v_des, v_cmd, h):
        margin = float(np.min(h)) if h.size else float("inf")
        payload = {
            "t": round(rospy.get_time(), 3),
            "gate_index": int(self._gate_idx),
            "n_gates": int(len(self.gate_positions)),
            "position": [round(float(x), 3) for x in self._pos],
            "v_des": [round(float(x), 3) for x in v_des],
            "v_cmd": [round(float(x), 3) for x in v_cmd],
            "safety_margin": None if not np.isfinite(margin) else round(margin, 3),
            "cbf_active": bool(
                np.linalg.norm(np.asarray(v_cmd) - np.asarray(v_des)) > 1e-3
            ),
        }
        self.pub_status.publish(String(data=json.dumps(payload)))


def _quat_to_rpy(x, y, z, w):
    """Quaternion -> roll/pitch/yaw (matches pybullet.getEulerFromQuaternion)."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def main():
    if not ROS_AVAILABLE:
        print("ERROR: rospy not found. Source your ROS workspace first:\n"
              "  source /opt/ros/noetic/setup.bash", file=sys.stderr)
        return 1
    rospy.init_node("drl_navigator", anonymous=False)
    node = DrlNavigatorNode()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
