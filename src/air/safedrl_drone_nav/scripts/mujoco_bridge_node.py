#!/usr/bin/env python3
"""MuJoCo plant for the SafeDRL navigator.

This node is the MuJoCo counterpart of ``dummy_state_publisher.py`` and of the
AirSim bridge.  It loads the bundled MJCF scene, applies the velocity command
published on ``/drone/cmd_vel`` through a first-order velocity loop, steps the
rigid-body dynamics, and publishes ENU/FLU odometry and a horizontal
``LaserScan`` produced with MuJoCo's ray caster.

The velocity loop uses the same abstraction the CBF filter assumes
in ``safety.py``::

    m * dv/dt = (m / tau) * (v_des - v) + m * g

so the barrier's ``tau_lag`` and braking-distance parameters remain valid when
the policy is moved from the offline PyBullet checks to MuJoCo.

Usage::

    roslaunch safedrl_drone_nav mujoco_navigate.launch
    roslaunch safedrl_drone_nav mujoco_navigate.launch use_cbf:=false
"""

from __future__ import annotations

import math
import os
import sys
import threading

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from sim_frames import quat_wxyz_to_rpy  # noqa: E402

try:
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    ROS_AVAILABLE = True
except ImportError:  # pragma: no cover - keeps helpers importable
    ROS_AVAILABLE = False

try:
    import mujoco
    MUJOCO_AVAILABLE = True
except Exception:  # pragma: no cover - optional dependency
    mujoco = None
    MUJOCO_AVAILABLE = False


DEFAULT_MODEL = os.path.abspath(
    os.path.join(_HERE, "..", "models", "quadrotor_scene.xml")
)


def default_model_path() -> str:
    return DEFAULT_MODEL


def make_laserscan(ranges, angle_min=-math.pi, angle_max=None, range_max=5.0):
    """Fill a LaserScan message from a range array (helper kept ROS-free)."""
    ranges = np.asarray(ranges, dtype=float).reshape(-1)
    n = max(int(ranges.size), 1)
    step = 2.0 * math.pi / n
    if angle_max is None:
        angle_max = angle_min + step * (n - 1)
    scan = LaserScan()
    scan.header.frame_id = "base_link"
    scan.angle_min = float(angle_min)
    scan.angle_max = float(angle_max)
    scan.angle_increment = float(step)
    scan.range_min = 0.10
    scan.range_max = float(range_max)
    scan.ranges = ranges.astype(float).tolist()
    return scan


class MujocoBridgeNode:
    """MuJoCo rigid-body plant exposing the standard ROS topics."""

    def __init__(self):
        if not ROS_AVAILABLE:
            raise RuntimeError("rospy is not available")
        if not MUJOCO_AVAILABLE:
            raise RuntimeError(
                "the mujoco package is not installed; run `pip install mujoco>=3.0`"
            )

        self.model_path = rospy.get_param("~model_path", DEFAULT_MODEL)
        self.control_rate = float(rospy.get_param("~control_rate", 30.0))
        self.tau_lag = float(rospy.get_param("~tau_lag", 0.20))
        self.max_accel = float(rospy.get_param("~max_accel", 12.0))
        self.max_yaw_accel = float(rospy.get_param("~max_yaw_accel", 12.0))
        self.publish_scan = bool(rospy.get_param("~publish_scan", True))
        self.n_rays = int(rospy.get_param("~n_rays", 72))
        self.scan_range = float(rospy.get_param("~scan_range", 5.0))
        self.odom_topic = rospy.get_param("~odom_topic", "/drone/odom")
        self.cmd_topic = rospy.get_param("~cmd_vel_topic", "/drone/cmd_vel")
        self.scan_topic = rospy.get_param("~scan_topic", "/drone/scan")
        start = rospy.get_param("~start", [0.0, 0.0, 1.5])

        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"
        )
        if self.body_id < 0:
            raise RuntimeError("MJCF scene has no body named 'quadrotor'")

        self.data.qpos[:3] = np.asarray(start, dtype=float)
        self.data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
        mujoco.mj_forward(self.model, self.data)

        self.mass = float(self.model.body_mass[self.body_id])
        self.inertia_z = float(self.model.body_inertia[self.body_id][2])
        self.gravity = float(abs(self.model.opt.gravity[2]))
        self.dt = float(self.model.opt.timestep)
        self.substeps = max(1, int(round(1.0 / (self.control_rate * self.dt))))

        self._lock = threading.Lock()
        self._cmd = np.zeros(4)
        self._last_cmd_time = 0.0

        self.pub_odom = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)
        self.pub_scan = rospy.Publisher(self.scan_topic, LaserScan, queue_size=1)
        rospy.Subscriber(self.cmd_topic, Twist, self._on_cmd, queue_size=1)

        rospy.loginfo(
            "MuJoCo plant: model=%s mass=%.3f kg dt=%.4f s substeps=%d",
            self.model_path,
            self.mass,
            self.dt,
            self.substeps,
        )

    def _on_cmd(self, msg: "Twist"):
        with self._lock:
            self._cmd = np.array(
                [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z],
                dtype=float,
            )
            self._last_cmd_time = rospy.get_time()

    def _desired(self):
        with self._lock:
            cmd = self._cmd.copy()
            stale = rospy.get_time() - self._last_cmd_time > 0.5
        if stale:
            cmd[:] = 0.0
        return cmd

    def _body_velocity(self):
        """World-frame linear and angular velocity from MuJoCo."""
        res = np.zeros(6, dtype=float)
        mujoco.mj_objectVelocity(
            self.model,
            self.data,
            mujoco.mjtObj.mjOBJ_BODY,
            self.body_id,
            res,
            0,
        )
        return res[3:6].copy(), res[0:3].copy()

    def _apply_velocity_loop(self, cmd):
        v_actual, w_actual = self._body_velocity()
        v_des = cmd[:3]
        force = self.mass * (v_des - v_actual) / max(self.tau_lag, 1e-3)
        force[2] += self.mass * self.gravity
        f_norm = float(np.linalg.norm(force))
        f_max = self.mass * self.max_accel + self.mass * self.gravity
        if f_norm > f_max:
            force *= f_max / f_norm

        torque = np.zeros(3)
        torque[2] = (
            self.inertia_z * (cmd[3] - w_actual[2]) / max(self.tau_lag, 1e-3)
        )
        t_max = self.inertia_z * self.max_yaw_accel
        torque[2] = float(np.clip(torque[2], -t_max, t_max))

        self.data.xfrc_applied[self.body_id, :3] = force
        self.data.xfrc_applied[self.body_id, 3:] = torque

    def _step(self, cmd):
        for _ in range(self.substeps):
            self._apply_velocity_loop(cmd)
            mujoco.mj_step(self.model, self.data)

    def _publish_odom(self):
        pos = self.data.qpos[:3]
        quat = self.data.qpos[3:7]
        v_world, w_world = self._body_velocity()
        roll, pitch, yaw = quat_wxyz_to_rpy(*quat)

        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        msg.pose.pose.orientation.x = float(quat[1])
        msg.pose.pose.orientation.y = float(quat[2])
        msg.pose.pose.orientation.z = float(quat[3])
        msg.pose.pose.orientation.w = float(quat[0])
        msg.twist.twist.linear.x = float(v_world[0])
        msg.twist.twist.linear.y = float(v_world[1])
        msg.twist.twist.linear.z = float(v_world[2])
        msg.twist.twist.angular.x = float(w_world[0])
        msg.twist.twist.angular.y = float(w_world[1])
        msg.twist.twist.angular.z = float(w_world[2])
        self.pub_odom.publish(msg)
        return roll, pitch, yaw

    def _ray_ranges(self):
        origin = np.asarray(self.data.qpos[:3], dtype=float)
        geomid = np.zeros(1, dtype=np.int32)
        geomgroup = np.array([1, 1, 0, 0, 0, 0], dtype=np.uint8)
        ranges = np.full(self.n_rays, self.scan_range, dtype=float)
        step = 2.0 * math.pi / self.n_rays
        for i in range(self.n_rays):
            angle = -math.pi + i * step
            direction = np.array([math.cos(angle), math.sin(angle), 0.0])
            dist = mujoco.mj_ray(
                self.model,
                self.data,
                origin,
                direction,
                geomgroup,
                1,
                self.body_id,
                geomid,
            )
            if dist is not None and dist >= 0.0:
                ranges[i] = min(float(dist), self.scan_range)
        return ranges

    def _publish_scan(self):
        if not self.publish_scan:
            return
        scan = make_laserscan(
            self._ray_ranges(), range_max=self.scan_range
        )
        scan.header.stamp = rospy.Time.now()
        self.pub_scan.publish(scan)

    def spin(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            cmd = self._desired()
            self._step(cmd)
            self._publish_odom()
            self._publish_scan()
            rate.sleep()


def main():
    if not ROS_AVAILABLE:
        print(
            "ERROR: rospy not found. Source your ROS workspace first:\n"
            "  source /opt/ros/noetic/setup.bash",
            file=sys.stderr,
        )
        return 1
    rospy.init_node("mujoco_bridge", anonymous=False)
    try:
        node = MujocoBridgeNode()
    except Exception as exc:  # noqa: BLE001
        rospy.logfatal("MuJoCo bridge failed to start: %s", exc)
        return 2
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
