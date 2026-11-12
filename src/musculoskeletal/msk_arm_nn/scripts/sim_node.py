#!/usr/bin/env python3
"""MSK-Arm simulation node (Hill-muscle arm model owner).

Publishes every sensor topic documented in ``docs/ros_api.md``:

=======================  ===============================  ==========
topic                    type                             rate
=======================  ===============================  ==========
/msk/joints              sensor_msgs/JointState           20 Hz
/msk/odom                nav_msgs/Odometry                20 Hz
/msk/muscle_state        std_msgs/Float32MultiArray       20 Hz
/msk/depth               sensor_msgs/Image (32FC1)        10 Hz
/msk/ee_pose             geometry_msgs/PoseStamped        20 Hz
/msk/state               std_msgs/String (JSON)           20 Hz
=======================  ===============================  ==========

It subscribes to ``/msk/muscle_cmd`` (std_msgs/Float32MultiArray with four
muscle activation commands in ``[0, 1]``: shoulder flexor/extensor, elbow
flexor/extensor) and optionally to ``/msk/key`` (std_msgs/String, single key
label for the HUD).  The simulator maps the 4-vector directly onto its Hill
muscle activation dynamics and reports ``reached`` / ``collisions`` / ``success``
in ``/msk/state``.

The process re-execs itself with the project virtual environment (see
``msk_arm_nn/runtime.py``); the Hill-muscle physics itself is pure NumPy.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time


def _repo_root() -> str:
    """Locate the repository root even when started through a catkin relay."""
    env = os.environ.get("MSK_ROOT", "")
    if env:
        cand = os.path.realpath(env)
        if os.path.isfile(os.path.join(cand, "msk_arm_nn", "runtime.py")):
            return cand
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.dirname(here), here):
        if os.path.isfile(os.path.join(cand, "msk_arm_nn", "runtime.py")):
            return cand
    for entry in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if not entry:
            continue
        pkg = os.path.join(entry, "msk_arm_nn")
        if os.path.isfile(os.path.join(pkg, "package.xml")):
            return os.path.dirname(pkg)
    return os.path.dirname(here)


ROOT = _repo_root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from msk_arm_nn.runtime import ensure_venv

ensure_venv(__file__)

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float32MultiArray, String

from msk_arm_nn import config as C

# --------------------------------------------------------------- simulation
try:  # primary class name for the musculoskeletal arm simulator
    from msk_arm_nn.sim import MskArmSim
except Exception:  # pragma: no cover - defensive against module churn
    MskArmSim = None
if MskArmSim is None:  # pragma: no cover - defensive
    try:
        from msk_arm_nn import sim as _sim_module
        MskArmSim = (getattr(_sim_module, "MskArmSim", None)
                     or getattr(_sim_module, "ArmSim", None)
                     or getattr(_sim_module, "MuscleArmSim", None))
    except Exception:
        MskArmSim = None

# --------------------------------------------------------------------- config
N_MUSCLES = len(getattr(C, "MUSCLES", (0, 0, 0, 0)))
JOINT_NAMES = ("shoulder", "elbow")
MUSCLE_NAMES = tuple(str(m[0]) for m in getattr(C, "MUSCLES", ())) or (
    "shoulder_flexor", "shoulder_extensor", "elbow_flexor", "elbow_extensor")


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(0.5 * float(yaw)),
                      w=math.cos(0.5 * float(yaw)))


def _diag(n: int, values) -> list:
    """Row-major diagonal covariance matrix as a plain python list."""
    out = [0.0] * (n * n)
    for i, v in enumerate(values):
        out[i * n + i] = float(v)
    return out


class SimNode:
    """Owns the Hill-muscle arm loop and publishes the ROS sensor interface."""

    def __init__(self) -> None:
        self.scene_name = str(rospy.get_param("~scene", "reach"))
        self.seed = int(rospy.get_param("~seed", C.DEFAULT_SEED))
        self.noise = bool(rospy.get_param("~noise", True))
        self.duration = float(rospy.get_param("~duration", 0.0))
        self.mode = str(rospy.get_param("~mode", "TELEOP"))
        self.state_hz = max(1.0, float(rospy.get_param("~state_hz", 20.0)))
        self.sensor_hz = max(1.0, float(rospy.get_param("~sensor_hz",
                                                         C.SENSOR_HZ)))
        self.camera_hz = max(0.5, float(rospy.get_param("~camera_hz", 10.0)))
        self.cmd_timeout = max(0.05, float(rospy.get_param("~cmd_timeout", 0.5)))

        if MskArmSim is None:  # pragma: no cover - defensive
            rospy.logfatal("cannot import the arm simulator from "
                           "msk_arm_nn.sim")
            raise SystemExit(1)
        try:
            self.sim = MskArmSim(self.scene_name, seed=self.seed,
                                 noise=self.noise)
        except Exception as exc:  # pragma: no cover - environment dependent
            rospy.logfatal("cannot create MskArmSim for scene %r: %s",
                           self.scene_name, exc)
            raise SystemExit(1)

        self._cmd = np.zeros(N_MUSCLES)
        self._cmd_stamp = -1e9
        self._last_cmd = np.zeros(N_MUSCLES)
        self._key = "-"
        self._q = np.zeros(2)
        self._dq = np.zeros(2)
        self._activations = np.zeros(N_MUSCLES)
        self._forces = np.zeros(N_MUSCLES)
        self._ee = np.zeros(2)
        self._odom = np.zeros(4)
        self._depth = np.zeros((C.CAM_HEIGHT, C.CAM_WIDTH), dtype=np.float32)
        self._ee_velocity = np.zeros(2)
        self._gyro = 0.0
        self._reached = 0
        self._collisions = 0
        self._success = False
        self._max_torque = 0.0
        self._effort = 0.0
        self._summary_done = False
        # fractional-time accumulators keep the publish rates exact
        self._depth_accum = 0.0
        self._state_accum = 0.0

        # ---------------------------------------------------------- publishers
        self.prefix = str(rospy.get_param("~topic_prefix", "/msk"))
        prefix = self.prefix
        self.pub_joints = rospy.Publisher(prefix + "/joints", JointState,
                                          queue_size=10)
        self.pub_odom = rospy.Publisher(prefix + "/odom", Odometry,
                                        queue_size=10)
        self.pub_muscles = rospy.Publisher(prefix + "/muscle_state",
                                           Float32MultiArray, queue_size=10)
        self.pub_depth = rospy.Publisher(prefix + "/depth", Image,
                                         queue_size=2)
        self.pub_ee = rospy.Publisher(prefix + "/ee_pose", PoseStamped,
                                      queue_size=10)
        self.pub_target = rospy.Publisher(prefix + "/target",
                                          Float32MultiArray, queue_size=10)
        self.pub_state = rospy.Publisher(prefix + "/state", String,
                                         queue_size=5)
        # ------------------------------------------------------- subscriptions
        rospy.Subscriber(prefix + "/muscle_cmd", Float32MultiArray,
                         self._on_cmd, queue_size=5)
        rospy.Subscriber(prefix + "/key", String, self._on_key, queue_size=5)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("sim_node: scene=%s seed=%d noise=%s duration=%.1f mode=%s",
                      self.scene_name, self.seed, self.noise, self.duration,
                      self.mode)
        rospy.loginfo("sim_node: %d muscles %s, %d targets, workspace x=%s y=%s",
                      N_MUSCLES, list(MUSCLE_NAMES),
                      len(getattr(self.sim.scene, "targets", ())),
                      getattr(C, "WORK_X", "?"), getattr(C, "WORK_Y", "?"))

    # ------------------------------------------------------------- callbacks
    def _on_cmd(self, msg) -> None:
        data = np.asarray(msg.data, dtype=np.float64).ravel()
        if data.size < N_MUSCLES:
            data = np.pad(data, (0, N_MUSCLES - data.size))
        self._cmd = np.clip(data[:N_MUSCLES], 0.0, 1.0)
        self._cmd_stamp = time.time()

    def _on_key(self, msg: String) -> None:
        text = str(msg.data).strip()
        self._key = text if text else "-"

    def _current_cmd(self) -> np.ndarray:
        if time.time() - self._cmd_stamp > self.cmd_timeout:
            return np.zeros(N_MUSCLES)
        return self._cmd.copy()

    # ------------------------------------------------------------- publishing
    def _publish_joints(self, stamp) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(JOINT_NAMES)
        msg.position = [float(self._q[0]), float(self._q[1])]
        msg.velocity = [float(self._dq[0]), float(self._dq[1])]
        msg.effort = []
        self.pub_joints.publish(msg)

    def _publish_odom(self, stamp) -> None:
        od = self._odom
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.child_frame_id = "msk_arm/ee"
        msg.pose.pose.position = Point(x=float(self._ee[0]),
                                       y=float(self._ee[1]), z=0.0)
        msg.pose.pose.orientation = _quaternion_from_yaw(
            float(self._q[0] + self._q[1]))
        msg.pose.covariance = _diag(6, [4e-4, 4e-4, 1e-6, 1e-4, 1e-4, 2e-4])
        msg.twist.twist.linear = Vector3(x=float(self._ee_velocity[0]),
                                         y=float(self._ee_velocity[1]),
                                         z=0.0)
        msg.twist.twist.angular = Vector3(x=0.0, y=0.0,
                                          z=float(self._gyro))
        msg.twist.covariance = _diag(6, [2e-4, 2e-4, 1e-6, 1e-4, 1e-4, 1e-4])
        self.pub_odom.publish(msg)

    def _publish_muscles(self) -> None:
        msg = Float32MultiArray()
        msg.data = [float(v) for v in np.concatenate(
            [self._activations, self._forces])]
        self.pub_muscles.publish(msg)

    def _publish_depth(self, stamp) -> None:
        depth = np.asarray(self._depth, dtype=np.float32)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "msk_arm/camera"
        msg.height = int(depth.shape[0])
        msg.width = int(depth.shape[1])
        msg.encoding = "32FC1"
        msg.is_bigendian = 0
        msg.step = int(depth.shape[1] * 4)
        msg.data = np.ascontiguousarray(depth, dtype=np.float32).tobytes()
        self.pub_depth.publish(msg)

    def _publish_ee(self, stamp) -> None:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.pose.position = Point(x=float(self._ee[0]),
                                  y=float(self._ee[1]), z=0.0)
        msg.pose.orientation = _quaternion_from_yaw(
            float(self._q[0] + self._q[1]))
        self.pub_ee.publish(msg)

    def _publish_target(self) -> None:
        """Lightweight 50 Hz target stream for the reaching controller.

        ``/msk/state`` is only published at ``~state_hz`` (20 Hz default), but
        the multi-target ``draw`` task advances the goal the instant a target
        is reached; a 50 Hz target message keeps the controller's goal in sync
        with the simulator without the cost of the full JSON snapshot.
        """
        try:
            target = np.asarray(self.sim.goal(), dtype=np.float64).ravel()
            msg = Float32MultiArray()
            msg.data = [float(target[0]), float(target[1]),
                        float(getattr(self.sim, "target_index", 0)),
                        float(self._reached),
                        1.0 if self._success else 0.0]
            self.pub_target.publish(msg)
        except Exception:  # pragma: no cover - defensive
            pass

    def _publish_state(self) -> None:
        target = np.asarray(self.sim.goal(), dtype=np.float64).ravel()
        payload = {
            "time": round(float(self.sim.time), 3),
            "q": [round(float(v), 4) for v in self._q],
            "dq": [round(float(v), 4) for v in self._dq],
            "a": [round(float(v), 4) for v in self._activations],
            "forces": [round(float(v), 3) for v in self._forces],
            "ee": [round(float(self._ee[0]), 4),
                   round(float(self._ee[1]), 4)],
            "odom": [round(float(v), 4) for v in self._odom],
            "target": [round(float(v), 4) for v in target[:2]],
            "target_index": int(getattr(self.sim, "target_index", 0)),
            "reached": int(self._reached),
            "collisions": int(self._collisions),
            "success": bool(self._success),
            "max_torque": round(float(self._max_torque), 3),
            "effort": round(float(self._effort), 3),
            "scene": self.scene_name,
            "key": self._key,
            "mode": self.mode,
        }
        try:  # end-effector trail for the top-down viewer
            payload["trail"] = [[round(float(x), 4), round(float(y), 4)]
                                for x, y in list(self.sim.ee_trail)[-300:]]
        except Exception:
            payload["trail"] = []
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
            self._q = np.asarray(self.sim.joints(), dtype=np.float64)
            self._dq = np.asarray(getattr(self.sim, "dq", np.zeros(2)),
                                  dtype=np.float64)
            self._activations = np.asarray(self.sim.muscle_activations(),
                                           dtype=np.float64)
            self._forces = np.asarray(self.sim.muscle_forces(),
                                      dtype=np.float64)
            self._ee = np.asarray(self.sim.ee(), dtype=np.float64)
            self._ee_velocity = np.asarray(self.sim.jacobian_velocity(),
                                           dtype=np.float64)
            try:
                self._gyro = float(self.sim.imu()[2])
            except Exception:
                self._gyro = float(self._dq[0] + self._dq[1])
            self._reached = int(getattr(self.sim, "reached", 0))
            self._collisions = int(getattr(self.sim, "collisions", 0))
            self._success = bool(getattr(self.sim, "success", False))
            self._max_torque = float(getattr(self.sim, "max_torque", 0.0))
            self._effort = float(getattr(self.sim, "effort", 0.0))

            stamp = rospy.Time.now()
            self._publish_joints(stamp)
            self._publish_odom(stamp)
            self._publish_muscles()
            self._publish_ee(stamp)
            self._publish_target()

            self._depth_accum += dt
            if self._depth_accum >= 1.0 / self.camera_hz:
                self._depth_accum -= 1.0 / self.camera_hz
                self._depth = np.asarray(self.sim.depth_camera(),
                                         dtype=np.float32)
                self._publish_depth(stamp)

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
                     "reached": int(self._reached),
                     "collisions": int(self._collisions),
                     "success": bool(self._success),
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
        payload = {
            "event": "sim_summary",
            "scene": self.scene_name,
            "mode": self.mode,
            "time": round(float(self.sim.time), 3),
            "reached": int(self._reached),
            "collisions": int(self._collisions),
            "success": bool(self._success),
            "max_torque": round(float(self._max_torque), 3),
            "effort": round(float(self._effort), 3),
            "final_ee": [round(float(self._ee[0]), 4),
                         round(float(self._ee[1]), 4)],
            "final_target": [round(float(v), 4)
                             for v in np.asarray(self.sim.goal()).ravel()[:2]],
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
