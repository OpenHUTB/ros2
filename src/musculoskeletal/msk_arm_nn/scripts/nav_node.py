#!/usr/bin/env python3
"""MSK-Arm reaching-controller node (Hill-muscle synergy control).

Subscribes
    /msk/state   (std_msgs/String, JSON)   joints, EE, target, target_index
    /msk/depth   (sensor_msgs/Image 32FC1) optional workspace depth image

Publishes
    /msk/muscle_cmd   (std_msgs/Float32MultiArray, 4 activations in [0, 1])
    /msk/plan         (nav_msgs/Path, end-effector workspace path)
    /msk/status       (std_msgs/String, JSON)

The controller comes from ``msk_arm_nn.control`` (owned by the navigation
engineer).  The wrapper is deliberately API-tolerant: it looks for the usual
class names (``MissionController`` / ``ReachController`` / ``MuscleController``
/ ``ArmController``), binds the ``update`` arguments by name from the available
``q`` / ``dq`` / ``ee`` / ``pose`` / ``goal`` / ``target`` / ``dt`` values and
clips whatever it returns into the four activation commands.  If the module is
missing the node degrades to a status-only observer so the rest of the bringup
keeps working.
"""

from __future__ import annotations

import inspect
import json
import math
import os
import sys
import time


def _repo_root() -> str:
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
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float32MultiArray, String

from msk_arm_nn import config as C
from msk_arm_nn.scenes import get_scene

N_MUSCLES = len(getattr(C, "MUSCLES", (0, 0, 0, 0)))
PLAN_RESOLUTION = float(getattr(C, "PLAN_RESOLUTION", 0.02))
CONTROL_HZ = float(getattr(C, "CONTROL_HZ", 50.0))

# ------------------------------------------------------------- controller
CONTROL_AVAILABLE = False
CONTROL_ERROR = ""
_CONTROLLER_CLASS = None
try:  # independent module owned by nav-eng
    from msk_arm_nn import control as _control_module
    for _name in ("MissionController", "ReachController", "MuscleController",
                  "ArmController", "SynergyController"):
        _candidate = getattr(_control_module, _name, None)
        if _candidate is not None:
            _CONTROLLER_CLASS = _candidate
            CONTROL_AVAILABLE = True
            break
    if _CONTROLLER_CLASS is None:
        CONTROL_ERROR = "no controller class found in msk_arm_nn.control"
except Exception as _exc:  # pragma: no cover - concurrent development
    CONTROL_ERROR = str(_exc)


def _yaw_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0,
                      z=math.sin(0.5 * float(yaw)),
                      w=math.cos(0.5 * float(yaw)))


class NavNode:
    """Reaching controller ROS wrapper."""

    def __init__(self) -> None:
        self.scene_name = str(rospy.get_param("~scene", "reach"))
        self.replan = float(rospy.get_param("~replan", 2.0))
        self.control_hz = max(1.0, float(rospy.get_param("~control_hz", 50.0)))
        self.state_stale = max(0.5, float(rospy.get_param("~state_stale", 1.5)))

        self.scene = get_scene(self.scene_name)
        self.controller = None
        self._state = None
        self._state_stamp = -1e9
        self._pose_ee = None
        self._pose_stamp = -1e9
        self._pose_seq = 0
        self._target = None
        self._target_stamp = -1e9
        self._joints = None
        self._joints_stamp = -1e9
        self._success = False
        self._cached_cmd = np.zeros(N_MUSCLES)
        self._zero = Float32MultiArray()
        self._zero.data = [0.0] * N_MUSCLES
        self._success_reported = False

        if CONTROL_AVAILABLE:
            cls = _CONTROLLER_CLASS
            created = None
            for kwargs in ({"resolution": PLAN_RESOLUTION,
                            "replan_interval": self.replan},
                           {"resolution": PLAN_RESOLUTION},
                           {}):
                try:
                    created = cls(self.scene, **kwargs)
                    break
                except TypeError:
                    continue
                except Exception as exc:  # pragma: no cover - defensive
                    rospy.logerr("nav_node: %s init failed: %s", cls.__name__, exc)
                    break
            self.controller = created
            if created is not None:
                rospy.loginfo("nav_node: %s ready (scene=%s, targets=%s)",
                              cls.__name__, self.scene_name,
                              list(getattr(self.scene, "targets", ())))
                # aero-nav: reset once per episode so the controller's internal
                # clock / previous-pose bookkeeping starts with this run
                try:
                    reset = getattr(created, "reset", None)
                    if reset is not None:
                        reset()
                except Exception as exc:  # pragma: no cover - defensive
                    rospy.logwarn("nav_node: controller reset failed: %s", exc)
            else:
                rospy.logwarn("nav_node: controller could not be constructed - "
                              "status-only mode")
        else:
            rospy.logwarn("nav_node: msk_arm_nn.control unavailable (%s) - "
                          "status-only mode", CONTROL_ERROR)

        self.pub_cmd = rospy.Publisher("/msk/muscle_cmd",
                                       Float32MultiArray, queue_size=5)
        self.pub_plan = rospy.Publisher("/msk/plan", Path, queue_size=2)
        self.pub_status = rospy.Publisher("/msk/status", String, queue_size=5)
        rospy.Subscriber("/msk/state", String, self._on_state, queue_size=5)
        rospy.Subscriber("/msk/target", Float32MultiArray, self._on_target,
                         queue_size=10)
        rospy.Subscriber("/msk/depth", Image, self._on_depth, queue_size=2)
        rospy.Subscriber("/msk/ee_pose", PoseStamped, self._on_ee_pose,
                         queue_size=10)
        rospy.Subscriber("/msk/odom", Odometry, self._on_odom, queue_size=10)
        rospy.Subscriber("/msk/joints", JointState, self._on_joints,
                         queue_size=10)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("nav_node: scene=%s replan=%.1f control_hz=%.1f",
                      self.scene_name, self.replan, self.control_hz)

    # ------------------------------------------------------------- callbacks
    def _on_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self._state = data
        self._state_stamp = time.time()
        self._success = bool(data.get("success", False))

    def _on_target(self, msg: Float32MultiArray) -> None:
        # 50 Hz lightweight goal stream (keeps the multi-target draw task in
        # sync with the simulator between 20 Hz /msk/state snapshots)
        try:
            data = np.asarray(msg.data, dtype=np.float64).ravel()
            if data.size >= 2:
                self._target = np.array([data[0], data[1]], dtype=np.float64)
                self._target_stamp = time.time()
        except Exception:
            pass

    def _on_depth(self, msg: Image) -> None:
        # keep the last depth image around for controllers that consume it
        try:
            self._depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                int(msg.height), int(msg.width)).copy()
        except Exception:
            self._depth = None

    def _on_ee_pose(self, msg: PoseStamped) -> None:
        # /msk/ee_pose carries the exact end-effector pose at the full control
        # rate (50 Hz); /msk/odom is the noisy fused estimate, kept as fallback
        try:
            position = msg.pose.position
            self._pose_ee = np.array([float(position.x), float(position.y),
                                      0.0, 0.0], dtype=np.float64)
            self._pose_stamp = time.time()
            self._pose_seq += 1
        except Exception:
            pass

    def _on_odom(self, msg: Odometry) -> None:
        # /msk/odom runs at the full control rate, so the controller can see a
        # fresh end-effector pose on every 50 Hz cycle instead of the 20 Hz
        # /msk/state snapshot
        try:
            position = msg.pose.pose.position
            self._pose_ee = np.array([float(position.x), float(position.y),
                                      0.0, 0.0], dtype=np.float64)
            self._pose_stamp = time.time()
        except Exception:
            pass

    def _on_joints(self, msg: JointState) -> None:
        try:
            self._joints = (np.asarray(msg.position[:2], dtype=np.float64),
                            np.asarray(msg.velocity[:2], dtype=np.float64))
            self._joints_stamp = time.time()
        except Exception:
            pass

    # ------------------------------------------------------- controller call
    def _bind_update(self, pose_ee, q, dq, goal):
        """Call ``controller.update`` binding arguments by name."""
        values = {
            "q": q,
            "dq": dq,
            "joint_positions": q,
            "joint_velocities": dq,
            "ee": pose_ee[:2],
            "pose": pose_ee,
            "end_effector": pose_ee[:2],
            "goal": goal,
            "target": goal,
            "targets": list(getattr(self.scene, "targets", ())),
            "dt": 1.0 / self.control_hz,
            "depth": getattr(self, "_depth", None),
            "time": float(getattr(self, "_sim_time", 0.0)),
        }
        try:
            signature = inspect.signature(self.controller.update)
            kwargs = {}
            for pname, param in signature.parameters.items():
                if pname == "self":
                    continue
                if param.kind in (param.VAR_POSITIONAL, param.VAR_KEYWORD):
                    continue
                if pname in values:
                    kwargs[pname] = values[pname]
                elif param.default is param.empty:
                    kwargs[pname] = values.get(pname)
            return self.controller.update(**kwargs)
        except (TypeError, ValueError):
            pass
        # positional fallbacks for the common controller shapes
        for args in ((q, dq, goal), (pose_ee, goal), (q, dq, goal,
                                                     1.0 / self.control_hz),
                     (q, dq, goal, 1.0 / self.control_hz)):
            try:
                return self.controller.update(*args)
            except TypeError:
                continue
        raise TypeError("unsupported controller.update signature")

    def _command(self, pose_ee, q, dq, goal):
        raw = self._bind_update(pose_ee, q, dq, goal)
        arr = np.asarray(raw, dtype=np.float64).ravel() if raw is not None \
            else np.zeros(N_MUSCLES)
        if arr.size < N_MUSCLES:
            arr = np.pad(arr, (0, N_MUSCLES - arr.size))
        arr = np.nan_to_num(arr[:N_MUSCLES], nan=0.0)
        # accept either [0,1] activations or [-1,1] normalised outputs
        if float(np.min(arr)) < -0.05:
            arr = (arr + 1.0) * 0.5
        return np.clip(arr, 0.0, 1.0)

    # ------------------------------------------------------------ publishing
    def _publish_plan(self) -> None:
        if self.controller is None:
            return
        path = list(getattr(self.controller, "path", []) or [])
        if len(path) < 2:
            return
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        for point in path:
            try:
                x, y = float(point[0]), float(point[1])
            except (TypeError, ValueError, IndexError):
                continue
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position = Point(x=x, y=y, z=0.0)
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            msg.poses.append(pose)
        self.pub_plan.publish(msg)

    def _publish_status(self, status: str, extra=None) -> None:
        payload = {
            "node": "nav",
            "status": status,
            "scene": self.scene_name,
            "success": bool(self._success),
        }
        if self.controller is not None:
            payload["replans"] = int(getattr(self.controller, "replans", 0))
            payload["path_points"] = len(getattr(self.controller, "path", []) or [])
            for key in ("target_ee", "reach_error", "synergy", "target_speed",
                        "target_activation"):
                value = getattr(self.controller, key, None)
                if value is not None:
                    try:
                        payload[key] = round(float(value), 4)
                    except (TypeError, ValueError):
                        pass
        if self._state is not None:
            try:
                payload["target"] = [float(v)
                                     for v in self._state.get("target", [])[:2]]
                payload["ee"] = [float(v) for v in self._state.get("ee", [])[:2]]
            except (TypeError, ValueError):
                pass
        if extra:
            payload.update(extra)
        self.pub_status.publish(String(data=json.dumps(payload,
                                                       separators=(",", ":"))))

    # -------------------------------------------------------------------- run
    def spin(self) -> None:
        rate = rospy.Rate(self.control_hz)
        last_status = 0.0
        last_plan = 0.0
        last_pose_seq = -1
        while not rospy.is_shutdown():
            now = time.time()
            state = self._state
            if (self.controller is None or state is None
                    or now - self._state_stamp > self.state_stale):
                self.pub_cmd.publish(self._zero)
                if now - last_status >= 1.0:
                    last_status = now
                    self._publish_status("SUCCESS" if self._success
                                         else "WAITING")
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break
                continue

            # only step the controller when a fresh end-effector pose arrived:
            # exactly one controller update per 0.02 s simulation tick, which
            # reproduces the deterministic pure-loop behaviour (repeating the
            # same pose at 50 Hz makes the internal velocity estimate read 0)
            if self._pose_seq != last_pose_seq:
                last_pose_seq = self._pose_seq
                try:
                    q = np.asarray(state.get("q", [0.0, 0.0]),
                                   dtype=np.float64)
                    dq = np.asarray(state.get("dq", [0.0, 0.0]),
                                    dtype=np.float64)
                    ee = np.asarray(state.get("ee", [0.0, 0.0]),
                                    dtype=np.float64)
                    target = np.asarray(state.get("target", [0.0, 0.0]),
                                        dtype=np.float64)
                    self._sim_time = float(state.get("time", 0.0))
                    if (self._target is not None
                            and now - self._target_stamp < 0.3):
                        target = self._target.copy()
                    pose_ee = np.array([ee[0], ee[1], 0.0, 0.0],
                                       dtype=np.float64)
                    if (self._pose_ee is not None
                            and now - self._pose_stamp < 0.5):
                        pose_ee = self._pose_ee.copy()
                    if (self._joints is not None
                            and now - self._joints_stamp < 0.5):
                        q, dq = self._joints[0].copy(), self._joints[1].copy()
                    goal = np.array([target[0], target[1], 0.0],
                                    dtype=np.float64)
                    cmd = self._command(pose_ee, q, dq, goal)
                except Exception as exc:  # pragma: no cover - defensive
                    rospy.logwarn_throttle(
                        2.0, "nav_node: controller update failed: %s", exc)
                    cmd = np.zeros(N_MUSCLES)

            if self._success:
                # keep the controller running: its PD/synergy loop holds the
                # reached pose, which looks (and measures) far better than
                # relaxing the muscles the instant the target is counted.
                if not self._success_reported:
                    self._success_reported = True
                    rospy.loginfo("nav_node: task SUCCESS reported by simulator")
                    self._publish_status("SUCCESS")

            msg = Float32MultiArray()
            msg.data = [float(v) for v in cmd]
            self.pub_cmd.publish(msg)

            if now - last_plan >= 0.5:
                last_plan = now
                self._publish_plan()
            if now - last_status >= 1.0:
                last_status = now
                self._publish_status("SUCCESS" if self._success else "RUNNING",
                                     {"cmd": [round(float(v), 3) for v in cmd]})
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _on_shutdown(self) -> None:
        try:
            self.pub_cmd.publish(self._zero)
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
