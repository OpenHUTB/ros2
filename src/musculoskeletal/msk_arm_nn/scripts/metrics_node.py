#!/usr/bin/env python3
"""MSK-Arm metrics node: records the reaching mission and writes a JSON report.

Subscribes the ground-truth ``/msk/state`` stream (plus ``/msk/joints`` and
``/msk/odom`` for rate statistics), records the end-effector trajectory, and
when the task reports ``success`` / ``reached`` - or after ``~timeout``
simulation seconds - writes::

    results/ros_<scene>_<mode>.json

with ``success``, ``elapsed_s``, ``collisions``, ``reached``,
``final_error_m``, ``max_torque``, ``effort`` and the measured topic rates.
"""

from __future__ import annotations

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

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from msk_arm_nn import config as C
from msk_arm_nn.scenes import get_scene

RESULTS_DIR = str(getattr(C, "RESULTS_DIR", os.path.join(ROOT, "results")))


class _TopicStats:
    """Receive-time rate estimator for one ROS topic."""

    __slots__ = ("count", "first", "last")

    def __init__(self) -> None:
        self.count = 0
        self.first = 0.0
        self.last = 0.0

    def tick(self) -> None:
        now = time.time()
        if self.count == 0:
            self.first = now
        self.last = now
        self.count += 1

    def as_dict(self) -> dict:
        hz = 0.0
        if self.count > 1 and self.last > self.first:
            hz = (self.count - 1) / (self.last - self.first)
        return {"count": int(self.count), "hz": round(hz, 2)}


class MetricsNode:
    """Reaching-task recorder / report writer."""

    def __init__(self) -> None:
        self.scene_name = str(rospy.get_param("~scene", "reach"))
        self.mode = str(rospy.get_param("~mode", "reach")).lower()
        self.timeout = float(rospy.get_param("~timeout", 60.0))
        self.shutdown_when_done = bool(rospy.get_param("~shutdown", False))
        self.out_dir = str(rospy.get_param("~out_dir", RESULTS_DIR))

        self.scene = get_scene(self.scene_name)
        self.stats = {
            "/msk/state": _TopicStats(),
            "/msk/joints": _TopicStats(),
            "/msk/odom": _TopicStats(),
        }
        self._trail = []
        self._first_time = None
        self._last_time = None
        self._collisions = 0
        self._reached = 0
        self._success = False
        self._max_torque = 0.0
        self._effort = 0.0
        self._target = list(self.scene.targets[0])
        self._ee = [0.0, 0.0]
        self._success_ee = None
        self._success_target = None
        self._written = False
        self._report_path = ""
        self._start_wall = time.time()

        self.pub_status = rospy.Publisher("/msk/status", String, queue_size=5)
        rospy.Subscriber("/msk/state", String, self._on_state, queue_size=5)
        rospy.Subscriber("/msk/joints", JointState, self._on_joints,
                         queue_size=5)
        rospy.Subscriber("/msk/odom", Odometry, self._on_odom, queue_size=5)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("metrics_node: scene=%s mode=%s timeout=%.1f",
                      self.scene_name, self.mode, self.timeout)

    # ------------------------------------------------------------- callbacks
    def _on_state(self, msg: String) -> None:
        self.stats["/msk/state"].tick()
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        t = float(data.get("time", 0.0))
        self._collisions = max(self._collisions, int(data.get("collisions", 0)))
        self._reached = max(self._reached, int(data.get("reached", 0)))
        self._success = bool(data.get("success", False))
        try:
            self._max_torque = max(self._max_torque,
                                   float(data.get("max_torque", 0.0) or 0.0))
            self._effort = max(self._effort,
                               float(data.get("effort", 0.0) or 0.0))
        except (TypeError, ValueError):
            pass
        ee = data.get("ee")
        target = data.get("target")
        if isinstance(target, list) and len(target) >= 2:
            try:
                self._target = [float(target[0]), float(target[1])]
            except (TypeError, ValueError):
                pass
        if isinstance(ee, list) and len(ee) >= 2:
            try:
                x, y = float(ee[0]), float(ee[1])
            except (TypeError, ValueError):
                return
            self._ee = [x, y]
            if (not self._trail or
                    math.hypot(x - self._trail[-1][1],
                               y - self._trail[-1][2]) > 0.001):
                self._trail.append((t, x, y))
        if self._first_time is None:
            self._first_time = t
        self._last_time = t
        # freeze the goal / end-effector pair at the moment the task succeeds:
        # after success the arm relaxes, so a shutdown-time error would be
        # misleading (the report should state the accuracy that earned success)
        if self._success and self._success_ee is None:
            self._success_ee = list(self._ee)
            self._success_target = list(self._target)

    def _on_joints(self, msg: JointState) -> None:
        self.stats["/msk/joints"].tick()

    def _on_odom(self, msg: Odometry) -> None:
        self.stats["/msk/odom"].tick()

    # ---------------------------------------------------------------- report
    def _path_length(self) -> float:
        total = 0.0
        for a, b in zip(self._trail[:-1], self._trail[1:]):
            total += math.hypot(b[1] - a[1], b[2] - a[2])
        return float(total)

    def _final_error(self) -> float:
        ee = self._success_ee if self._success_ee is not None else self._ee
        target = (self._success_target if self._success_target is not None
                  else self._target)
        return float(math.hypot(target[0] - ee[0], target[1] - ee[1]))

    def _write_report(self, reason: str) -> str:
        if self._written:
            return self._report_path
        self._written = True
        elapsed = 0.0
        if self._first_time is not None and self._last_time is not None:
            elapsed = max(0.0, self._last_time - self._first_time)
        final_error = self._final_error()
        success = bool(self._success or
                       final_error <= float(getattr(C, "TARGET_TOLERANCE",
                                                    0.05)))
        total_steps = max(1, len(self._trail) // 600 + 1)
        trail = [[round(v, 4) for v in row] for row in self._trail[::total_steps]]
        report = {
            "scene": self.scene_name,
            "mode": self.mode,
            "success": success,
            "elapsed_s": round(float(elapsed), 2),
            "collisions": int(self._collisions),
            "reached": int(self._reached),
            "final_error_m": round(final_error, 4),
            "path_length_m": round(self._path_length(), 4),
            "max_torque": round(float(self._max_torque), 3),
            "effort": round(float(self._effort), 3),
            "target": [round(float(v), 4) for v in self._target],
            "final_ee": [round(float(self._ee[0]), 4),
                         round(float(self._ee[1]), 4)],
            "targets_total": len(getattr(self.scene, "targets", ())),
            "topics": {name: st.as_dict() for name, st in self.stats.items()},
            "trajectory": trail,
            "trajectory_columns": ["t", "ee_x", "ee_y"],
            "finished_by": reason,
        }
        try:
            os.makedirs(self.out_dir, exist_ok=True)
            path = os.path.join(self.out_dir,
                                "ros_%s_%s.json" % (self.scene_name, self.mode))
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(report, fh, indent=2, sort_keys=True)
            self._report_path = path
        except Exception as exc:  # pragma: no cover - defensive
            rospy.logerr("metrics_node: cannot write report: %s", exc)
            self._report_path = ""
        summary = {
            "event": "ros_metrics",
            "scene": self.scene_name,
            "mode": self.mode,
            "success": success,
            "elapsed_s": report["elapsed_s"],
            "collisions": report["collisions"],
            "reached": report["reached"],
            "final_error_m": report["final_error_m"],
            "report": self._report_path,
        }
        print(json.dumps(summary, separators=(",", ":")))
        sys.stdout.flush()
        rospy.loginfo("metrics_node: report -> %s", self._report_path)
        return self._report_path

    def _publish_status(self, status: str) -> None:
        payload = {
            "node": "metrics",
            "status": status,
            "scene": self.scene_name,
            "mode": self.mode,
            "success": bool(self._success),
            "reached": int(self._reached),
            "report": self._report_path,
        }
        self.pub_status.publish(String(data=json.dumps(payload,
                                                       separators=(",", ":"))))

    # -------------------------------------------------------------------- run
    def spin(self) -> None:
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            if not self._written:
                timeout = (self.timeout > 0.0
                           and self._last_time is not None
                           and self._last_time >= self.timeout)
                if self._success or timeout:
                    reason = "success" if self._success else "timeout"
                    self._write_report(reason)
                    self._publish_status("DONE")
                    if self.shutdown_when_done:
                        rospy.signal_shutdown("metrics complete")
                        break
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _on_shutdown(self) -> None:
        if not self._written and self._trail:
            self._write_report("shutdown")
        try:
            self._publish_status("SHUTDOWN")
        except Exception:
            pass


def main() -> int:
    rospy.init_node("metrics_node", anonymous=False)
    node = MetricsNode()
    try:
        node.spin()
    finally:
        node._on_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
