#!/usr/bin/env python3
"""HNN humanoid metrics node: records the mission and writes a JSON report.

Subscribes the ground-truth ``/humanoid/state`` stream (plus ``/humanoid/scan``
and ``/humanoid/odom`` for rate statistics), records the trajectory, and when
the robot is within ``~goal_radius`` of the goal - or after ``~timeout``
seconds - writes::

    results/ros_<scene>_<mode>.json

with ``success``, ``elapsed_s``, ``collisions``, ``falls``, ``final_error_m``,
``path_length_m``, ``steps`` and the measured topic rates.  A one-line summary
is printed to stdout for the bringup log.
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

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from humanoid_walk_nn import config as C
from humanoid_walk_nn.scenes import get_scene

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
    """Mission recorder / benchmark report writer."""

    def __init__(self) -> None:
        self.scene_name = str(rospy.get_param("~scene", "hall"))
        self.mode = str(rospy.get_param("~mode", "teleop")).lower()
        self.timeout = float(rospy.get_param("~timeout", 120.0))
        self.goal_radius = float(rospy.get_param("~goal_radius", 0.6))
        self.shutdown_when_done = bool(rospy.get_param("~shutdown", False))
        self.out_dir = str(rospy.get_param("~out_dir", RESULTS_DIR))

        scene = get_scene(self.scene_name)
        self.goal_xy = (self._float_param("~goal_x", scene.goal[0]),
                        self._float_param("~goal_y", scene.goal[1]))

        self.stats = {
            "/humanoid/state": _TopicStats(),
            "/humanoid/scan": _TopicStats(),
            "/humanoid/odom": _TopicStats(),
        }
        self._trail = []
        self._first_time = None
        self._last_time = None
        self._collisions = 0
        self._falls = 0
        self._steps = 0
        self._reached = False
        self._written = False
        self._report_path = ""
        self._start_wall = time.time()

        self.pub_status = rospy.Publisher("/humanoid/status", String,
                                          queue_size=5)
        rospy.Subscriber("/humanoid/state", String, self._on_state,
                         queue_size=5)
        rospy.Subscriber("/humanoid/scan", LaserScan, self._on_scan,
                         queue_size=2)
        rospy.Subscriber("/humanoid/odom", Odometry, self._on_odom,
                         queue_size=5)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("metrics_node: scene=%s mode=%s goal=(%.2f, %.2f) "
                      "timeout=%.1f", self.scene_name, self.mode,
                      self.goal_xy[0], self.goal_xy[1], self.timeout)

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

    # ------------------------------------------------------------- callbacks
    def _on_state(self, msg: String) -> None:
        self.stats["/humanoid/state"].tick()
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        truth = data.get("truth")
        if not isinstance(truth, list) or len(truth) < 4:
            return
        t = float(data.get("time", 0.0))
        x, y, _z, yaw = [float(v) for v in truth[:4]]
        speed = float(data.get("speed", 0.0) or 0.0)
        self._collisions = max(self._collisions, int(data.get("collisions", 0)))
        self._falls = max(self._falls, int(data.get("falls", 0)))
        try:
            self._steps = max(self._steps, int(data.get("steps", 0)))
        except (TypeError, ValueError):
            pass
        if self._first_time is None:
            self._first_time = t
        self._last_time = t
        if (not self._trail or
                math.hypot(x - self._trail[-1][1], y - self._trail[-1][2]) > 0.03):
            self._trail.append((t, x, y, yaw, speed))
        dist = math.hypot(self.goal_xy[0] - x, self.goal_xy[1] - y)
        if dist <= self.goal_radius:
            self._reached = True

    def _on_scan(self, msg: LaserScan) -> None:
        self.stats["/humanoid/scan"].tick()

    def _on_odom(self, msg: Odometry) -> None:
        self.stats["/humanoid/odom"].tick()

    # ---------------------------------------------------------------- report
    def _path_length(self) -> float:
        total = 0.0
        for a, b in zip(self._trail[:-1], self._trail[1:]):
            total += math.hypot(b[1] - a[1], b[2] - a[2])
        return float(total)

    def _final_error(self) -> float:
        if not self._trail:
            return float("nan")
        last = self._trail[-1]
        return float(math.hypot(self.goal_xy[0] - last[1],
                                self.goal_xy[1] - last[2]))

    def _write_report(self, reason: str) -> str:
        if self._written:
            return self._report_path
        self._written = True
        elapsed = 0.0
        if self._first_time is not None and self._last_time is not None:
            elapsed = max(0.0, self._last_time - self._first_time)
        final_error = self._final_error()
        success = bool(self._reached or
                       (not math.isnan(final_error)
                        and final_error <= self.goal_radius))
        total_steps = max(1, len(self._trail) // 600 + 1)
        trail = [[round(v, 3) for v in row] for row in self._trail[::total_steps]]
        report = {
            "scene": self.scene_name,
            "mode": self.mode,
            "success": success,
            "elapsed_s": round(float(elapsed), 2),
            "collisions": int(self._collisions),
            "falls": int(self._falls),
            "final_error_m": (None if math.isnan(final_error)
                              else round(final_error, 3)),
            "path_length_m": round(self._path_length(), 3),
            "goal": {"x": round(self.goal_xy[0], 3),
                     "y": round(self.goal_xy[1], 3),
                     "radius": self.goal_radius},
            "steps": len(self._trail),
            "gait_steps": int(self._steps),
            "topics": {name: st.as_dict() for name, st in self.stats.items()},
            "trajectory": trail,
            "trajectory_columns": ["t", "x", "y", "yaw", "speed"],
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
            "falls": report["falls"],
            "final_error_m": report["final_error_m"],
            "path_length_m": report["path_length_m"],
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
            "reached": bool(self._reached),
            "steps": len(self._trail),
            "report": self._report_path,
        }
        self.pub_status.publish(String(data=json.dumps(payload,
                                                       separators=(",", ":"))))

    # -------------------------------------------------------------------- run
    def spin(self) -> None:
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            if not self._written:
                reached = self._reached
                timeout = (self.timeout > 0.0
                           and self._last_time is not None
                           and self._last_time >= self.timeout)
                if reached or timeout:
                    reason = "goal_reached" if reached else "timeout"
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
