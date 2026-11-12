#!/usr/bin/env python3
"""RVNN metrics node: records the mission and writes a JSON report.

Subscribes the ground-truth ``/rov/state`` stream (plus ``/rov/scan``,
``/rov/odom`` and ``/rov/depth`` for rate statistics), records the trajectory,
and when the ROV is within ``~goal_radius`` of the goal - or after
``~timeout`` seconds - writes::

    results/ros_<scene>_<mode>.json

with ``success``, ``elapsed_s``, ``collisions``, ``final_error_m``,
``path_length_m`` and the measured topic rates.  A one-line summary is printed
to stdout for the bringup log.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time


def _repo_root() -> str:
    env = os.environ.get("RVNN_ROOT", "")
    if env:
        cand = os.path.realpath(env)
        if os.path.isfile(os.path.join(cand, "rov_numpy_nn_slam", "runtime.py")):
            return cand
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.dirname(here), here):
        if os.path.isfile(os.path.join(cand, "rov_numpy_nn_slam", "runtime.py")):
            return cand
    for entry in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if not entry:
            continue
        pkg = os.path.join(entry, "rov_numpy_nn_slam")
        if os.path.isfile(os.path.join(pkg, "package.xml")):
            return os.path.dirname(pkg)
    return os.path.dirname(here)


ROOT = _repo_root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from rov_numpy_nn_slam.runtime import ensure_venv

ensure_venv(__file__)

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String

from rov_numpy_nn_slam import config as C
from rov_numpy_nn_slam.scenes import get_scene


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
        self.scene_name = str(rospy.get_param("~scene", "pool"))
        self.mode = str(rospy.get_param("~mode", "teleop")).lower()
        self.timeout = float(rospy.get_param("~timeout", 90.0))
        self.goal_radius = float(rospy.get_param("~goal_radius", 0.7))
        self.depth_radius = float(rospy.get_param("~depth_radius", 0.6))
        self.shutdown_when_done = bool(rospy.get_param("~shutdown", False))
        self.out_dir = str(rospy.get_param("~out_dir", C.RESULTS_DIR))

        scene = get_scene(self.scene_name)
        self.goal_xy = (self._float_param("~goal_x", scene.goal[0]),
                        self._float_param("~goal_y", scene.goal[1]))
        self.goal_depth = self._float_param("~goal_depth", scene.goal[2])

        self.stats = {
            "/rov/state": _TopicStats(),
            "/rov/scan": _TopicStats(),
            "/rov/odom": _TopicStats(),
            "/rov/depth": _TopicStats(),
        }
        self._trail = []
        self._first_time = None
        self._last_time = None
        self._collisions = 0
        self._reached = False
        self._written = False
        self._report_path = ""
        self._start_wall = time.time()

        self.pub_status = rospy.Publisher("/rov/status", String, queue_size=5)
        rospy.Subscriber("/rov/state", String, self._on_state, queue_size=5)
        rospy.Subscriber("/rov/scan", LaserScan, self._on_scan, queue_size=2)
        rospy.Subscriber("/rov/odom", Odometry, self._on_odom, queue_size=5)
        rospy.Subscriber("/rov/depth", Float64, self._on_depth, queue_size=5)
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
        self.stats["/rov/state"].tick()
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        truth = data.get("truth")
        if not isinstance(truth, list) or len(truth) < 4:
            return
        t = float(data.get("time", 0.0))
        x, y, depth, yaw = [float(v) for v in truth[:4]]
        self._collisions = max(self._collisions, int(data.get("collisions", 0)))
        if self._first_time is None:
            self._first_time = t
        self._last_time = t
        if (not self._trail or
                math.hypot(x - self._trail[-1][1], y - self._trail[-1][2]) > 0.05):
            self._trail.append((t, x, y, depth, yaw))
        dist = math.hypot(self.goal_xy[0] - x, self.goal_xy[1] - y)
        if dist <= self.goal_radius and abs(depth - self.goal_depth) <= self.depth_radius:
            self._reached = True

    def _on_scan(self, msg: LaserScan) -> None:
        self.stats["/rov/scan"].tick()

    def _on_odom(self, msg: Odometry) -> None:
        self.stats["/rov/odom"].tick()

    def _on_depth(self, msg: Float64) -> None:
        self.stats["/rov/depth"].tick()

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
        success = bool(self._reached or final_error <= self.goal_radius)
        total_steps = max(1, len(self._trail) // 600 + 1)
        trail = [[round(v, 3) for v in row] for row in self._trail[::total_steps]]
        report = {
            "scene": self.scene_name,
            "mode": self.mode,
            "success": success,
            "elapsed_s": round(float(elapsed), 2),
            "collisions": int(self._collisions),
            "final_error_m": (None if math.isnan(final_error)
                              else round(final_error, 3)),
            "path_length_m": round(self._path_length(), 3),
            "goal": {"x": round(self.goal_xy[0], 3),
                     "y": round(self.goal_xy[1], 3),
                     "depth": round(self.goal_depth, 3),
                     "radius": self.goal_radius},
            "steps": len(self._trail),
            "topics": {name: st.as_dict() for name, st in self.stats.items()},
            "trajectory": trail,
            "trajectory_columns": ["t", "x", "y", "depth", "yaw"],
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
