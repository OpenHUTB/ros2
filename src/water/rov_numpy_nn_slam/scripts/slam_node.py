#!/usr/bin/env python3
"""RVNN SLAM node: sonar + drifting odometry -> occupancy grid + pose.

Subscribes
    /rov/scan   (sensor_msgs/LaserScan, 64 beams)
    /rov/odom   (nav_msgs/Odometry, drifting DVL/IMU)

Publishes
    /rov/map        (nav_msgs/OccupancyGrid, resolution ``~resolution``)
    /rov/slam_pose  (geometry_msgs/PoseStamped)
    /rov/status     (std_msgs/String, JSON)

The heavy lifting lives in ``rov_numpy_nn_slam.slam`` (owned by the navigation
engineer); if that module is missing the node still runs and reports its state
on ``/rov/status`` instead of taking the whole bringup down.
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

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from rov_numpy_nn_slam import config as C
from rov_numpy_nn_slam.geometry import wrap_angle
from rov_numpy_nn_slam.scenes import get_scene

try:  # independent module owned by nav-eng
    from rov_numpy_nn_slam.slam import OccupancyGridMap, SlamFrontend
    SLAM_AVAILABLE = True
    SLAM_ERROR = ""
except Exception as _exc:  # pragma: no cover - concurrent development
    OccupancyGridMap = None
    SlamFrontend = None
    SLAM_AVAILABLE = False
    SLAM_ERROR = str(_exc)


def _yaw_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0,
                      z=math.sin(0.5 * float(yaw)),
                      w=math.cos(0.5 * float(yaw)))


def _yaw_from_quaternion(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SlamNode:
    """Occupancy-grid SLAM wrapper around :class:`rov_numpy_nn_slam.slam.SlamFrontend`."""

    def __init__(self) -> None:
        self.scene_name = str(rospy.get_param("~scene", "pool"))
        self.resolution = float(rospy.get_param("~resolution",
                                                C.MAP_RESOLUTION))
        self.publish_hz = max(0.5, float(rospy.get_param("~publish_hz", 2.0)))
        self.stale_timeout = max(0.1, float(rospy.get_param("~stale_timeout",
                                                            1.0)))
        scan_topic = str(rospy.get_param("~scan_topic", "/rov/scan"))
        odom_topic = str(rospy.get_param("~odom_topic", "/rov/odom"))

        self.scene = get_scene(self.scene_name)
        self.frontend = None
        self._odom = None
        self._odom_stamp = -1e9
        self._pose = np.zeros(4)
        self._scans = 0
        self._updates = 0
        self._errors = 0

        if SLAM_AVAILABLE:
            try:
                self.frontend = SlamFrontend(self.scene)
                current = float(self.frontend.map.resolution)
                if abs(current - self.resolution) > 1e-9 and OccupancyGridMap:
                    self.frontend.map = OccupancyGridMap(
                        self.scene.width, self.scene.height, self.resolution)
                rospy.loginfo("slam_node: SlamFrontend ready (%dx%d cells @ %.3f m)",
                              self.frontend.map.width, self.frontend.map.height,
                              self.frontend.map.resolution)
            except Exception as exc:  # pragma: no cover - defensive
                self.frontend = None
                rospy.logerr("slam_node: SlamFrontend failed: %s", exc)
        else:
            rospy.logwarn("slam_node: rov_numpy_nn_slam.slam unavailable (%s) - "
                          "publishing status only", SLAM_ERROR)

        self.pub_map = rospy.Publisher("/rov/map", OccupancyGrid, queue_size=2)
        self.pub_pose = rospy.Publisher("/rov/slam_pose", PoseStamped,
                                        queue_size=5)
        self.pub_status = rospy.Publisher("/rov/status", String, queue_size=5)

        rospy.Subscriber(scan_topic, LaserScan, self._on_scan, queue_size=2)
        rospy.Subscriber(odom_topic, Odometry, self._on_odom, queue_size=5)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("slam_node: scene=%s resolution=%.3f publish_hz=%.1f",
                      self.scene_name, self.resolution, self.publish_hz)

    # ------------------------------------------------------------- callbacks
    def _on_odom(self, msg) -> None:
        q = msg.pose.pose.orientation
        self._odom = np.array([float(msg.pose.pose.position.x),
                               float(msg.pose.pose.position.y),
                               -float(msg.pose.pose.position.z),
                               _yaw_from_quaternion(q)], dtype=np.float64)
        self._odom_stamp = time.time()

    def _on_scan(self, msg: LaserScan) -> None:
        self._scans += 1
        if self.frontend is None:
            return
        if self._odom is None or time.time() - self._odom_stamp > self.stale_timeout:
            rospy.logwarn_throttle(5.0, "slam_node: no fresh /rov/odom, skipping scan")
            return
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        try:
            pose = self.frontend.update(self._odom, ranges)
        except Exception as exc:  # pragma: no cover - defensive
            self._errors += 1
            rospy.logwarn_throttle(2.0, "slam_node: update failed: %s", exc)
            return
        self._pose = np.asarray(pose, dtype=np.float64)
        self._updates += 1
        self._publish_pose(msg.header.stamp)

    # ------------------------------------------------------------ publishing
    def _publish_pose(self, stamp) -> None:
        pose = self._pose
        msg = PoseStamped()
        msg.header.stamp = stamp if stamp is not None else rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position = Point(x=float(pose[0]), y=float(pose[1]),
                                  z=-float(pose[2]))
        msg.pose.orientation = _yaw_quaternion(float(pose[3]))
        self.pub_pose.publish(msg)

    def _publish_map(self) -> None:
        if self.frontend is None:
            return
        grid = self.frontend.map
        try:
            log_odds = np.asarray(grid.log_odds, dtype=np.float64)
            prob = np.asarray(grid.probability, dtype=np.float64)
            unknown = np.abs(log_odds) < 1e-6
            data = np.where(unknown, -1.0, np.round(prob * 100.0))
            data = np.clip(data, -1.0, 100.0).astype(np.int8)
        except Exception as exc:  # pragma: no cover - defensive
            rospy.logwarn_throttle(2.0, "slam_node: map conversion failed: %s",
                                   exc)
            return
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = float(grid.resolution)
        msg.info.width = int(grid.width)
        msg.info.height = int(grid.height)
        msg.info.origin.position = Point(x=0.0, y=0.0, z=0.0)
        msg.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        msg.data = data.ravel().tolist()
        self.pub_map.publish(msg)

    def _publish_status(self) -> None:
        payload = {
            "node": "slam",
            "status": "OK" if self.frontend is not None else "UNAVAILABLE",
            "scene": self.scene_name,
            "updates": int(self._updates),
            "scans": int(self._scans),
            "errors": int(self._errors),
            "pose": [round(float(v), 4) for v in self._pose],
        }
        if self.frontend is not None:
            payload["map"] = [int(self.frontend.map.width),
                              int(self.frontend.map.height)]
            payload["resolution"] = float(self.frontend.map.resolution)
            payload["trajectory"] = len(self.frontend.trajectory)
        else:
            payload["error"] = SLAM_ERROR
        self.pub_status.publish(String(data=json.dumps(payload,
                                                       separators=(",", ":"))))

    # -------------------------------------------------------------------- run
    def spin(self) -> None:
        rate = rospy.Rate(self.publish_hz)
        status_timer = 0.0
        while not rospy.is_shutdown():
            if self.frontend is not None:
                self._publish_map()
            status_timer += 1.0 / self.publish_hz
            if status_timer >= 1.0:
                status_timer = 0.0
                self._publish_status()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _on_shutdown(self) -> None:
        try:
            self._publish_status()
        except Exception:
            pass


def main() -> int:
    rospy.init_node("slam_node", anonymous=False)
    node = SlamNode()
    try:
        node.spin()
    finally:
        node._on_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
